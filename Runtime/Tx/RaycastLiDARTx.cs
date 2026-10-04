using System;
using System.Collections.Generic;
using UnitySensors.Sensor.LiDAR;
using sensor_msgs.msg;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine;
using UnitySensors.Data.PointCloud;


namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/RaycastLiDAR")]
    public class RaycastLiDARTx : ProBridgeTxStamped<PointCloud2>
    {
        public enum PatternShift
        {
            [Tooltip("The same pattern points every scan.")]
            None,

            [Tooltip("Every scan takes the next interleaved subset (every N-th point with a shifting phase): " +
                     "full field of view each scan, accumulating fills the whole pattern.")]
            Interleaved,

            [Tooltip("Every scan takes the next consecutive part of the pattern, " +
                     "like a time-ordered Livox pattern played back in real time.")]
            Sequential
        }

        [Header("Lidar Params")] public ScanPattern _scanPattern;
        public float _minRange = 0.5f;
        public float _maxRange = 100.0f;
        public float _gaussianNoiseSigma = 0.0f;
        public float minAzimuthAngle = 0;
        public float maxAzimuthAngle = 360f;
        [Tooltip("Points per second, as in the lidar datasheet (Livox Mid-70: 100000, Mid-360: 200000, " +
                 "Horizon: 240000, Velodyne VLP-16: 300000). Points per scan = this * Send Rate, " +
                 "reduced in proportion to the azimuth crop (the density stays). 0: use Down Sample Scale.")]
        [Min(0)] public int pointsPerSecond = 0;
        [Tooltip("Share of the pattern points skipped in every scan (when Points Per Second is 0).")]
        [Range(0f, 0.99f)] public float downSampleScale = 0.9f;
        [Tooltip("How the skipped points change from scan to scan (non-repeating pattern).")]
        public PatternShift patternShift = PatternShift.None;

        [Header("Intensity")]
        public bool _includeIntensity = true;
        [Tooltip("Intensity of a retroreflector at zero range.")]
        public float _maxIntensity = 255.0f;
        [Tooltip("Intensity of a 100% diffuse surface hit straight on at zero range. " +
                 "Livox: 150, Velodyne: 100; values above are left to retroreflectors.")]
        public float diffuseMaxIntensity = 150.0f;
        [Tooltip("Reflectivity of colliders without LidarReflectivity and without a material.")]
        [Range(0f, 1f)] public float defaultReflectivity = LidarReflectivity.DefaultValue;
        [Tooltip("Intensity drop at max range: 0 = none (calibrated reflectivity), 1 = down to zero.")]
        [Range(0f, 1f)] public float rangeFalloff = 0.2f;

        private NativeArray<float3> _directions;
        private NativeArray<RaycastCommand> _commands;
        private NativeArray<RaycastHit> _hits;
        private NativeArray<PointXYZI> _points;
        private NativeParallelHashMap<LidarColliderId, float> _reflectivities;
        private NativeParallelHashSet<LidarColliderId> _unknownColliders;
        private int _reflectivityVersion = -1;

        private NativeArray<float> _packed;
        private NativeArray<int> _packedCount;

        private int _pointsNum;   // points per scan
        private int _columnSize;  // pattern points sharing one azimuth (VLP-16: 16, Livox: 1)
        private int _stride;      // in columns
        private int _start;       // in columns
        private uint _noiseSeed = 1;

        // A scan is cast in slices, one per physics step between two sends (like a real lidar scanning over time).
        // Each slice is chained after the previous one and the chain is completed only when the scan is sent,
        // so the main thread doesn't wait for the rays (also when several physics steps run in one frame).
        private int _slicesPerScan;
        private int _slice;        // slices of the current scan scheduled so far
        private JobHandle _sliceHandle;
        private bool _slicePending;

        private const int MaxCollidersResolvedPerScan = 16;

        private static readonly ProfilerMarker ScheduleMarker = new ProfilerMarker("RaycastLiDAR.ScheduleSlice");
        private static readonly ProfilerMarker CompleteMarker = new ProfilerMarker("RaycastLiDAR.CompleteSlice");
        private static readonly ProfilerMarker PackMarker = new ProfilerMarker("RaycastLiDAR.Pack");
        private static readonly ProfilerMarker ResolveMarker = new ProfilerMarker("RaycastLiDAR.ResolveColliders");

#if ROS_V2
        protected override Qos CreateDefaultQos() => new Qos
        {
            qosType = Qos.QOSType.Dict,
            reliability = Qos.Reliability.RELIABLE,
            history = Qos.History.KEEP_LAST,
            depth = 5,
            durability = Qos.Durability.VOLATILE,
            liveliness = Qos.Liveliness.SYSTEM_DEFAULT
        };
#endif

        protected override void AfterEnable()
        {
            if (!_scanPattern)
            {
                Debug.LogError($"[{topic}] Scan pattern is not set.", this);
                enabled = false;
                return;
            }

            var scans = SelectAzimuth(_scanPattern.scans, minAzimuthAngle, maxAzimuthAngle);
            if (scans.Length == 0)
            {
                Debug.LogError($"[{topic}] No pattern points within azimuth {minAzimuthAngle}..{maxAzimuthAngle}.", this);
                enabled = false;
                return;
            }

            _directions = new NativeArray<float3>(scans, Allocator.Persistent);

            // Thinning keeps whole columns, so a rotating lidar keeps all its rings.
            _columnSize = DetectColumnSize(scans);
            int columns = _directions.Length / _columnSize;

            // The datasheet rate is for the full pattern: an azimuth crop keeps the density, not the point count.
            float scanPeriod = sendRate > 0f ? sendRate : Time.fixedDeltaTime;
            float keptShare = (float)_directions.Length / _scanPattern.scans.Length;
            int targetPoints = pointsPerSecond > 0
                ? Mathf.Clamp(Mathf.RoundToInt(pointsPerSecond * scanPeriod * keptShare), _columnSize, _directions.Length)
                : Mathf.CeilToInt(_directions.Length * (1f - downSampleScale));

            if (patternShift == PatternShift.Sequential)
            {
                // Consecutive chunks of a time-ordered pattern (Livox): exactly the requested count
                _stride = 1;
                _pointsNum = Mathf.Max(_columnSize, targetPoints / _columnSize * _columnSize);
            }
            else
            {
                // Every stride-th column over the whole pattern (full field of view in every scan)
                _stride = Mathf.Max(1, Mathf.RoundToInt((float)_directions.Length / targetPoints));
                _pointsNum = (columns + _stride - 1) / _stride * _columnSize;
            }
            _start = 0;

            _slicesPerScan = Mathf.Clamp(Mathf.RoundToInt(scanPeriod / Time.fixedDeltaTime), 1, _pointsNum);
            _slice = 0;
            _slicePending = false;
            int maxSlice = (_pointsNum + _slicesPerScan - 1) / _slicesPerScan;

            _commands = new NativeArray<RaycastCommand>(maxSlice, Allocator.Persistent);
            _hits = new NativeArray<RaycastHit>(maxSlice, Allocator.Persistent);
            _points = new NativeArray<PointXYZI>(_pointsNum, Allocator.Persistent);
            _packed = new NativeArray<float>(_pointsNum * 4, Allocator.Persistent);
            _packedCount = new NativeArray<int>(1, Allocator.Persistent);
            _reflectivities = new NativeParallelHashMap<LidarColliderId, float>(256, Allocator.Persistent);
            _unknownColliders = new NativeParallelHashSet<LidarColliderId>(_pointsNum, Allocator.Persistent);
            _reflectivityVersion = -1;

            data.fields = new PointField[_includeIntensity ? 4 : 3];
            for (int i = 0; i < 3; i++)
            {
                data.fields[i] = new PointField();
                data.fields[i].name = ((char)('x' + i)).ToString();
                data.fields[i].offset = 0;
                data.fields[i].datatype = PointField.FLOAT32;
                data.fields[i].count = 1;
            }

            if (_includeIntensity)
            {
                data.fields[3] = new PointField();
                data.fields[3].name = "intensity";
                data.fields[3].offset = 0;
                data.fields[3].datatype = PointField.FLOAT32;
                data.fields[3].count = 1;
            }

            CalculateFieldsOffset();
        }

        protected override void AfterDisable()
        {
            _sliceHandle.Complete();
            _slicePending = false;

            if (_packed.IsCreated) _packed.Dispose();
            if (_packedCount.IsCreated) _packedCount.Dispose();
            if (_directions.IsCreated) _directions.Dispose();
            if (_commands.IsCreated) _commands.Dispose();
            if (_hits.IsCreated) _hits.Dispose();
            if (_points.IsCreated) _points.Dispose();
            if (_reflectivities.IsCreated) _reflectivities.Dispose();
            if (_unknownColliders.IsCreated) _unknownColliders.Dispose();
        }

        private static float3[] SelectAzimuth(float3[] scans, float minAzimuth, float maxAzimuth)
        {
            // Full-circle case
            if (Math.Abs(maxAzimuth - 360f) < ANGLE_TOLERANCE && Mathf.Abs(minAzimuth) < ANGLE_TOLERANCE)
                return scans;

            float NormalizeSignedAngle(float angle)
            {
                angle %= 360f;
                if (angle > 180f) angle -= 360f;
                if (angle <= -180f) angle += 360f;
                return angle;
            }

            minAzimuth = NormalizeSignedAngle(minAzimuth);
            maxAzimuth = NormalizeSignedAngle(maxAzimuth);

            var selected = new List<float3>();
            foreach (var scan in scans)
            {
                var azimuth = NormalizeSignedAngle(Mathf.Atan2(-scan.x, -scan.z) * Mathf.Rad2Deg + 180f);
                if (azimuth >= minAzimuth && azimuth <= maxAzimuth)
                    selected.Add(scan);
            }

            return selected.ToArray();
        }

        private const double ANGLE_TOLERANCE = 0.001;

        /// <summary>
        /// Number of leading pattern points with the same azimuth (one firing of all rings of a rotating lidar).
        /// 1 if the pattern doesn't split evenly into such columns.
        /// </summary>
        private static int DetectColumnSize(float3[] scans)
        {
            const float Tolerance = 1e-4f;
            float Azimuth(float3 d) => math.atan2(d.x, d.z);

            float first = Azimuth(scans[0]);
            int size = 1;
            while (size < scans.Length && math.abs(Azimuth(scans[size]) - first) < Tolerance)
                size++;

            return size > 1 && size < scans.Length && scans.Length % size == 0 ? size : 1;
        }

        private void CalculateFieldsOffset()
        {
            uint offset = 0;
            foreach (var field in data.fields)
            {
                field.offset = offset;
                offset += GetTypeSize(field) * field.count;
            }
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            if (!_directions.IsCreated)
                return null;

            CompleteSlice();
            ResolveUnknownColliders(); // no jobs run now: the reflectivity table can be updated
            if (_slice == 0)
                return null; // nothing cast since the previous message

            int scanned = SliceStart(_slice); // a late send may come before all slices are cast
            int count;
            using (PackMarker.Auto())
            {
                new PackPointCloud2Job
                {
                    points = _points.GetSubArray(0, scanned),
                    includeIntensity = _includeIntensity,
                    data = _packed,
                    count = _packedCount
                }.Run();
                count = _packedCount[0];

                data.is_bigendian = false;
                data.width = (uint)count;
                data.height = 1;
                data.point_step = CalculateFieldsSize();
                data.row_step = data.width * data.point_step;
                data.is_dense = true;

                int byteCount = (int)data.row_step;
                // Reused buffer of the largest scan, only its filled part is sent
                if (data.data == null || data.data.Length < byteCount)
                    data.data = new byte[_packed.Length * sizeof(float)];
                data.dataLength = byteCount;
                NativeArray<byte>.Copy(_packed.Reinterpret<byte>(sizeof(float)), data.data, byteCount);
            }

            StartNextScan();
            return base.GetMsg(ts);
        }

        // Runs after ProBridgeServer.FixedUpdate (which sends), once per physics step: schedules the next slice
        // after the previous ones, until the scan is complete and waits for the send.
        private void FixedUpdate()
        {
            if (!_directions.IsCreated)
                return;

            if (!IsScanning())
            {
                CompleteSlice();
                _slice = 0;
                return;
            }

            if (_slice < _slicesPerScan)
                ScheduleSlice(_slice++);
        }

        // Same conditions as for sending: no rays while nothing would be sent.
        private bool IsScanning()
        {
            if (!Active || !host || topic == "")
                return false;
            if (!useWithoutConnect && !host.IsConnected)
                return false;
            return true;
        }

        private int SliceStart(int slice) => (int)((long)slice * _pointsNum / _slicesPerScan);

        private void ScheduleSlice(int slice)
        {
            using (ScheduleMarker.Auto())
            {
                // The table is read by the slice jobs: change it only before the first slice of a scan
                if (slice == 0)
                {
                    CompleteSlice();
                    if (_reflectivityVersion != LidarReflectivity.Version)
                    {
                        _reflectivities.Clear();
                        _reflectivityVersion = LidarReflectivity.Version;
                    }
                }

                int offset = SliceStart(slice);
                int count = SliceStart(slice + 1) - offset;
                if (count <= 0)
                    return;

                var window = new ScanWindow
                {
                    start = _start,
                    stride = _stride,
                    columnSize = _columnSize,
                    patternSize = _directions.Length
                };
                quaternion rotation = transform.rotation;
                var commands = _commands.GetSubArray(0, count);
                var hits = _hits.GetSubArray(0, count);

                var buildCommands = new BuildRaycastCommandsJob
                {
                    directions = _directions,
                    window = window,
                    scanOffset = offset,
                    origin = transform.position,
                    rotation = rotation,
                    maxRange = _maxRange,
                    commands = commands
                };

                var hitsToPoints = new RaycastHitsToPointsJob
                {
                    directions = _directions,
                    hits = hits,
                    reflectivities = _reflectivities,
                    unknownColliders = _unknownColliders.AsParallelWriter(),
                    window = window,
                    scanOffset = offset,
                    rotation = rotation,
                    minRange = _minRange,
                    maxRange = _maxRange,
                    noiseSigma = _gaussianNoiseSigma,
                    noiseSeed = _noiseSeed,
                    defaultReflectivity = defaultReflectivity,
                    diffuseMaxIntensity = diffuseMaxIntensity,
                    maxIntensity = _maxIntensity,
                    rangeFalloff = rangeFalloff,
                    points = _points
                };

                // After the previous slice: they share the command and hit buffers
                var handle = buildCommands.Schedule(count, 64, _sliceHandle);
                handle = RaycastCommand.ScheduleBatch(commands, hits, 64, handle);
                _sliceHandle = hitsToPoints.Schedule(count, 64, handle);
                _slicePending = true;
                JobHandle.ScheduleBatchedJobs();
            }
        }

        private void CompleteSlice()
        {
            if (!_slicePending)
                return;
            using (CompleteMarker.Auto())
                _sliceHandle.Complete();
            _slicePending = false;
        }

        private void StartNextScan()
        {
            _slice = 0;
            _noiseSeed += (uint)_pointsNum;
            switch (patternShift)
            {
                case PatternShift.None:
                    break;
                case PatternShift.Interleaved:
                    _start = (_start + 1) % _stride;
                    break;
                case PatternShift.Sequential:
                    _start = (_start + _pointsNum / _columnSize) % (_directions.Length / _columnSize);
                    break;
            }
        }

        /// <summary>
        /// Colliders hit for the first time got the default reflectivity; resolve a few per scan for the next scans
        /// (resolving a new material reads its texture back from the GPU). Called only when no slice job runs.
        /// </summary>
        private void ResolveUnknownColliders()
        {
            if (_unknownColliders.IsEmpty)
                return;

            using (ResolveMarker.Auto())
            using (var ids = _unknownColliders.ToNativeArray(Allocator.Temp))
            {
                int resolved = Mathf.Min(ids.Length, MaxCollidersResolvedPerScan);
                for (int i = 0; i < resolved; i++)
                {
                    var id = ids[i];
                    _reflectivities[id] = LidarReflectivityResolver.Resolve(id.ToCollider(), defaultReflectivity);
                    _unknownColliders.Remove(id);
                }
            }
        }


        private uint CalculateFieldsSize()
        {
            uint size = 0;

            foreach (var field in data.fields)
            {
                uint typeSize;
                typeSize = GetTypeSize(field);

                size += typeSize * field.count;
            }

            return size;
        }

        private uint GetTypeSize(PointField field)
        {
            uint typeSize;
            switch (field.datatype)
            {
                case PointField.INT8:
                    typeSize = 1;
                    break;
                case PointField.UINT8:
                    typeSize = 1;
                    break;
                case PointField.INT16:
                    typeSize = 2;
                    break;
                case PointField.UINT16:
                    typeSize = 2;
                    break;
                case PointField.INT32:
                    typeSize = 4;
                    break;
                case PointField.UINT32:
                    typeSize = 4;
                    break;
                case PointField.FLOAT32:
                    typeSize = 4;
                    break;
                case PointField.FLOAT64:
                    typeSize = 8;
                    break;
                default:
                    throw new InvalidOperationException($"Unsupported data type: {field.datatype}");
            }

            return typeSize;
        }
    }
}
