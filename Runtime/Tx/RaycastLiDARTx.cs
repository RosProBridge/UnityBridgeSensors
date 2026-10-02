using System;
using System.Collections.Generic;
using UnitySensors.Sensor.LiDAR;
using sensor_msgs.msg;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
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
        [Tooltip("Share of the pattern points skipped in every scan.")]
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

        private int _pointsNum;
        private int _stride;
        private int _start;
        private uint _noiseSeed = 1;

#if ROS_V2 && PROBRIDGE_DEFAULT_QOS // com.ars.probridge >= 3.6.0
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
            _stride = Mathf.Max(1, Mathf.RoundToInt(1f / (1f - downSampleScale)));
            _pointsNum = (_directions.Length + _stride - 1) / _stride;
            _start = 0;

            _commands = new NativeArray<RaycastCommand>(_pointsNum, Allocator.Persistent);
            _hits = new NativeArray<RaycastHit>(_pointsNum, Allocator.Persistent);
            _points = new NativeArray<PointXYZI>(_pointsNum, Allocator.Persistent);
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

            ScanPoints();

            var filtered = new NativeQueue<PointXYZI>(Allocator.TempJob);
            new FilterZeroPointsParallelJob
            {
                inputArray = _points,
                outputQueue = filtered.AsParallelWriter()
            }.Schedule(_pointsNum, 64).Complete();

            int count = filtered.Count;
            data.is_bigendian = false;
            data.width = (uint)count;
            data.height = 1;
            data.point_step = CalculateFieldsSize();
            data.row_step = data.width * data.point_step;
            data.is_dense = true;

            var filteredPoints = filtered.ToArray(Allocator.TempJob);
            var bytes = new NativeArray<byte>((int)(data.row_step * data.height), Allocator.TempJob);
            new PointsToPointCloud2MsgJob
            {
                points = filteredPoints,
                data = bytes,
                _includeIntensity = _includeIntensity
            }.Schedule(count, 64).Complete();

            data.data = bytes.ToArray();

            filtered.Dispose();
            filteredPoints.Dispose();
            bytes.Dispose();

            return base.GetMsg(ts);
        }

        private void ScanPoints()
        {
            if (_reflectivityVersion != LidarReflectivity.Version)
            {
                _reflectivities.Clear();
                _reflectivityVersion = LidarReflectivity.Version;
            }

            var window = new ScanWindow
            {
                start = _start,
                stride = patternShift == PatternShift.Sequential ? 1 : _stride,
                patternSize = _directions.Length
            };
            quaternion rotation = transform.rotation;

            var buildCommands = new BuildRaycastCommandsJob
            {
                directions = _directions,
                window = window,
                origin = transform.position,
                rotation = rotation,
                maxRange = _maxRange,
                commands = _commands
            };

            var hitsToPoints = new RaycastHitsToPointsJob
            {
                directions = _directions,
                hits = _hits,
                reflectivities = _reflectivities,
                unknownColliders = _unknownColliders.AsParallelWriter(),
                window = window,
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

            var handle = buildCommands.Schedule(_pointsNum, 64);
            handle = RaycastCommand.ScheduleBatch(_commands, _hits, 64, handle);
            handle = hitsToPoints.Schedule(_pointsNum, 64, handle);
            handle.Complete();

            ResolveUnknownColliders();

            _noiseSeed += (uint)_pointsNum;
            switch (patternShift)
            {
                case PatternShift.None:
                    break;
                case PatternShift.Interleaved:
                    _start = (_start + 1) % _stride;
                    break;
                case PatternShift.Sequential:
                    _start = (_start + _pointsNum) % _directions.Length;
                    break;
            }
        }

        /// <summary>
        /// Colliders hit for the first time got the default reflectivity in this scan; resolve them for the next ones.
        /// </summary>
        private void ResolveUnknownColliders()
        {
            if (_unknownColliders.IsEmpty)
                return;

            using (var ids = _unknownColliders.ToNativeArray(Allocator.Temp))
            {
                foreach (var id in ids)
                {
                    _reflectivities[id] = LidarReflectivityResolver.Resolve(id.ToCollider(), defaultReflectivity);
                }
            }

            _unknownColliders.Clear();
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
