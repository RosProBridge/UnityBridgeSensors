using System;
using System.Collections.Generic;
using System.Linq;
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
        [Header("Lidar Params")] public ScanPattern _scanPattern;
        public int _pointsNumPerScan = 10000;
        public float _minRange = 0.5f;
        public float _maxRange = 100.0f;
        public float _gaussianNoiseSigma = 0.0f;
        public float _maxIntensity = 255.0f;
        public float minAzimuthAngle = 0;
        public float maxAzimuthAngle = 360f;
        [Range(0f, 1f)] public float downSampleScale = 0.5f;


        private RaycastLiDARSensor sensor;

        private PointsToPointCloud2MsgJob<PointXYZI> _pointsToPointCloud2MsgJob;
        private FilterZeroPointsParallelJob _zeroFilterJob;

        private JobHandle _jobHandle;
        private NativeArray<byte> tempData;
        NativeQueue<PointXYZI> tempQueue;
        NativeQueue<PointXYZI>.ParallelWriter tempQueueWriter;
        NativeArray<PointXYZI> tempPointsInput;
        private bool sensorReady = false;
        private TimeSpan __dataTime;

        protected override void AfterEnable()
        {
            sensor = gameObject.AddComponent<RaycastLiDARSensor>();

            var processedScanPattern = DownSampleScanPattern(_scanPattern, 1 - downSampleScale);
            processedScanPattern = ReduceScanPatternAngle(processedScanPattern, minAzimuthAngle, maxAzimuthAngle);

            sensor._scanPattern = processedScanPattern;
            sensor._pointsNumPerScan = Mathf.Min(processedScanPattern.scans.Length, _pointsNumPerScan);
            sensor._minRange = _minRange;
            sensor._maxRange = _maxRange;
            sensor._gaussianNoiseSigma = _gaussianNoiseSigma;
            sensor._maxIntensity = _maxIntensity;
            sensor._frequency_inv = sendRate;

            sensor.enabled = true;
            sensor.onSensorUpdated += OnSensorUpdated;
            sensor.Init();
            sensor.UpdateSensor();


            data.fields = new PointField[3];
            for (int i = 0; i < 3; i++)
            {
                data.fields[i] = new PointField();
                data.fields[i].name = ((char)('x' + i)).ToString();
                data.fields[i].offset = 0;
                data.fields[i].datatype = PointField.FLOAT32;
                data.fields[i].count = 1;
            }

            CalculateFieldsOffset();
        }

        protected override void AfterDisable()
        {
            _jobHandle.Complete();
        }

        private ScanPattern ReduceScanPatternAngle(ScanPattern scanPattern, float minAzimuth, float maxAzimuth)
        {
            if (Math.Abs(maxAzimuth - 360f) < ANGLE_TOLERANCE && minAzimuth == 0f) return scanPattern;

            var newScans = (from scan in scanPattern.scans
                            let azimuth = Mathf.Atan2(-scan.x, -scan.z) * Mathf.Rad2Deg + 180f
                            where azimuth >= minAzimuth && azimuth <= maxAzimuth
                            select scan).ToList();

            var newScanPattern = Instantiate(scanPattern);
            newScanPattern.scans = newScans.ToArray();
            newScanPattern.size = newScans.Count;
            return newScanPattern;
        }

        private const double ANGLE_TOLERANCE = 0.001;

        private ScanPattern DownSampleScanPattern(ScanPattern scanPattern, float downSample)
        {
            var newScanPattern = Instantiate(scanPattern);
            List<float3> newScans = new List<float3>();

            int downSampleNum = (int)(1 / downSample);

            for (int i = 0; i < scanPattern.size; i += downSampleNum)
            {
                newScans.Add(scanPattern.scans[i]);
            }

            newScanPattern.scans = newScans.ToArray();
            newScanPattern.size = newScans.Count;

            return newScanPattern;
        }


        private void OnSensorUpdated()
        {
            __dataTime = ProBridgeServer.SimTime;
            sensorReady = true;
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
            if (!sensorReady)
                return null;

            tempQueue = new NativeQueue<PointXYZI>(Allocator.TempJob);
            tempQueueWriter = tempQueue.AsParallelWriter();

            _zeroFilterJob = new FilterZeroPointsParallelJob()
            {
                inputArray = sensor.pointCloud.points,
                outputQueue = tempQueueWriter
            };

            _zeroFilterJob.Schedule(sensor.pointsNum, 12).Complete();

            data.is_bigendian = false;
            data.width = (uint)tempQueue.Count;
            data.height = 1;
            data.point_step = CalculateFieldsSize();
            data.row_step = data.width * data.point_step;
            data.is_dense = true;
            data.data = new byte[data.row_step * data.height];
            tempData = new NativeArray<byte>((int)(data.row_step * data.height), Allocator.TempJob);
            tempPointsInput = tempQueue.ToArray(Allocator.TempJob);
            _pointsToPointCloud2MsgJob = new PointsToPointCloud2MsgJob<PointXYZI>()
            {
                points = tempPointsInput,
                data = tempData
            };

            _jobHandle = _pointsToPointCloud2MsgJob.Schedule(tempQueue.Count, 12);
            _jobHandle.Complete();
            _pointsToPointCloud2MsgJob.data.CopyTo(tempData);

            tempData.CopyTo(data.data);

            sensorReady = false;

            _jobHandle.Complete();

            tempQueue.Dispose();
            tempData.Dispose();
            tempPointsInput.Dispose();

            return base.GetMsg(__dataTime);
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