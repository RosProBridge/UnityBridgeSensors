using System;
using System.Collections.Generic;
using Codice.CM.WorkspaceServer.DataStore.Configuration;
using UnitySensors.Sensor.LiDAR;
using sensor_msgs.msg;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnitySensors.Data.PointCloud;


namespace ProBridge.Tx.Sensor
{
    // [RequireComponent(typeof(RaycastLiDARSensor))]
    public class RaycastLiDARTx : ProBridgeTxStamped<PointCloud2>
    {
        [Header("Lidar Params")] public ScanPattern _scanPattern;
        public int _pointsNumPerScan = 10000;
        public float _minRange = 0.5f;
        public float _maxRange = 100.0f;
        public float _gaussianNoiseSigma = 0.0f;
        public float _maxIntensity = 255.0f;
        [Range(0f, 1f)] public float downSampleScale = 0.5f;


        private RaycastLiDARSensor sensor;
        private IPointsToPointCloud2MsgJob<PointXYZI> _pointsToPointCloud2MsgJob;
        private JobHandle _jobHandle;
        private NativeArray<byte> tempData;
        private bool sensorReady = false;


        protected override void OnStart()
        {
            sensor = gameObject.AddComponent<RaycastLiDARSensor>();

            ScanPattern downSampledPattern = DownSampleScanPattern(_scanPattern, 1 - downSampleScale);

            sensor._scanPattern = downSampledPattern;
            sensor._pointsNumPerScan = Mathf.Min(downSampledPattern.scans.Length, _pointsNumPerScan);
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

            data.is_bigendian = false;
            data.width = (uint)sensor.pointsNum;
            data.height = 1;
            data.point_step = CalculateFieldsSize();
            data.row_step = data.width * data.point_step;
            data.is_dense = true;
            data.data = new byte[data.row_step * data.height];
            tempData = new NativeArray<byte>((int)(data.row_step * data.height), Allocator.Persistent);

            _pointsToPointCloud2MsgJob = new IPointsToPointCloud2MsgJob<PointXYZI>()
            {
                points = sensor.pointCloud.points,
                data = tempData
            };


            base.OnStart();
        }

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
            {
                throw new Exception("Sensor is not ready, PC might be running too slow for the requested frequency.");
            }

            _jobHandle = _pointsToPointCloud2MsgJob.Schedule(sensor.pointsNum, 12);
            _jobHandle.Complete();
            _pointsToPointCloud2MsgJob.data.CopyTo(tempData);

            tempData.CopyTo(data.data);

            sensorReady = false;

            return base.GetMsg(ts);
        }


        protected override void OnStop()
        {
            _jobHandle.Complete();
            tempData.Dispose();

            base.OnStop();
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