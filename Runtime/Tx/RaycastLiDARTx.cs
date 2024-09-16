using System;
using UnitySensors.Sensor.LiDAR;
using sensor_msgs.msg;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnitySensors.Data.PointCloud;


namespace ProBridge.Tx.Sensor
{
    // [RequireComponent(typeof(RaycastLiDARSensor))]
    public class RaycastLiDARTx : ProBridgeTxStamped<PointCloud2>
    {
        
        [Header("Lidar Params")]
        public ScanPattern _scanPattern;
        public float _minRange = 0.5f;
        public float _maxRange = 100.0f;
        public float _gaussianNoiseSigma = 0.0f;
        public float _maxIntensity = 255.0f;
        
        
        
        private RaycastLiDARSensor sensor;
        private IPointsToPointCloud2MsgJob<PointXYZI> _pointsToPointCloud2MsgJob;
        private JobHandle _jobHandle;
        private NativeArray<byte> tempData;


        protected override void OnStart()
        {

            sensor = gameObject.AddComponent<RaycastLiDARSensor>();
            
            sensor._scanPattern = _scanPattern;
            sensor._pointsNumPerScan = _scanPattern.scans.Length;
            sensor._minRange = _minRange;
            sensor._maxRange = _maxRange;
            sensor._gaussianNoiseSigma = _gaussianNoiseSigma;
            sensor._maxIntensity = _maxIntensity;
            sensor._frequency = 1.0f / sendRate;

            sensor.enabled = true;
            sensor.Init();
            
            
            
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
            _jobHandle = _pointsToPointCloud2MsgJob.Schedule(sensor.pointsNum, 12);
            _jobHandle.Complete();
            _pointsToPointCloud2MsgJob.data.CopyTo(tempData);

            tempData.CopyTo(data.data);
            
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