using System;
using UnitySensors.Sensor.LiDAR;
using sensor_msgs.msg;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnitySensors.Data.PointCloud;


namespace ProBridge.Tx.Sensor
{
    [RequireComponent(typeof(RaycastLiDARSensor))]
    public class RaycastLiDARTx : ProBridgeTxStamped<PointCloud2>
    {
        private RaycastLiDARSensor sensor;
        private IPointsToPointCloud2MsgJob<PointXYZI> _pointsToPointCloud2MsgJob;
        private JobHandle _jobHandle;
        private NativeArray<byte> tempData;


        protected override void OnStart()
        {
            sensor = GetComponent<RaycastLiDARSensor>();
            
            
            data.fields = new PointField[3];
            for (int i = 0; i < 3; i++)
            {
                data.fields[i] = new PointField();
                data.fields[i].name = ((char)('x' + i)).ToString();
                data.fields[i].offset = (uint)(4 * i);
                data.fields[i].datatype = 7; // FLOAT32
                data.fields[i].count = 1;
            }

            data.is_bigendian = false;
            data.width = (uint)sensor.pointsNum;
            data.height = 1;
            data.point_step = 12; // 3 floats (x, y, z) * 4 bytes each
            data.row_step = data.width * data.point_step;
            data.is_dense = true;
            data.data = new byte[(uint)sensor.pointsNum * 12];
            tempData = new NativeArray<byte>(sensor.pointsNum * 12, Allocator.Persistent);
            
            _pointsToPointCloud2MsgJob = new IPointsToPointCloud2MsgJob<PointXYZI>()
            {
                points = sensor.pointCloud.points,
                data = tempData
            };
            
            
            base.OnStart();
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
    }
}