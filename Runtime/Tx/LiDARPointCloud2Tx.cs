using System;
using ProBridge.ROS.Msgs.Sensors;
using ProBridge.ROS.Msgs.Std;
using UnitySensors.Sensor.LiDAR;
using UnityEngine;
using Unity.Collections;


namespace ProBridge.Tx.Sensor
{
    public class LiDARPointCloud2Tx : ProBridgeTxStamped<ROS.Msgs.Sensors.PointCloud2>
    {

        public RaycastLiDARSensor sensor;

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.fields = new PointField[3];
            for (int i = 0; i < 3; i++)
            {
                data.fields[i] = new PointField();
                data.fields[i].name = ((char)('x' + i)).ToString();
                data.fields[i].offset = (uint)(4 * i);
                data.fields[i].datatype = 7;  // FLOAT32
                data.fields[i].count = 1;
            }
            data.is_bigendian = false;
            data.width = (uint)sensor.pointsNum;
            data.height = 1;
            data.point_step = 12;  // 3 floats (x, y, z) * 4 bytes each
            data.row_step = data.width * data.point_step;
            data.is_dense = true;
            data.data = new byte[(uint)sensor.pointsNum * 12];

            NativeArray<byte> tempData = new NativeArray<byte>(sensor.pointsNum * 12, Allocator.Persistent);

            for (int i = 0; i < sensor.pointCloud.points.Length; i++)
            {
                NativeArray<float> tmp = new NativeArray<float>(3, Allocator.Temp);
                tmp[0] = sensor.pointCloud.points[i].position.z;
                tmp[1] = -sensor.pointCloud.points[i].position.x;
                tmp[2] = sensor.pointCloud.points[i].position.y;

                var slice = new NativeSlice<float>(tmp).SliceConvert<byte>();
                slice.CopyTo(tempData.GetSubArray(i * 12, 12));
            }

            tempData.CopyTo(data.data);
            return base.GetMsg(ts);
        }
    }
}
