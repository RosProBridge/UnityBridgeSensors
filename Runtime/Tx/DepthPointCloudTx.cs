using System;
using ProBridge.Tx;
using sensor_msgs.msg;
using UnityEngine;
using UnitySensors.Sensor.Camera;
using UnitySensors.Data.PointCloud;
using ProBridge.Utils;

[AddComponentMenu("ProBridge/Tx/sensor_msgs/PointCloud2 (Depth)")]
public class DepthPointCloudTx : ProBridgeTxStamped<PointCloud2>
{
    
    public bool skipNaN = false;          // true => drop NaN points (changes width/height semantics)
    public bool isSource16bit;

    private bool sensorReady;

    private Depth16bitCameraTx depth16Tx;
    private DepthCameraTx depthTx;
    private Depth16bitCameraSensor sensor_16;
    private DepthCameraSensor sensor;
    protected override void AfterEnable()
    {
        if (isSource16bit)
            depth16Tx = GetComponent<Depth16bitCameraTx>();
        else
            depthTx = GetComponent<DepthCameraTx>();
    }

    protected override void AfterDisable()
    {
        if (sensor != null)
            sensor.onSensorUpdated -= OnSensorUpdated;
    }

    private void OnSensorUpdated() => sensorReady = true;

    protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
    {
        if (isSource16bit)
        {
            if (sensor_16 == null && depth16Tx)
            {
                sensor_16 = depth16Tx._cameraSensor;
                sensor_16.onSensorUpdated += OnSensorUpdated;
                return null; 
            }
        }
        else
        {
            if (sensor == null && depthTx)
            {
                sensor = depthTx._cameraSensor;
                sensor.onSensorUpdated += OnSensorUpdated;
                return null; 
            }
        }

        if (isSource16bit ? sensor_16 == null : sensor == null)
            return null;


        if (!sensorReady) return null;
        sensorReady = false;

        PointCloud<PointXYZ> pc;
        int W, H;

        if (isSource16bit)
        {
            if (!sensor_16.getPointCloud) return null;
            pc = sensor_16.pointCloud;
            W = sensor_16.Width;
            H = sensor_16.Height;
        }
        else
        {
            if (!sensor.getPointCloud) return null; 
            pc = sensor.pointCloud;
            W = sensor._resolution.x; 
            H = sensor._resolution.y; 
        }

        if (pc.points.Length == 0) 
        {
            Debug.Log("empty pcd!");
            return null;
        }

        int N = pc.points.Length;
        // Build PointCloud2 fields: x,y,z float32
        var fields = new PointField[3];
        fields[0] = new PointField { name = "x", offset = 0, datatype = PointField.FLOAT32, count = 1 };
        fields[1] = new PointField { name = "y", offset = 4, datatype = PointField.FLOAT32, count = 1 };
        fields[2] = new PointField { name = "z", offset = 8, datatype = PointField.FLOAT32, count = 1 };

        int pointStep = 12;                    // 3 * float32
        byte[] bytes;

        if (!skipNaN)
        {
            bytes = new byte[N * pointStep];
            var tmp = new float[3];

            for (int i = 0; i < N; i++)
            {
                var p= (Vector3)pc.points[i].position;
                var pROS = p.ToRos();
                tmp[0] = (float) pROS.x; tmp[1] = (float)pROS.y; tmp[2] = (float)pROS.z;
                Buffer.BlockCopy(tmp, 0, bytes, i * pointStep, pointStep);
            }

            data.height = (uint)H;
            data.width = (uint)W;
            data.is_dense = false;
        }
        else
        {
            var list = new System.Collections.Generic.List<byte>(N * pointStep);
            var tmp = new float[3];

            for (int i = 0; i < N; i++)
            {
                var p = (Vector3)pc.points[i].position;
                var pROS = p.ToRos();

                if (double.IsNaN(pROS.x) || double.IsNaN(pROS.y) || double.IsNaN(pROS.z)) continue;

                tmp[0] = (float)pROS.x; tmp[1] = (float)pROS.y; tmp[2] = (float)pROS.z;
                var row = new byte[pointStep];
                Buffer.BlockCopy(tmp, 0, row, 0, pointStep);
                list.AddRange(row);
            }
            bytes = list.ToArray();

            data.height = 1;
            data.width = (uint)(bytes.Length / pointStep);
            data.is_dense = true;
        }


        data.fields = fields;
        data.is_bigendian = false;
        data.point_step = (uint)pointStep;
        data.row_step = (uint)(pointStep * (int)data.width);
        data.data = bytes;

        return base.GetMsg(ts);               
    }
}
