using System;
using ProBridge.Tx;
using sensor_msgs.msg;
using UnityEngine;
using UnitySensors.Sensor.Camera;
using UnitySensors.Data.PointCloud;

[AddComponentMenu("ProBridge/Tx/sensor_msgs/PointCloud2 (Depth)")]
public class DepthPointCloudTx : ProBridgeTxStamped<PointCloud2>
{
    public DepthCameraTx depthTx;
    public bool skipNaN = false;          // true => drop NaN points (changes width/height semantics)

    private bool sensorReady;
    private DepthCameraSensor sensor;
    protected override void AfterEnable()
    {
    }

    protected override void AfterDisable()
    {
        if (sensor != null)
            sensor.onSensorUpdated -= OnSensorUpdated;
    }

    private void OnSensorUpdated() => sensorReady = true;

    protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
    {
        if (sensor == null && depthTx)
        {
            sensor = depthTx._cameraSensor;
            sensor.onSensorUpdated += OnSensorUpdated;
            return null;
        }
            if (!sensor.Initialized)
        {
            Debug.Log("sensor not initialized!");
            return null;
        }

       if (!sensorReady) 
            return null;
       
        sensorReady = false;

        if (sensor == null || !sensor.getPointCloud)
            return null; 

        PointCloud<PointXYZ> pc = sensor.pointCloud;
        if (pc.points.Length == 0)
        {
            Debug.Log("empty pcd!");
            return null; }

        int W = sensor.Width;
        int H = sensor.Height;
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
                var p = pc.points[i].position;
                tmp[0] = p.x; tmp[1] = p.y; tmp[2] = p.z;
                Buffer.BlockCopy(tmp, 0, bytes, i * pointStep, pointStep);
            }

            data.height = (uint)H;           
            data.width = (uint)W;
            data.is_dense = false;            // NaNs allowed
        }
        else
        {
            // compact: drop NaNs
            var list = new System.Collections.Generic.List<byte>(N * pointStep);
            var tmp = new float[3];

            for (int i = 0; i < N; i++)
            {
                var p = pc.points[i].position;
                if (float.IsNaN(p.x) || float.IsNaN(p.y) || float.IsNaN(p.z)) continue;
                tmp[0] = p.x; tmp[1] = p.y; tmp[2] = p.z;

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
