using System;
using ProBridge.Tx;
using UnityEngine;
using UnityEngine.UI;
using UnitySensors.Data.PointCloud;
using UnitySensors.Sensor.Camera;

[AddComponentMenu("ProBridge/Tx/sensor_msgs/Depth16bit Camera")]
public class Depth16bitCameraTx : ProBridgeTxStamped<sensor_msgs.msg.Image>
{
    public Camera renderCamera;
    public float _minRange = 0.05f;
    public float _maxRange = 100.0f;
    public float _gaussianNoiseSigma = 0.0f;
    public int fov = 30;
    public int textureWidth = 1024;
    public int textureHeight = 1024;
    public RawImage _rawImage;
    [Range(1, 100)] public int CompressionQuality = 90;
    public Shader depthShader;
    public bool getPointCloud = false;

    public PointCloud<PointXYZ> pointCloud
    {
        get => _cameraSensor.pointCloud;
    }

    public Depth16bitCameraSensor _cameraSensor { get; private set; }
    private bool sensorReady = false;

    private int H;
    private int W;

    protected override void AfterEnable()
    {

        _cameraSensor = renderCamera.gameObject.AddComponent<Depth16bitCameraSensor>();
        _cameraSensor.mat = new Material(depthShader);
        _cameraSensor._camera = renderCamera;
        _cameraSensor.onSensorUpdated += OnSensorUpdated;
        _cameraSensor._minRange = _minRange;
        _cameraSensor._maxRange = _maxRange;
        _cameraSensor._gaussianNoiseSigma = _gaussianNoiseSigma;
        _cameraSensor._fov = fov;
        _cameraSensor._frequency_inv = sendRate;
        _cameraSensor._resolution.x = textureWidth;
        _cameraSensor._resolution.y = textureHeight;
        _cameraSensor.getPointCloud = getPointCloud;
        _cameraSensor.Init();

        W = _cameraSensor.Width;
        H = _cameraSensor.Height;

        data.height = (uint)H;
        data.width = (uint)W;
        data.encoding = "16UC1";
        data.is_bigendian = 0;
        data.step = (uint)(W * 2);
    }

    protected override void AfterDisable()
    {
        if (_cameraSensor != null)
            _cameraSensor.DisposeSensor();
    }

    private void OnSensorUpdated()
    {
        sensorReady = true;
    }

    protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
    {
        if (!sensorReady) return null;
        sensorReady = false;

        if (W <= 0 || H <= 0) return null;

        // Packed R16 UNorm from the sensor (length = W*H*2)
        ReadOnlySpan<byte> src = _cameraSensor.LatestR16Bytes();
        if (src.Length != W * H * 2) return null;

        float farM = _cameraSensor.m_camera.farClipPlane;

        // Reuse these as fields if you want to avoid GC each frame
        ushort[] dstU16 = new ushort[W * H];

        // Convert: R16 UNorm -> distanceNorm [0..1] -> depth meters -> mm (uint16)
        // Also flip vertically (Unity bottom-left -> ROS top-left)
        int rowBytes = W * 2;
        for (int y = 0; y < H; y++)
        {
            int srcY = H - 1 - y;                 // flip
            int srcRow = srcY * rowBytes;
            int dstRow = y * W;

            for (int x = 0; x < W; x++)
            {
                // read uint16 little-endian
                int b0 = src[srcRow + (x << 1) + 0];
                int b1 = src[srcRow + (x << 1) + 1];
                int u16 = (b1 << 8) | b0;

                float distanceNorm = u16 * (1.0f / 65535.0f);      // 0..1
                float depthM = (1.0f - distanceNorm) * farM;       // meters (undo shader)
                int mm = (int)Mathf.Round(depthM * 1000.0f);       // to mm

                if (mm < 0) mm = 0;
                else if (mm > 65535) mm = 65535;

                dstU16[dstRow + x] = (ushort)mm;
            }
        }

        // Pack to bytes
        byte[] bytes = new byte[W * H * 2];
        Buffer.BlockCopy(dstU16, 0, bytes, 0, bytes.Length);

        data.data = bytes;

        return base.GetMsg(ts);
    }

}