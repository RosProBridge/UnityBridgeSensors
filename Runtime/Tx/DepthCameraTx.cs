using System;
using ProBridge.Tx;
using UnityEngine;
using UnityEngine.UI;
using UnitySensors.Data.PointCloud;
using UnitySensors.Sensor.Camera;

[AddComponentMenu("ProBridge/Tx/sensor_msgs/Depth Camera")]
public class DepthCameraTx : ProBridgeTxStamped<sensor_msgs.msg.Image>
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

    private DepthCameraSensor _cameraSensor;
    private bool sensorReady = false;

    // Reuse the compressor to avoid per-frame allocations.
    //private TJCompressor _compressor;

    protected override void AfterEnable()
    {
        //_compressor = new TJCompressor();

        _cameraSensor = renderCamera.gameObject.AddComponent<DepthCameraSensor>();
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
    }

    protected override void AfterDisable()
    {
        if (_cameraSensor != null)
            _cameraSensor.DisposeSensor();

        //if (_compressor != null)
        //{
        //    _compressor.Dispose();
        //    _compressor = null;
        //}
    }

    private void OnSensorUpdated()
    {
        sensorReady = true;
    }

    protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
    {
        if (!sensorReady) return null;
        sensorReady = false;

        var tex = _cameraSensor.texture0;   // TextureFormat.RGBAFloat
        if (tex == null) return null;

        int W = tex.width, H = tex.height, N = W * H;

        // --- Read depth in METERS as float per pixel ---
        float[] src; // one float per pixel, row-major (Unity origin = bottom-left)

        if (tex.format == TextureFormat.RFloat)
        {
            // Single-channel 32F texture
            src = tex.GetRawTextureData<float>().ToArray();
        }
        else
        {
            // RGBAFloat / RGBAHalf path: take R channel as meters
            var px = tex.GetPixelData<Color>(0);
            if (!px.IsCreated || px.Length != N) return null;
            src = new float[N];
            for (int i = 0; i < N; i++) src[i] = px[i].r;
        }

        // --- FLIP VERTICALLY: Unity (bottom-left) -> ROS/OpenCV (top-left) ---
        var flipped = new float[N];
        for (int y = 0; y < H; y++)
        {
            int srcY = H - 1 - y;                 // take rows from bottom to top
            Array.Copy(src, srcY * W, flipped, y * W, W);
        }

        // --- Pack to bytes and publish as 32FC1 ---
        var bytes = new byte[N * sizeof(float)];
        Buffer.BlockCopy(flipped, 0, bytes, 0, bytes.Length);

        data.height = (uint)H;
        data.width = (uint)W;
        data.encoding = "32FC1";    // meters
        data.is_bigendian = 0;
        data.step = (uint)(W * 4);
        data.data = bytes;
        data.header.frame_id = "depth_camera";

        return base.GetMsg(ts);
    }

}