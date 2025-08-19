using System;
using ProBridge.Tx;
using sensor_msgs.msg;
using TurboJpegWrapper;
using Unity.Collections;
using UnityEngine;
using UnityEngine.UI;
using UnitySensors.Data.PointCloud;
using UnitySensors.Sensor.Camera;

[AddComponentMenu("ProBridge/Tx/sensor_msgs/Depth Camera")]
public class DepthCameraTx : ProBridgeTxStamped<CompressedImage>
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
    private TJCompressor _compressor;

    protected override void AfterEnable()
    {
        _compressor = new TJCompressor();

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

        if (_compressor != null)
        {
            _compressor.Dispose();
            _compressor = null;
        }
    }

    private void OnSensorUpdated()
    {
        sensorReady = true;
    }

    protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
    {
        if (!sensorReady)
            throw new Exception("Sensor is not ready");

        var tex = _cameraSensor.texture0;
        if (tex == null)
            throw new Exception("Depth sensor texture is not a Texture2D.");

        if (_rawImage != null)
            _rawImage.texture = tex;

        sensorReady = false;

        NativeArray<byte> raw = tex.GetRawTextureData<byte>();
        byte[] rgbaBytes = raw.ToArray();

        var jpg = _compressor.Compress(
            rgbaBytes, 0,
            tex.width, tex.height,
            TJPixelFormats.TJPF_RGBA,
            TJSubsamplingOptions.TJSAMP_GRAY,
            CompressionQuality,
            TJFlags.FASTDCT | TJFlags.BOTTOMUP
        );

        data.format = "jpeg";
        data.data = jpg;

        return base.GetMsg(ts);
    }
}