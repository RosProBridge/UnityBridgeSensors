using System;
using ProBridge.Tx;
using ProBridge.Tx.Sensor;
using sensor_msgs.msg;
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
    public bool invertDepthColors = false;
    public bool getPointCloud = false;

    public PointCloud<PointXYZ> pointCloud
    {
        get => _cameraSensor.pointCloud;
    }

    public DepthCameraSensor _cameraSensor { get; private set; }
    private bool sensorReady = false;

    // Reuse the encoder to avoid per-frame allocations.
    private JpegEncoder _encoder;

    protected override void AfterEnable()
    {
        _encoder = new JpegEncoder();

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

        if (_encoder != null)
        {
            _encoder.Dispose();
            _encoder = null;
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

        sensorReady = false;

        var tex = _cameraSensor.texture0;
        if (tex == null) return null;

        if (_rawImage != null) _rawImage.texture = tex;

        NativeArray<byte> raw = tex.GetRawTextureData<byte>();
        byte[] rgbaBytes = raw.ToArray();

        if (invertDepthColors)
        {
            for (int i = 0; i < rgbaBytes.Length; i += 4)
            {
                byte inv = (byte)(255 - rgbaBytes[i]);
                rgbaBytes[i] = inv;
                rgbaBytes[i + 1] = inv;
                rgbaBytes[i + 2] = inv;
            }
        }

        var jpg = _encoder.Encode(rgbaBytes, tex.width, tex.height, CompressionQuality, grayscale: true);

        data.format = "jpeg";
        data.data = jpg;
        return base.GetMsg(ts);
    }

}