using System;
using System.Threading;
using ProBridge.Tx;
using sensor_msgs.msg;
using TurboJpegWrapper;
using UnityEngine;
using UnityEngine.UI;
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


    private DepthCameraSensor _cameraSensor;

    private bool sensorReady = false;


    protected override void AfterEnable()
    {
        _cameraSensor = renderCamera.gameObject.AddComponent<DepthCameraSensor>();
        _cameraSensor._camera = renderCamera;
        _cameraSensor.onSensorUpdated += OnSensorUpdated;
        _cameraSensor._minRange = _minRange;
        _cameraSensor._maxRange = _maxRange;
        _cameraSensor._gaussianNoiseSigma = _gaussianNoiseSigma;
        _cameraSensor._fov = fov;
        _cameraSensor._frequency_inv = sendRate;
        _cameraSensor._resolution.x = textureWidth;
        _cameraSensor._resolution.y = textureHeight;
        _cameraSensor.Init();
    }

    private void OnSensorUpdated()
    {
        sensorReady = true;
    }

    protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
    {
        if (!sensorReady)
        {
            throw new Exception("Sensor is not ready");
        }

        if (_rawImage != null)
            _rawImage.texture = _cameraSensor.texture0;

        sensorReady = false;

        data.format = "jpeg";
        data.data = _cameraSensor.texture0.EncodeToJPG(CompressionQuality);

        return base.GetMsg(ts);
    }
}