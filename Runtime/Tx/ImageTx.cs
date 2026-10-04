using System;
using Unity.Collections;
using UnityEngine;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// Publishes the camera image uncompressed (sensor_msgs/Image, rgb8). No encoding cost, but a lot of traffic:
    /// 640x480 at 10 Hz is about 9.2 MB/s (74 Mbit/s).
    /// </summary>
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/Image")]
    public class ImageTx : ProBridgeTxStamped<sensor_msgs.msg.Image>, ICameraImageSource
    {
        #region Inspector

        [Tooltip("Camera whose image is published. It is rendered on demand at sendRate, not every frame.")]
        public Camera renderCamera;
        public int textureWidth = 640;
        public int textureHeight = 480;

        [Header("Debug")]
        public float frameRate;
        #endregion

        private const int BytesPerPixel = 3;

        private CameraCapture _capture;
        private TimeSpan _captureStamp;

        // Serialized right after GetMsg on the main thread, so one buffer is reused for every frame.
        private byte[] _frame;
        private TimeSpan _frameStamp;
        private bool _hasFrame;

        private int __frameRateCounter = 0;

        public Camera ImageCamera => renderCamera;
        public int ImageWidth => textureWidth;
        public int ImageHeight => textureHeight;
        public string ImageFrameId => frame_id;
        public event Action<TimeSpan> FramePublished;

        protected override void AfterEnable()
        {
            _capture = CameraCapture.Create(renderCamera, textureWidth, textureHeight, TextureFormat.RGB24, OnFrame, this);
            if (_capture == null)
            {
                enabled = false;
                return;
            }

            _frame = new byte[textureWidth * textureHeight * BytesPerPixel];
            _hasFrame = false;

            data.width = (uint)textureWidth;
            data.height = (uint)textureHeight;
            data.encoding = "rgb8";
            data.is_bigendian = 0;
            data.step = (uint)(textureWidth * BytesPerPixel);

            // sendRate 0 means "every simulation step".
            _capture.SetPeriod(sendRate > 0f ? sendRate : Time.fixedDeltaTime);
            InvokeRepeating(nameof(CalcFPS), 0, 1);
        }

        protected override void AfterDisable()
        {
            CancelInvoke(nameof(CalcFPS));

            _capture?.Dispose();
            _capture = null;
            _hasFrame = false;
        }

        void CalcFPS()
        {
            frameRate = __frameRateCounter;
            __frameRateCounter = 0;
        }

        private void Update()
        {
            if (_capture == null || !_capture.IsDue || !Active)
                return;
            if (!useWithoutConnect && (host == null || !host.IsConnected))
                return;

            _captureStamp = ProBridgeServer.SimTime;
            _capture.Capture();
        }

        private void OnFrame(NativeArray<byte> frame)
        {
            // Texture rows go bottom-up, ROS image rows go top-down.
            int row = textureWidth * BytesPerPixel;
            for (int y = 0; y < textureHeight; y++)
                NativeArray<byte>.Copy(frame, (textureHeight - 1 - y) * row, _frame, y * row, row);

            _frameStamp = _captureStamp;
            _hasFrame = true;
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            if (!_hasFrame)
                return null;
            _hasFrame = false;

            data.data = _frame;

            __frameRateCounter++;
            FramePublished?.Invoke(_frameStamp);
            return base.GetMsg(_frameStamp);
        }
    }
}
