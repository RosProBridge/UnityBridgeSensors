using System;
using System.Threading;
using sensor_msgs.msg;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/CompressedImage")]
    public class CompressedImageTx : ProBridgeTxStamped<CompressedImage>
    {
        public enum Format
        {
            jpeg,
            png
        }

        #region Inspector

        public Format format = Format.jpeg;
        [Tooltip("Camera whose image is published. It is rendered on demand at sendRate, not every frame.")]
        public Camera renderCamera;
        public int textureWidth = 1024;
        public int textureHeight = 1024;
        [Range(1, 100)] public uint CompressionQuality = 90;

        [Header("Debug")]
        public float frameRate;
        #endregion

        /// <summary>
        /// Encoder thread of one enable/disable cycle: takes a raw frame, publishes the encoded one.
        /// </summary>
        private sealed class Encoder
        {
            private readonly Format _format;
            private readonly int _width, _height;
            public volatile int quality;

            private readonly byte[] _raw;
            private TimeSpan _rawStamp;
            private volatile bool _busy;
            private volatile bool _running = true;
            private readonly AutoResetEvent _rawReady = new AutoResetEvent(false);
            private readonly Thread _thread;

            private readonly object _sendLock = new object();
            private byte[] _encoded;
            private TimeSpan _encodedStamp;

            public Encoder(Format format, int width, int height, int quality)
            {
                _format = format;
                _width = width;
                _height = height;
                this.quality = quality;
                _raw = new byte[width * height * (format == Format.png ? 3 : 4)];

                _thread = new Thread(Loop) { IsBackground = true, Name = "CompressedImageTx encoder" };
                _thread.Start();
            }

            public bool Busy => _busy;

            public void Submit(NativeArray<byte> frame, TimeSpan stamp)
            {
                if (_busy || !_running)
                    return;

                frame.CopyTo(_raw);
                _rawStamp = stamp;
                _busy = true;
                _rawReady.Set();
            }

            public bool TryTake(out byte[] encoded, out TimeSpan stamp)
            {
                lock (_sendLock)
                {
                    encoded = _encoded;
                    stamp = _encodedStamp;
                    _encoded = null;
                }
                return encoded != null;
            }

            public void Stop()
            {
                _running = false;
                _rawReady.Set();
                if (!_thread.Join(1000))
                    Debug.LogWarning("CompressedImageTx: encoder thread did not stop in time.");
            }

            private void Loop()
            {
                var jpeg = _format == Format.jpeg ? new JpegEncoder() : null;
                try
                {
                    while (_running)
                    {
                        if (!_rawReady.WaitOne(500) || !_running || !_busy)
                            continue;

                        try
                        {
                            byte[] encoded = _format == Format.jpeg
                                ? jpeg.Encode(_raw, _width, _height, quality)
                                : ImageConversion.EncodeArrayToPNG(_raw, GraphicsFormat.R8G8B8_UNorm, (uint)_width, (uint)_height);

                            lock (_sendLock)
                            {
                                _encoded = encoded;
                                _encodedStamp = _rawStamp;
                            }
                        }
                        catch (Exception e)
                        {
                            Debug.LogException(e);
                        }
                        finally
                        {
                            _busy = false;
                        }
                    }
                }
                finally
                {
                    jpeg?.Dispose();
                }
            }
        }

        private CameraCapture _capture;
        private Encoder _encoder;
        private TimeSpan _captureStamp;

        private int __frameRateCounter = 0;

        protected override void AfterEnable()
        {
            _capture = CameraCapture.Create(renderCamera, textureWidth, textureHeight,
                format == Format.png ? TextureFormat.RGB24 : TextureFormat.RGBA32, OnFrame, this);
            if (_capture == null)
            {
                enabled = false;
                return;
            }

            _encoder = new Encoder(format, textureWidth, textureHeight, (int)CompressionQuality);

            // sendRate 0 means "every simulation step".
            _capture.SetPeriod(sendRate > 0f ? sendRate : Time.fixedDeltaTime);
            InvokeRepeating(nameof(CalcFPS), 0, 1);
        }

        protected override void AfterDisable()
        {
            CancelInvoke(nameof(CalcFPS));

            _encoder?.Stop();
            _encoder = null;
            _capture?.Dispose();
            _capture = null;
        }

        void CalcFPS()
        {
            frameRate = __frameRateCounter;
            __frameRateCounter = 0;
        }

        private void Update()
        {
            // A new frame is rendered when due and the previous one has been read back and encoded.
            if (_capture == null || !_capture.IsDue || _encoder.Busy || !Active)
                return;
            if (!useWithoutConnect && (host == null || !host.IsConnected))
                return;

            _encoder.quality = (int)CompressionQuality;
            _captureStamp = ProBridgeServer.SimTime;
            _capture.Capture();
        }

        private void OnFrame(NativeArray<byte> frame)
        {
            _encoder?.Submit(frame, _captureStamp);
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            if (_encoder == null || !_encoder.TryTake(out var encoded, out ts))
                return null;

            data.format = format == Format.png ? "png" : "jpeg";
            data.data = encoded;

            __frameRateCounter++;
            return base.GetMsg(ts);
        }
    }
}
