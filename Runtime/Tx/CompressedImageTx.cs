using System;
using System.Threading;
using sensor_msgs.msg;
using TurboJpegWrapper;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

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
        public Camera renderCamera;
        public int textureWidth = 1024;
        public int textureHeight = 1024;
        [Range(1, 100)] public uint CompressionQuality = 90;

        [Header("Debug")]
        public float frameRate;
        #endregion

        private struct PipeBuffer
        {
            public bool useRender, useCompressor;
            public TimeSpan timeRender, timeCompressor, timeSender;
            public NativeArray<byte> bufRender;
            public byte[] bufCompressor;
            public byte[] bufSender;
            public object syncSender;
            public string formatSender;

            public Texture2D textPNG;

            public void Init(int width, int height)
            {
                textPNG = new Texture2D(width, height, TextureFormat.RGB24, false);
                bufRender = new NativeArray<byte>(width * height * 4, Allocator.Persistent, NativeArrayOptions.ClearMemory);
                bufCompressor = new byte[0];
                bufSender = null;
                useRender = true;
                useCompressor = true;
                syncSender = new object();
                formatSender = "";
            }

            public void Dispose()
            {
                bufSender = null;
                bufCompressor = null;
                try
                {
                    bufRender.Dispose();
                }
                catch { }
                try
                {
                    Destroy(textPNG);
                }
                catch { }
            }
        }


        private RenderTexture renderTexture;

        private int __frameRateCounter = 0;
        private PipeBuffer __pb = new PipeBuffer();

        private EventWaitHandle __readyRawTextureData = new EventWaitHandle(false, EventResetMode.AutoReset);
        private Thread jpegCompressionThread;

        private bool __active = false;

        private bool inRequest;
        private bool disposing;

        protected override void AfterEnable()
        {
            if (renderCamera == null)
            {
                Debug.LogWarning("Render camera is not set.");
                enabled = false;
            }

            if (renderCamera.targetTexture == null)
            {
                renderTexture = new RenderTexture(textureWidth, textureHeight, 24, RenderTextureFormat.ARGB32);
                renderTexture.Create();
                renderCamera.targetTexture = renderTexture;
            }
            else
            {
                renderTexture = renderCamera.targetTexture;
                if (renderTexture.format != RenderTextureFormat.ARGB32)
                {
                    throw new Exception("The RenderTexture format must be ARGB32.");
                }

                if (renderTexture.width != textureWidth || renderTexture.height != textureHeight)
                {
                    throw new Exception($"RenderTexture dimensions are incorrect. Expected {textureWidth}x{textureHeight}, but got {renderTexture.width}x{renderTexture.height}.");
                }
            }

            __pb.Init(textureWidth, textureHeight);

            __active = true;
            disposing = false;
            __readyRawTextureData.Reset();
            jpegCompressionThread = new Thread(JpegCompressor);
            jpegCompressionThread.Start();

            InvokeRepeating(nameof(RenderLoop), 0, sendRate);
            InvokeRepeating(nameof(CalcFPS), 0, 1);
        }

        protected override void AfterDisable()
        {
            if(inRequest)
            {
                disposing = true;
                return;
            }

            __active = false;
        
            // This might be called after the component got destroyed; prevents getting a null ref exception.
            if (this != null)
            {
                CancelInvoke(nameof(RenderLoop));
                CancelInvoke(nameof(CalcFPS));
            }

            if (jpegCompressionThread != null)
            {
                try
                {
                    __readyRawTextureData.Set();
                    if (!jpegCompressionThread.Join(500))
                        jpegCompressionThread.Abort();
                }
                catch { }
                finally
                {
                    jpegCompressionThread = null;
                }
            }

            __pb.Dispose();

            if (renderTexture != null)
            {
                renderTexture.Release();
            }
        }

        void CalcFPS()
        {
            frameRate = __frameRateCounter;
            __frameRateCounter = 0;
        }

        void RenderLoop()
        {
            if (__pb.useRender && __active)
            {
                __pb.useRender = false;
                __pb.timeRender = ProBridgeServer.SimTime;

                if (inRequest) return;
                inRequest = true;

                switch (format)
                {
                    case Format.jpeg:
                        AsyncGPUReadback.RequestIntoNativeArray(ref __pb.bufRender, renderTexture, 0, OnCompleteReadback);
                        break;

                    case Format.png:
                        AsyncGPUReadback.Request(renderTexture, 0, TextureFormat.RGB24, OnCompleteReadbackPNG);
                        break;
                }
            }
        }

        private void OnCompleteReadback(AsyncGPUReadbackRequest request)
        {
            inRequest = false;
            if (disposing)
            {
                AfterDisable();
                return;
            }

            __pb.useRender = true;
            if (request.hasError || !__active || !__pb.useCompressor)
                return;

            __pb.bufCompressor = __pb.bufRender.ToArray();
            __pb.timeCompressor = __pb.timeRender;
            __readyRawTextureData.Set();
        }

        private void OnCompleteReadbackPNG(AsyncGPUReadbackRequest request)
        {
            __pb.useRender = true;
            if (request.hasError || !__active)
                return;

            __pb.textPNG.LoadRawTextureData(request.GetData<Color32>());
            __pb.textPNG.Apply();

            __pb.timeSender = __pb.timeRender;
            __pb.formatSender = "png";
            __pb.bufSender = __pb.textPNG.EncodeToPNG();
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            lock (__pb.syncSender)
            {
                if (__pb.bufSender == null)
                    return null;

                ts = __pb.timeSender;
                data.format = __pb.formatSender;
                data.data = __pb.bufSender;
                __pb.bufSender = null;
            }

            __frameRateCounter++;
            return base.GetMsg(ts);
        }

        private void JpegCompressor()
        {
            var compressor = new TJCompressor();
            try
            {
                while (__active)
                {
                    if (!__readyRawTextureData.WaitOne(500) || format != Format.jpeg || !__active)
                        continue;

                    __pb.useCompressor = false;

                    var jpg = compressor.Compress(__pb.bufCompressor, 0,
                        textureWidth, textureHeight,
                        TJPixelFormat.RGBA, TJSubsamplingOption.Chrominance444,
                        (int)CompressionQuality,
                        TJFlags.FastDct | TJFlags.BottomUp);

                    lock (__pb.syncSender)
                    {
                        if (__pb.bufSender == null)
                        {
                            __pb.bufSender = jpg;
                            __pb.timeSender = __pb.timeCompressor;
                            __pb.formatSender = "jpeg";
                        }
                    }

                    __pb.useCompressor = true;
                }
            }
            finally
            {
                compressor.Dispose();
            }
        }
    }
}