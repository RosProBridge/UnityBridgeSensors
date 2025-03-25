using System;
using System.Threading;
using sensor_msgs.msg;
using TurboJpegWrapper;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using Time = std_msgs.Time;

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

        #endregion


        [HideInInspector] public float frameRate;


        private RenderTexture renderTexture;
        private Texture2D texture2D;
        private TJCompressor compressor;
        private bool newFrameAvailable;
        private NativeArray<Color32> imageData;

        private (byte[], Time) rawTextureData;
        private Thread jpegCompressionThread;

        private DateTime lastRenderTime;

        protected override void OnStart()
        {
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

            texture2D = new Texture2D(textureWidth, textureHeight, TextureFormat.RGB24, false);
            compressor = new TJCompressor();
            lastRenderTime = DateTime.Now;

            InvokeRepeating(nameof(RenderLoop), 0, sendRate);

            if (format != Format.jpeg) return;
            jpegCompressionThread = new Thread(JpegCompressor);
            jpegCompressionThread.Start();
        }


        void RenderLoop()
        {
            renderCamera.Render();
            Time frameTimestamp = ProBridgeServer.SimTime;
            AsyncGPUReadback.Request(renderTexture, 0, TextureFormat.RGB24, (request) => OnCompleteReadback(request, frameTimestamp));
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.format = format.ToString();

            if (format == Format.png && newFrameAvailable)
            {
                // TODO: optimize png encoding
                data.data = texture2D.EncodeToPNG();
                data.header.stamp = ProBridgeServer.SimTime;
                newFrameAvailable = false;

                UpdateFrameRate();
            }

            data.data ??= new byte[] { 0, 0, 0};            

            return base.GetMsg(data.header.stamp);
        }

        private void OnCompleteReadback(AsyncGPUReadbackRequest request, Time frameTimestamp)
        {
            if (request.hasError)
            {
                Debug.LogError("GPU read-back error detected.");
                return;
            }

            if (texture2D == null) return;

            imageData = request.GetData<Color32>();
            texture2D.LoadRawTextureData(imageData);
            texture2D.Apply();

            if (format == Format.jpeg)
            {
                rawTextureData.Item1 = texture2D.GetRawTextureData();
                rawTextureData.Item2 = frameTimestamp;
            }

            newFrameAvailable = true;
        }


        private void JpegCompressor()
        {
            while (true)
            {
                if (newFrameAvailable)
                {
                    data.data = compressor.Compress(rawTextureData.Item1, 0,
                        textureWidth,
                        textureHeight, TJPixelFormat.RGB, TJSubsamplingOption.Chrominance420,
                        (int)CompressionQuality,
                        TJFlags.FastDct | TJFlags.BottomUp);
                    data.header.stamp = rawTextureData.Item2;

                    newFrameAvailable = false;

                    UpdateFrameRate();
                }

                Thread.Sleep((int)sendRate * 1000);
            }
        }

        private void UpdateFrameRate()
        {
            frameRate = (float)(1 / (DateTime.Now - lastRenderTime).TotalSeconds);
            lastRenderTime = DateTime.Now;
        }


        private void OnDestroy()
        {
            if (renderTexture != null)
            {
                renderTexture.Release();
            }

            if (texture2D != null)
            {
                Destroy(texture2D);
            }

            if (format == Format.jpeg)
            {
                jpegCompressionThread.Abort();
            }
        }
    }
}