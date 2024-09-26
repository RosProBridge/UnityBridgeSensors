using System;
using System.Collections;
using sensor_msgs.msg;
using TurboJpegWrapper;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Sensors/Tx/CompressedImage")]
    public class CompresedImageTx : ProBridgeTxStamped<CompressedImage>
    {
        public enum Format
        {
            jpeg,
            png
        }

        public Format format = Format.jpeg;

        public Camera renderCamera;
        public int textureWidth = 1024;
        public int textureHeight = 1024;
        private RenderTexture renderTexture;
        private Texture2D texture2D;
        private bool gotFirstFrame = false;
        private TJCompressor compressor;
        private bool newFrame = false;
        private NativeArray<Color32> imageData;

        protected override void OnStart()
        {
            renderTexture = new RenderTexture(textureWidth, textureHeight, 24, RenderTextureFormat.ARGB32);
            renderTexture.Create();

            renderCamera.targetTexture = renderTexture;

            texture2D = new Texture2D(textureWidth, textureHeight, TextureFormat.RGBA32, false);
            compressor = new TJCompressor();

            InvokeRepeating(nameof(RenderLoop), 0, sendRate);
        }

        void RenderLoop()
        {
            renderCamera.Render();

            AsyncGPUReadback.Request(renderTexture, 0, TextureFormat.RGBA32, OnCompleteReadback);
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.format = format.ToString();

            if (gotFirstFrame)
            {
                data.data = format switch
                {
                    Format.jpeg => compressor.Compress(texture2D.GetRawTextureData(), texture2D.width * 4,
                        texture2D.width,
                        texture2D.height, TJPixelFormat.RGBA, TJSubsamplingOption.Chrominance420, 90, TJFlags.None),
                    Format.png => texture2D.EncodeToPNG(),
                    _ => data.data
                };
                newFrame = false;
            }
            else
                data.data = new byte[] { 0, 0, 0, 255 };

            return base.GetMsg(ts);
        }

        void OnCompleteReadback(AsyncGPUReadbackRequest request)
        {
            if (request.hasError)
            {
                Debug.LogError("GPU readback error detected.");
                return;
            }

            if (texture2D == null) return;

            imageData = request.GetData<Color32>();

            texture2D.LoadRawTextureData(imageData);
            texture2D.Apply();

            gotFirstFrame = true;
            newFrame = true;

        }


        void OnDestroy()
        {
            if (renderTexture != null)
            {
                renderTexture.Release();
            }

            if (texture2D != null)
            {
                Destroy(texture2D);
            }
        }
    }
    
}