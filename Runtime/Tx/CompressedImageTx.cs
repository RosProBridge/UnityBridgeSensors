﻿using System;
using System.Collections;
using sensor_msgs.msg;
using UnityEngine;

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
        
        public RenderTexture renderTexture;
        public Format format = Format.jpeg;


        private Texture2D tex;
        private WaitForEndOfFrame frameEnd = new WaitForEndOfFrame();
        private bool gotFirstFrame = false;

        protected override void OnStart()
        {
            InvokeRepeating("CallSnapShot", 0, sendRate);
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.format = format.ToString();

            if(gotFirstFrame)
                data.data = format switch
                {
                    Format.jpeg => tex.EncodeToJPG(),
                    Format.png => tex.EncodeToPNG(),
                    _ => data.data
                };
            else
                data.data = new byte[] {0, 0, 0, 255};

            return base.GetMsg(ts);
        }


        private void CallSnapShot()
        {

            StartCoroutine(TakeSnapshot());
        }
        
        public IEnumerator TakeSnapshot()
        {
            
            yield return frameEnd;
            RenderTexture.active = renderTexture;
            tex = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
            gotFirstFrame = true;
            
            RenderTexture.active = null;
        }
    }
}