/*
 * Portions of this code are derived from the ROS-TCP-Connector project,
 * originally developed by Unity Technologies and licensed under the Apache License 2.0.
 *
 * Modifications have been made to adapt it for use in this project.
 *
 * You can view the original code and license at:
 * https://github.com/Unity-Technologies/ROS-TCP-Connector
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 */


using System;
using sensor_msgs.msg;
using UnityEngine;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// sensor_msgs/CameraInfo. With an image source (CompressedImageTx, ImageTx) it is sent for every published
    /// frame with the frame's stamp, frame_id and resolution, and sendRate is not used. Without a source it is sent
    /// at sendRate for <see cref="camera"/> with this component's frame_id.
    /// Intrinsics come from the preset; without a preset an ideal pinhole camera is computed from the field of view.
    /// </summary>
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/CameraInfo")]
    public class CameraInfoTx : ProBridgeTxStamped<CameraInfo>
    {
        //The default Camera Info distortion model.
        const string k_PlumbBobDistortionModel = "plumb_bob";

        [Tooltip("Image publisher (CompressedImageTx, ImageTx) to follow: camera_info is sent with every frame, " +
                 "with its stamp, frame_id and resolution. Empty: sent at sendRate for the camera below.")]
        public MonoBehaviour imageSource;

        [Tooltip("Camera without an image source.")]
        public Camera camera;

        [Tooltip("Calibration (intrinsics, distortion). Empty: an ideal pinhole camera from the camera field of view.")]
        public CameraInfoPreset preset;

        private ICameraImageSource _source;
        private bool _hasFrame;
        private TimeSpan _frameStamp;
        private bool _presetSizeWarned;

        private void OnValidate()
        {
            if (imageSource != null && !(imageSource is ICameraImageSource))
            {
                Debug.LogWarning($"{imageSource.GetType().Name} is not a camera image publisher " +
                                 "(CompressedImageTx, ImageTx).", this);
                imageSource = null;
            }
        }

        protected override void AfterEnable()
        {
            _source = imageSource as ICameraImageSource;
            _hasFrame = false;
            if (_source != null)
            {
                // Sent on the step after each frame: check every step.
                sendRate = 0f;
                _source.FramePublished += OnFramePublished;
            }
        }

        protected override void AfterDisable()
        {
            if (_source != null)
                _source.FramePublished -= OnFramePublished;
            _source = null;
        }

        private void OnFramePublished(TimeSpan stamp)
        {
            _frameStamp = stamp;
            _hasFrame = true;
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            Camera cam;
            uint width, height;
            if (_source != null)
            {
                if (!_hasFrame)
                    return null;
                _hasFrame = false;

                ts = _frameStamp;
                cam = _source.ImageCamera;
                width = (uint)_source.ImageWidth;
                height = (uint)_source.ImageHeight;
            }
            else
            {
                cam = camera;
                if (cam == null)
                    return null;
                Rect pixelRect = cam.pixelRect;
                width = (uint)pixelRect.width;
                height = (uint)pixelRect.height;
            }

            if (preset != null)
                FillFromPreset(width, height);
            else if (cam != null)
                FillFromCamera(cam, width, height);
            else
                return null;

            var msg = base.GetMsg(ts);
            if (_source != null)
                data.header.frame_id = _source.ImageFrameId;
            return msg;
        }

        private void FillFromPreset(uint width, uint height)
        {
            if (!_presetSizeWarned && (preset.width != width || preset.height != height))
            {
                Debug.LogWarning($"[{topic}] Camera info preset {preset.name} is for {preset.width}x{preset.height}, " +
                                 $"the image is {width}x{height}.", this);
                _presetSizeWarned = true;
            }

            data.width = preset.width;
            data.height = preset.height;
            data.distortion_model = preset.distortionModel;
            data.d = preset.d;
            Array.Copy(preset.k, data.k, 9);
            Array.Copy(preset.r, data.r, 9);
            Array.Copy(preset.p, data.p, 12);
            data.binning_x = preset.binningX;
            data.binning_y = preset.binningY;
            SetFullRoi();
        }

        /// <summary>
        /// Ideal pinhole camera of the given resolution: focal length from the vertical field of view,
        /// principal point in the image centre, no distortion.
        /// </summary>
        private void FillFromCamera(Camera unityCamera, uint resolutionWidth, uint resolutionHeight)
        {
            if (unityCamera.lensShift != Vector2.zero)
            {
                throw new NotImplementedException(
                    $"Unable to construct CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                    "Lens shift is not yet supported.");
            }

            data.width = resolutionWidth;
            data.height = resolutionHeight;

            //Focal center currently assumes zero lens shift.
            double cX = resolutionWidth / 2.0;
            double cY = resolutionHeight / 2.0;

            //Get the vertical field of view of the camera taking into account any physical camera settings.
            float verticalFieldOfView = GetVerticalFieldOfView(unityCamera, resolutionWidth, resolutionHeight);

            //Sources
            //http://paulbourke.net/miscellaneous/lens/
            //http://ksimek.github.io/2013/06/18/calibrated-cameras-and-gluperspective/
            //Rearranging the equation for verticalFieldOfView given a focal length, determine the focal length in pixels.
            double focalLengthInPixels =
                (resolutionHeight / 2.0) / Math.Tan((Mathf.Deg2Rad * verticalFieldOfView) / 2.0);

            //As this is a perfect pinhole camera, the fx = fy = f
            //Source http://ksimek.github.io/2013/08/13/intrinsic/
            double fX = focalLengthInPixels;
            double fY = focalLengthInPixels;

            //Source: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
            //For a single camera, tX = tY = 0.
            double tX = 0.0;
            double tY = 0.0;

            //Axis Skew, Assuming none.
            double s = 0.0;

            //http://ksimek.github.io/2013/08/13/intrinsic/
            double[] k = data.k;
            k[0] = fX; k[1] = s;  k[2] = cX;
            k[3] = 0;  k[4] = fY; k[5] = cY;
            k[6] = 0;  k[7] = 0;  k[8] = 1;

            //No distortion: "plumb_bob" with d = {k1, k2, t1, t2, k3} = {0, 0, 0, 0, 0}
            data.distortion_model = k_PlumbBobDistortionModel;
            data.d = _zeroDistortion;

            //Rectification matrix (stereo cameras only): identity.
            double[] r = data.r;
            r[0] = 1; r[1] = 0; r[2] = 0;
            r[3] = 0; r[4] = 1; r[5] = 0;
            r[6] = 0; r[7] = 0; r[8] = 1;

            //Projection/camera matrix
            //     [fx'  0  cx' Tx]
            // P = [ 0  fy' cy' Ty]
            //     [ 0   0   1   0]
            double[] p = data.p;
            p[0] = fX; p[1] = 0;  p[2] = cX;  p[3] = tX;
            p[4] = 0;  p[5] = fY; p[6] = cY;  p[7] = tY;
            p[8] = 0;  p[9] = 0;  p[10] = 1;  p[11] = 0;

            //We're not worrying about binning...
            data.binning_x = 0;
            data.binning_y = 0;
            SetFullRoi();
        }

        private readonly double[] _zeroDistortion = new double[5];

        private void SetFullRoi()
        {
            if (data.roi == null)
                data.roi = new RegionOfInterest();
            data.roi.x_offset = 0;
            data.roi.y_offset = 0;
            data.roi.height = 0;
            data.roi.width = 0;
            data.roi.do_rectify = false;
        }

        private static float GetVerticalFieldOfView(Camera camera, uint width, uint height)
        {
            if (camera.usePhysicalProperties)
            {
                //The gateFit may influence the vertical field of view.
                Vector2 sensorSize = camera.sensorSize;

                float sensorRatioY = sensorSize.y / sensorSize.x;
                float pixelRatioY = (float)height / width;
                float fovMultiplier = pixelRatioY / sensorRatioY;

                switch (camera.gateFit)
                {
                    case Camera.GateFitMode.Vertical:
                        //The fieldOfView from the camera is accurate, return it.
                        return camera.fieldOfView;
                    case Camera.GateFitMode.Horizontal:
                        //The fieldOfView from the camera is influenced by the ratio of the pixels vs the sensor size ratio.
                        return camera.fieldOfView * fovMultiplier;
                    case Camera.GateFitMode.Fill:
                        //Same as GateFitMode.Vertical or Horizontal
                        return fovMultiplier >= 1.0f ? camera.fieldOfView : camera.fieldOfView * fovMultiplier;
                    case Camera.GateFitMode.Overscan:
                        //Same as GateFitMode.Vertical or Horizontal
                        return fovMultiplier <= 1.0f ? camera.fieldOfView : camera.fieldOfView * fovMultiplier;
                    case Camera.GateFitMode.None:
                        //The view is stretched, the fieldOfView is valid.
                        return camera.fieldOfView;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }

            //The fieldOfView from the camera is accurate, return it.
            return camera.fieldOfView;
        }
    }
}
