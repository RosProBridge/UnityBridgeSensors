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
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/CameraInfo")]
    public class CameraInfoTx : ProBridgeTxStamped<CameraInfo>
    {
        //The default Camera Info distortion model.
        const string k_PlumbBobDistortionModel = "plumb_bob";


        public Camera camera;

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            ConstructCameraInfoMessage(camera);
            return base.GetMsg(ts);
        }

        public void ConstructCameraInfoMessage(Camera unityCamera,
            float horizontalCameraOffsetDistanceMeters = 0.0f, float integerResolutionTolerance = 0.01f)
        {
            Rect pixelRect = unityCamera.pixelRect;
            var resolutionWidth = (uint)pixelRect.width;
            var resolutionHeight = (uint)pixelRect.height;

            //Check whether the resolution is an integer value, if not, raise a warning.
            //Note: While the resolution of a screen or a render texture is always an integer value,
            //      one can change the rendering region within the screen / texture using the
            //      viewport rect. It is possible that this region will be a non-integer resolution.
            //      since the resolution of the CameraInfo message is stored as a uint,
            //      non-integer values are not supported
            if ((pixelRect.width - (float)resolutionWidth) > integerResolutionTolerance)
            {
                Debug.LogWarning($"CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                                 $"Resolution width is not an integer: {pixelRect.width}. Adjust the viewport rect.");
            }

            if ((pixelRect.height - (float)resolutionHeight) > integerResolutionTolerance)
            {
                Debug.LogWarning($"CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                                 $"Resolution height is not an integer: {pixelRect.height}. Adjust the viewport rect.");
            }

            if (resolutionWidth != unityCamera.scaledPixelWidth || resolutionHeight != unityCamera.scaledPixelHeight)
            {
                //Check for resolution scaling (Not Implemented). TODO - Implement.
                throw new NotImplementedException(
                    $"Unable to construct CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                    $"Resolution scaling is not yet supported.");
            }

            if (unityCamera.lensShift != Vector2.zero)
            {
                throw new NotImplementedException(
                    $"Unable to construct CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                    "Lens shift is not yet supported.");
            }

            data.width = resolutionWidth;
            data.height = resolutionHeight;

            //Focal center currently assumes zero lens shift.
            //Focal center x.
            double cX = resolutionWidth / 2.0;
            //Focal center y.
            double cY = resolutionHeight / 2.0;

            //Get the vertical field of view of the camera taking into account any physical camera settings.
            float verticalFieldOfView = GetVerticalFieldOfView(unityCamera);

            //Sources
            //http://paulbourke.net/miscellaneous/lens/
            //http://ksimek.github.io/2013/06/18/calibrated-cameras-and-gluperspective/
            //Rearranging the equation for verticalFieldOfView given a focal length, determine the focal length in pixels.
            double focalLengthInPixels =
                (resolutionHeight / 2.0) / Math.Tan((Mathf.Deg2Rad * verticalFieldOfView) / 2.0);

            //As this is a perfect pinhole camera, the fx = fy = f
            //Source http://ksimek.github.io/2013/08/13/intrinsic/
            //Focal Length (x)
            double fX = focalLengthInPixels;
            //Focal Length (y)
            double fY = focalLengthInPixels;

            //Source: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
            //For a single camera, tX = tY = 0.
            //For a stereo camera, assuming Tz = 0, Ty = 0 and Tx = -fx' * B (for the second camera)
            double baseline = horizontalCameraOffsetDistanceMeters;

            double tX = -fX * baseline;
            double tY = 0.0;

            //Axis Skew, Assuming none.
            double s = 0.0;

            //http://ksimek.github.io/2013/08/13/intrinsic/
            data.k = new double[]
            {
                fX, s, cX,
                0, fY, cY,
                0, 0, 1
            };

            //The distortion parameters, size depending on the distortion model.
            //For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
            //No distortion means d = {k1, k2, t1, t2, k3} = {0, 0, 0, 0, 0}
            data.distortion_model = k_PlumbBobDistortionModel;
            data.d = new double[]
            {
                0.0, //k1
                0.0, //k2
                0.0, //t1
                0.0, //t2
                0.0 //k3
            };

            //Rectification matrix (stereo cameras only)
            //A rotation matrix aligning the camera coordinate system to the ideal
            //stereo image plane so that epipolar lines in both stereo images are
            //parallel.
            data.r = new double[]
            {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
            };


            //Projection/camera matrix
            //     [fx'  0  cx' Tx]
            // P = [ 0  fy' cy' Ty]
            //     [ 0   0   1   0]
            data.p = new double[]
            {
                fX, 0, cX, tX,
                0, fY, cY, tY,
                0, 0, 1, 0
            };

            //We're not worrying about binning...
            data.binning_x = 0;
            data.binning_y = 0;

            data.roi = new RegionOfInterest
            {
                x_offset = 0,
                y_offset = 0,
                height = 0,
                width = 0,
                do_rectify = false
            };
        }


        private static float GetVerticalFieldOfView(Camera camera)
        {
            if (camera.usePhysicalProperties)
            {
                //The gateFit may influence the vertical field of view.
                Vector2 sensorSize = camera.sensorSize;
                Rect pixelRect = camera.pixelRect;

                float sensorRatioY = sensorSize.y / sensorSize.x;
                float pixelRatioY = pixelRect.height / pixelRect.width;
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
                        if (fovMultiplier >= 1.0f)
                        {
                            //Same as GateFitMode.Vertical
                            return camera.fieldOfView;
                        }
                        else
                        {
                            //Same as GateFitMode.Horizontal
                            return camera.fieldOfView * fovMultiplier;
                        }
                    case Camera.GateFitMode.Overscan:
                        if (fovMultiplier <= 1.0f)
                        {
                            //Same as GateFitMode.Vertical
                            return camera.fieldOfView;
                        }
                        else
                        {
                            //Same as GateFitMode.Horizontal
                            return camera.fieldOfView * fovMultiplier;
                        }
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