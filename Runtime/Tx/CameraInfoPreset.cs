using System;
using System.Globalization;
using System.Text.RegularExpressions;
using UnityEngine;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// Camera calibration for <see cref="CameraInfoTx"/>: resolution, intrinsics and distortion, as in
    /// sensor_msgs/CameraInfo. Assign it like a physics material; without it CameraInfoTx computes an ideal
    /// pinhole camera from the Unity camera field of view. A ROS calibration file (camera_calibration YAML)
    /// can be imported from the asset's context menu.
    /// </summary>
    [CreateAssetMenu(fileName = "CameraInfo", menuName = "ProBridge/Camera Info")]
    public class CameraInfoPreset : ScriptableObject
    {
        [Tooltip("Calibrated image width, px. Should match the published image.")]
        public uint width = 640;
        [Tooltip("Calibrated image height, px. Should match the published image.")]
        public uint height = 480;

        [Tooltip("plumb_bob (k1, k2, t1, t2, k3), rational_polynomial (8 coefficients) or equidistant (4).")]
        public string distortionModel = "plumb_bob";
        [Tooltip("Distortion coefficients D.")]
        public double[] d = new double[5];

        [Tooltip("Intrinsic matrix K, 3x3 row-major: fx 0 cx / 0 fy cy / 0 0 1.")]
        public double[] k = { 0, 0, 0, 0, 0, 0, 0, 0, 1 };
        [Tooltip("Rectification matrix R, 3x3 row-major (identity for a monocular camera).")]
        public double[] r = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        [Tooltip("Projection matrix P, 3x4 row-major: fx' 0 cx' Tx / 0 fy' cy' Ty / 0 0 1 0.")]
        public double[] p = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };

        public uint binningX;
        public uint binningY;

        void OnValidate()
        {
            Array.Resize(ref k, 9);
            Array.Resize(ref r, 9);
            Array.Resize(ref p, 12);
            if (d == null) d = new double[0];
        }

#if UNITY_EDITOR
        [ContextMenu("Import ROS calibration YAML...")]
        void ImportYaml()
        {
            string path = UnityEditor.EditorUtility.OpenFilePanel("ROS camera calibration", "", "yaml,yml");
            if (string.IsNullOrEmpty(path))
                return;

            try
            {
                string yaml = System.IO.File.ReadAllText(path);
                UnityEditor.Undo.RecordObject(this, "Import camera calibration");
                FromYaml(yaml);
                UnityEditor.EditorUtility.SetDirty(this);
                Debug.Log($"[CameraInfoPreset] {name}: imported {path}", this);
            }
            catch (Exception e)
            {
                Debug.LogError($"[CameraInfoPreset] {name}: can't import {path}: {e.Message}", this);
            }
        }
#endif

        /// <summary>Fills the preset from a camera_calibration YAML (image_width, camera_matrix, ...).</summary>
        public void FromYaml(string yaml)
        {
            width = (uint)ReadNumber(yaml, "image_width");
            height = (uint)ReadNumber(yaml, "image_height");
            distortionModel = Regex.Match(yaml, @"distortion_model:\s*([\w]+)").Groups[1].Value;
            if (distortionModel == "") distortionModel = "plumb_bob";
            k = ReadMatrix(yaml, "camera_matrix", 9);
            d = ReadMatrix(yaml, "distortion_coefficients", -1);
            r = ReadMatrix(yaml, "rectification_matrix", 9);
            p = ReadMatrix(yaml, "projection_matrix", 12);
        }

        static double ReadNumber(string yaml, string key)
        {
            Match m = Regex.Match(yaml, $@"(?m)^\s*{key}:\s*([-+0-9.eE]+)");
            if (!m.Success) throw new FormatException($"no {key}");
            return double.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture);
        }

        static double[] ReadMatrix(string yaml, string key, int expected)
        {
            Match m = Regex.Match(yaml, $@"(?s){key}:.*?data:\s*\[(.*?)\]");
            if (!m.Success) throw new FormatException($"no {key}.data");

            string[] items = m.Groups[1].Value.Split(new[] { ',', ' ', '\n', '\r', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            var values = new double[items.Length];
            for (int i = 0; i < items.Length; i++)
                values[i] = double.Parse(items[i], CultureInfo.InvariantCulture);

            if (expected > 0 && values.Length != expected)
                throw new FormatException($"{key}.data has {values.Length} values, expected {expected}");
            return values;
        }
    }
}
