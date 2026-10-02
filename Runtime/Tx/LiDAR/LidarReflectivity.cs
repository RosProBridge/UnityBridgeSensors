using UnityEngine;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// Lidar reflectivity of the colliders on this object and its children.
    /// Overrides the value estimated from the material albedo.
    /// </summary>
    [AddComponentMenu("ProBridge/Sensors/Lidar Reflectivity")]
    public class LidarReflectivity : MonoBehaviour
    {
        /// <summary>Typical surface (asphalt ~0.1, concrete ~0.3, vegetation ~0.5, white paint ~0.8).</summary>
        public const float DefaultValue = 0.3f;

        [Tooltip("Diffuse (Lambertian) reflectivity at the lidar wavelength, 0..1. " +
                 "Asphalt ~0.1, concrete ~0.3, vegetation ~0.5, white paint ~0.8.")]
        [Range(0f, 1f)]
        public float reflectivity = DefaultValue;

        [Tooltip("Retroreflector (road signs, reflectors, license plates): reported above the diffuse range " +
                 "and does not fade with the incidence angle.")]
        public bool retroreflective;

        /// <summary>Incremented on any change, so lidars drop their cached values.</summary>
        public static int Version { get; private set; }

        private void OnEnable() => Version++;
        private void OnDisable() => Version++;
        private void OnValidate() => Version++;
    }
}
