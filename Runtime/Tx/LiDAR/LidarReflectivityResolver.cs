using System.Collections.Generic;
using UnityEngine;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// Resolves the lidar reflectivity of a collider (main thread, cached per material/texture).
    /// Encoded value: 0..1 diffuse reflectivity, RetroOffset + 0..1 for retroreflectors.
    /// </summary>
    public static class LidarReflectivityResolver
    {
        public const float RetroOffset = 2f;

        // Real surfaces rarely reflect more than ~90% or less than ~2%.
        private const float MinAlbedo = 0.02f;
        private const float MaxAlbedo = 0.9f;

        private static readonly Dictionary<Material, float> MaterialCache = new Dictionary<Material, float>();
        private static readonly Dictionary<Texture, Color> TextureCache = new Dictionary<Texture, Color>();
        private static readonly Dictionary<TerrainData, float> TerrainCache = new Dictionary<TerrainData, float>();

        public static float Resolve(Collider collider, float defaultReflectivity)
        {
            if (!collider)
                return defaultReflectivity;

            var custom = collider.GetComponentInParent<LidarReflectivity>();
            if (custom)
                return custom.retroreflective ? RetroOffset + custom.reflectivity : custom.reflectivity;

            if (collider is TerrainCollider terrainCollider)
                return FromTerrain(terrainCollider.terrainData, defaultReflectivity);

            var renderer = collider.GetComponent<Renderer>();
            if (!renderer) renderer = collider.GetComponentInChildren<Renderer>();
            if (!renderer) renderer = collider.GetComponentInParent<Renderer>();
            if (!renderer || !renderer.sharedMaterial)
                return defaultReflectivity;

            return FromMaterial(renderer.sharedMaterial);
        }

        private static float FromMaterial(Material material)
        {
            if (MaterialCache.TryGetValue(material, out var cached))
                return cached;

            var color = material.HasProperty("_BaseColor") ? material.GetColor("_BaseColor")
                : material.HasProperty("_Color") ? material.GetColor("_Color")
                : Color.white;
            var texture = material.HasProperty("_BaseMap") ? material.GetTexture("_BaseMap")
                : material.HasProperty("_MainTex") ? material.GetTexture("_MainTex")
                : null; // e.g. Shader Graph materials without a main texture

            var albedo = color.linear;
            if (texture)
                albedo *= AverageLinearColor(texture);

            return MaterialCache[material] = ToReflectivity(albedo);
        }

        private static float FromTerrain(TerrainData terrainData, float defaultReflectivity)
        {
            if (!terrainData)
                return defaultReflectivity;
            if (TerrainCache.TryGetValue(terrainData, out var cached))
                return cached;

            // One value for the whole terrain: the average of its layers.
            var sum = Color.black;
            int count = 0;
            foreach (var layer in terrainData.terrainLayers)
            {
                if (!layer || !layer.diffuseTexture) continue;
                sum += AverageLinearColor(layer.diffuseTexture);
                count++;
            }

            return TerrainCache[terrainData] = count > 0 ? ToReflectivity(sum / count) : defaultReflectivity;
        }

        private static float ToReflectivity(Color linearAlbedo)
        {
            float luminance = 0.2126f * linearAlbedo.r + 0.7152f * linearAlbedo.g + 0.0722f * linearAlbedo.b;
            return Mathf.Clamp(luminance, MinAlbedo, MaxAlbedo);
        }

        /// <summary>
        /// Average texture color: a blit into 1x1 samples the lowest mip. Works for non-readable textures.
        /// </summary>
        private static Color AverageLinearColor(Texture texture)
        {
            if (TextureCache.TryGetValue(texture, out var cached))
                return cached;

            var rt = RenderTexture.GetTemporary(1, 1, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
            var previous = RenderTexture.active;
            Graphics.Blit(texture, rt);
            RenderTexture.active = rt;

            var pixel = new Texture2D(1, 1, TextureFormat.RGBA32, false, true);
            pixel.ReadPixels(new Rect(0, 0, 1, 1), 0, 0, false);
            var color = pixel.GetPixel(0, 0);

            RenderTexture.active = previous;
            RenderTexture.ReleaseTemporary(rt);
            if (Application.isPlaying) Object.Destroy(pixel);
            else Object.DestroyImmediate(pixel);

            // In gamma color space sRGB textures are not decoded on sampling.
            if (QualitySettings.activeColorSpace == ColorSpace.Gamma)
                color = color.linear;

            return TextureCache[texture] = color;
        }
    }
}
