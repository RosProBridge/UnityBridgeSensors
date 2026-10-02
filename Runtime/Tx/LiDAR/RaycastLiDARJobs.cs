using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnitySensors.Data.PointCloud;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// Pattern point of the i-th ray in the current scan. The pattern is split into columns of
    /// <see cref="columnSize"/> consecutive points (e.g. 16 rings of a VLP-16 at one azimuth; 1 for Livox):
    /// a scan takes every <see cref="stride"/>-th column starting from column <see cref="start"/>, with all its points.
    /// </summary>
    public struct ScanWindow
    {
        public int start;
        public int stride;
        public int columnSize;
        public int patternSize;

        public int PatternIndex(int index)
        {
            int column = index / columnSize;
            int columns = patternSize / columnSize;
            long patternColumn = ((long)start + (long)column * stride) % columns;
            return (int)(patternColumn * columnSize + index % columnSize);
        }
    }

    /// <summary>Builds the rays of one scan slice: scan points [scanOffset, scanOffset + count).</summary>
    [BurstCompile]
    public struct BuildRaycastCommandsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> directions;
        public ScanWindow window;
        public int scanOffset;
        public float3 origin;
        public quaternion rotation;
        public float maxRange;

        [WriteOnly] public NativeArray<RaycastCommand> commands;

        public void Execute(int index)
        {
            float3 direction = math.normalize(math.mul(rotation, directions[window.PatternIndex(scanOffset + index)]));
#if UNITY_2022_2_OR_NEWER
            commands[index] = new RaycastCommand(origin, direction, QueryParameters.Default, maxRange);
#else
            commands[index] = new RaycastCommand(origin, direction, maxRange);
#endif
        }
    }

    /// <summary>Turns the hits of one scan slice into scan points [scanOffset, scanOffset + count).</summary>
    [BurstCompile]
    public struct RaycastHitsToPointsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> directions;
        [ReadOnly] public NativeArray<RaycastHit> hits;
        [ReadOnly] public NativeParallelHashMap<LidarColliderId, float> reflectivities;
        public NativeParallelHashSet<LidarColliderId>.ParallelWriter unknownColliders;

        public ScanWindow window;
        public int scanOffset;
        public quaternion rotation;
        public float minRange;
        public float maxRange;
        public float noiseSigma;
        public uint noiseSeed;

        public float defaultReflectivity;
        public float diffuseMaxIntensity;
        public float maxIntensity;
        public float rangeFalloff;

        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<PointXYZI> points;

        public void Execute(int index)
        {
            int scanIndex = scanOffset + index;
            var hit = hits[index];
            float distance = hit.distance;
            if (noiseSigma > 0f && distance > 0f)
                distance += noiseSigma * GaussianNoise(scanIndex);

            if (!(minRange < distance && distance < maxRange))
            {
                points[scanIndex] = default; // dropped by PackPointCloud2Job
                return;
            }

            float3 direction = math.normalize(directions[window.PatternIndex(scanIndex)]);

            var collider = LidarColliderId.From(hit);
            if (!reflectivities.TryGetValue(collider, out float reflectivity))
            {
                unknownColliders.Add(collider);
                reflectivity = defaultReflectivity;
            }

            points[scanIndex] = new PointXYZI
            {
                position = direction * distance,
                intensity = Intensity(reflectivity, hit.normal, direction, distance)
            };
        }

        private float Intensity(float reflectivity, float3 normal, float3 localDirection, float distance)
        {
            float range01 = math.saturate((distance - minRange) / (maxRange - minRange));
            float falloff = 1f - rangeFalloff * range01;

            if (reflectivity >= LidarReflectivityResolver.RetroOffset)
            {
                float retro = reflectivity - LidarReflectivityResolver.RetroOffset;
                return diffuseMaxIntensity + (maxIntensity - diffuseMaxIntensity) * retro * falloff;
            }

            float3 worldDirection = math.mul(rotation, localDirection);
            float cosIncidence = math.saturate(-math.dot(normal, worldDirection));
            return diffuseMaxIntensity * reflectivity * cosIncidence * falloff;
        }

        private float GaussianNoise(int index)
        {
            var random = Unity.Mathematics.Random.CreateFromIndex(noiseSeed + (uint)index);
            float u1 = math.max(random.NextFloat(), 1e-7f);
            float u2 = random.NextFloat();
            return math.sqrt(-2f * math.log(u1)) * math.cos(2f * math.PI * u2);
        }
    }

    /// <summary>
    /// Packs the valid points (non-zero position) into PointCloud2 data: x, y, z [, intensity] as float32,
    /// converted from Unity (left-handed, Y up) to ROS (right-handed, Z up). Writes the point count to count[0].
    /// </summary>
    [BurstCompile]
    public struct PackPointCloud2Job : IJob
    {
        [ReadOnly] public NativeArray<PointXYZI> points;
        public bool includeIntensity;

        [WriteOnly] public NativeArray<float> data;
        [WriteOnly] public NativeArray<int> count;

        public void Execute()
        {
            int stride = includeIntensity ? 4 : 3;
            int n = 0;
            for (int i = 0; i < points.Length; i++)
            {
                var p = points[i];
                if (p.position.x == 0f && p.position.y == 0f && p.position.z == 0f)
                    continue;

                int o = n * stride;
                data[o] = p.position.z;
                data[o + 1] = -p.position.x;
                data[o + 2] = p.position.y;
                if (includeIntensity)
                    data[o + 3] = p.intensity;
                n++;
            }
            count[0] = n;
        }
    }

    /// <summary>
    /// Collider id of a raycast hit. Unity 6.5 replaces instance IDs with <c>EntityId</c>;
    /// all version-dependent code is kept here.
    /// </summary>
    public readonly struct LidarColliderId : System.IEquatable<LidarColliderId>
    {
#if UNITY_6000_5_OR_NEWER
        private readonly EntityId _id;
        private LidarColliderId(EntityId id) => _id = id;

        public static LidarColliderId From(in RaycastHit hit) => new LidarColliderId(hit.colliderEntityId);
        public Collider ToCollider() => Resources.EntityIdToObject(_id) as Collider;
#else
        private readonly int _id;
        private LidarColliderId(int id) => _id = id;

        public static LidarColliderId From(in RaycastHit hit) => new LidarColliderId(hit.colliderInstanceID);
        public Collider ToCollider() => Resources.InstanceIDToObject(_id) as Collider;
#endif

        public bool Equals(LidarColliderId other) => _id.Equals(other._id);
        public override bool Equals(object obj) => obj is LidarColliderId other && Equals(other);
        public override int GetHashCode() => _id.GetHashCode();
    }
}
