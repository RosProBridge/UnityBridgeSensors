using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnitySensors.Data.PointCloud;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// Pattern point of the i-th ray in the current scan: (start + i * stride) % patternSize.
    /// </summary>
    public struct ScanWindow
    {
        public int start;
        public int stride;
        public int patternSize;

        public int PatternIndex(int index) => (int)(((long)start + (long)index * stride) % patternSize);
    }

    [BurstCompile]
    public struct BuildRaycastCommandsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> directions;
        public ScanWindow window;
        public float3 origin;
        public quaternion rotation;
        public float maxRange;

        [WriteOnly] public NativeArray<RaycastCommand> commands;

        public void Execute(int index)
        {
            float3 direction = math.normalize(math.mul(rotation, directions[window.PatternIndex(index)]));
#if UNITY_2022_2_OR_NEWER
            commands[index] = new RaycastCommand(origin, direction, QueryParameters.Default, maxRange);
#else
            commands[index] = new RaycastCommand(origin, direction, maxRange);
#endif
        }
    }

    [BurstCompile]
    public struct RaycastHitsToPointsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> directions;
        [ReadOnly] public NativeArray<RaycastHit> hits;
        [ReadOnly] public NativeParallelHashMap<LidarColliderId, float> reflectivities;
        public NativeParallelHashSet<LidarColliderId>.ParallelWriter unknownColliders;

        public ScanWindow window;
        public quaternion rotation;
        public float minRange;
        public float maxRange;
        public float noiseSigma;
        public uint noiseSeed;

        public float defaultReflectivity;
        public float diffuseMaxIntensity;
        public float maxIntensity;
        public float rangeFalloff;

        [WriteOnly] public NativeArray<PointXYZI> points;

        public void Execute(int index)
        {
            var hit = hits[index];
            float distance = hit.distance;
            if (noiseSigma > 0f && distance > 0f)
                distance += noiseSigma * GaussianNoise(index);

            if (!(minRange < distance && distance < maxRange))
            {
                points[index] = default; // dropped by FilterZeroPointsParallelJob
                return;
            }

            float3 direction = math.normalize(directions[window.PatternIndex(index)]);

            var collider = LidarColliderId.From(hit);
            if (!reflectivities.TryGetValue(collider, out float reflectivity))
            {
                unknownColliders.Add(collider);
                reflectivity = defaultReflectivity;
            }

            points[index] = new PointXYZI
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
