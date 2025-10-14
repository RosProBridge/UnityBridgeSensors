
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnitySensors.Data.PointCloud;

[BurstCompile]
public struct DepthMetersToPointsJob : IJobParallelFor
{
    public float near, far;
    [ReadOnly] public NativeArray<float3> directions;
    [ReadOnly] public NativeArray<float> depthMeters;
    [ReadOnly] public NativeArray<float> noises;  
    public bool hasNoises;

    public NativeArray<PointXYZ> points;

    public void Execute(int i)
    {
        float d = depthMeters[i];
        if (hasNoises) d += noises[i];

        bool valid = math.isfinite(d) & (d > near) & (d < far);
        points[i] = new PointXYZ
        {
            position = valid ? directions[i] * d : new float3(float.NaN, float.NaN, float.NaN)
        };
    }
}
