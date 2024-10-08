using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnitySensors.Data.PointCloud;

[BurstCompile]
public struct FilterZeroPointsParallelJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<PointXYZI> inputArray;
    [WriteOnly] public NativeQueue<PointXYZI>.ParallelWriter outputQueue;

    public void Execute(int index)
    {
        if (!(inputArray[index].position.x == 0 && inputArray[index].position.y == 0 &&
             inputArray[index].position.z == 0))
        {
            outputQueue.Enqueue(inputArray[index]);
        }
    }
}