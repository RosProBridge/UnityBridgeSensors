using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnitySensors.Data.PointCloud;

namespace ProBridge.Tx.Sensor
{
    [BurstCompile]
    public struct PointsToPointCloud2MsgJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<PointXYZI> points;
        [ReadOnly] public bool _includeIntensity;

        public NativeArray<byte> data;
        
        public void Execute(int index)
        {
            var tmp = CreateTempArray(index);
            var slice = new NativeSlice<float>(tmp).SliceConvert<byte>();
            var bytesPerPoint = _includeIntensity ? 16 : 12;
            slice.CopyTo(data.GetSubArray(index * bytesPerPoint, bytesPerPoint));
        }

        private NativeArray<float> CreateTempArray(int index)
        {
            var size = _includeIntensity ? 4 : 3;
            var tmp = new NativeArray<float>(size, Allocator.Temp);
            tmp[0] = points[index].position.z;
            tmp[1] = -points[index].position.x;
            tmp[2] = points[index].position.y;
            if (_includeIntensity)
            {
                tmp[3] = points[index].intensity;
            }
            return tmp;
        }

    }
}