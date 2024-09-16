// Copyright [2020-2024] Ryodo Tanaka (groadpg@gmail.com) and Akiro Harada
// SPDX-License-Identifier: Apache-2.0

using Unity.Mathematics;

namespace UnitySensors.Data.PointCloud
{
    public interface IPointXYZInterface : IPointInterface
    {
        public float3 position { get; }
    }
}