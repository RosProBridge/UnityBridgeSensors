// Copyright [2020-2024] Ryodo Tanaka (groadpg@gmail.com) and Akiro Harada
// SPDX-License-Identifier: Apache-2.0

using System;
using Unity.Collections;
using Unity.Mathematics;

namespace UnitySensors.Data.PointCloud
{
    public struct PointCloud<T> : IDisposable where T : struct, IPointInterface
    {
        public NativeArray<T> points;

        public void Dispose()
        {
            points.Dispose();
        }
    }
}