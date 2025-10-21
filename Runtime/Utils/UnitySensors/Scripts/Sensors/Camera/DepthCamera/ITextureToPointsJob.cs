// Copyright [2020-2024] Ryodo Tanaka (groadpg@gmail.com) and Akiro Harada
// SPDX-License-Identifier: Apache-2.0

using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnitySensors.Data.PointCloud;

namespace UnitySensors.Sensor.Camera
{
    [BurstCompile]
    public struct ITextureToPointsJob : IJobParallelFor
    {
        public float near;
        public float far;

        [ReadOnly] public NativeArray<float3> directions;
        [ReadOnly] public NativeArray<Color32> depthPixels; // 8-bit/channel
        [ReadOnly] public NativeArray<float> noises;

        public NativeArray<PointXYZ> points;

        public void Execute(int index)
        {
            byte r8 = depthPixels[index].r;
            float r01 = r8 * (1.0f / 255.0f);     // ← scale to 0..1

            // shader outputs: distance01 = 1 - depthMeters/_F
            float distance = (1.0f - math.saturate(r01)) * far; //meters

            float distance_noised = distance + noises[index];
            if (!(near < distance && distance < far && near < distance_noised && distance_noised < far))
                distance_noised = 0f;

            points[index] = new PointXYZ { position = directions[index] * distance_noised };
        }
    }

}