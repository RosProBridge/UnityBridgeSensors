// Copyright [2020-2024] Ryodo Tanaka (groadpg@gmail.com) and Akiro Harada
// SPDX-License-Identifier: Apache-2.0

using UnityEngine;
using UnityEditor;
using Unity.Mathematics;

using UnitySensors.Attribute;

namespace UnitySensors.Sensor.LiDAR
{
    public class ScanPattern : ScriptableObject
    {
        [SerializeField, HideInInspector]
        public float3[] scans;
        [SerializeField, ReadOnly]
        public int size;
        [SerializeField, ReadOnly]
        public float minZenithAngle;
        [SerializeField, ReadOnly]
        public float maxZenithAngle;
        [SerializeField, ReadOnly]
        public float minAzimuthAngle;
        [SerializeField, ReadOnly]
        public float maxAzimuthAngle;
    }
}
