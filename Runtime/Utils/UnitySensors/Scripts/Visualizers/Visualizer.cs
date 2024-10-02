// Copyright [2020-2024] Ryodo Tanaka (groadpg@gmail.com) and Akiro Harada
// SPDX-License-Identifier: Apache-2.0

using UnityEngine;
using UnitySensors.Sensor;

namespace UnitySensors.Visualization
{
    public abstract class Visualizer<T> : MonoBehaviour where T : UnitySensor
    {
        private T _sensor;
        public T sensor { get => _sensor; }

        private void Start()
        {
            _sensor = GetComponent<T>();
            _sensor.onSensorUpdated += Visualize;

            Init();
        }

        protected abstract void Init();
        protected abstract void Visualize();
    }
}
