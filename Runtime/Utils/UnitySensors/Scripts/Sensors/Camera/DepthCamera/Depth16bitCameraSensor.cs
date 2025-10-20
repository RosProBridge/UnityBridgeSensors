// Copyright [2020-2024] Ryodo Tanaka (groadpg@gmail.com) and Akiro Harada
// SPDX-License-Identifier: Apache-2.0

using System;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Serialization;
using UnityEngine.UI;
using UnitySensors.Data.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.Utils.Noise;
using Random = Unity.Mathematics.Random;
using UnityEngine.EventSystems;

namespace UnitySensors.Sensor.Camera
{
    [AddComponentMenu("")]
    public class Depth16bitCameraSensor : CameraSensor, ITextureInterface, IPointCloudInterface<PointXYZ>
    {
        [SerializeField] public float _minRange = 0.05f;
        [SerializeField] public float _maxRange = 100.0f;
        [SerializeField] public float _gaussianNoiseSigma = 0.0f;


        public UnityEngine.Camera _camera;
        private RenderTexture _rt = null;
        private Texture2D _texture;

        public Material mat;

        private JobHandle _jobHandle;

        private IUpdateGaussianNoisesJob _updateGaussianNoisesJob;
        private DepthMetersToPointsJob _depthMetersToPointsJob;

        private NativeArray<float> _noises;
        private NativeArray<float3> _directions;

        private PointCloud<PointXYZ> _pointCloud;
        private int _pointsNum;

        private NativeArray<float> _depthMeters; 

        public override UnityEngine.Camera m_camera
        {
            get => _camera;
        }

        public Texture2D texture0
        {
            get => _texture;
        }

        public Texture2D texture1
        {
            get => _texture;
        }

        public PointCloud<PointXYZ> pointCloud
        {
            get => _pointCloud;
        }

        public int pointsNum
        {
            get => _pointsNum;
        }

        public bool getPointCloud = false;

        // fields to reuse buffers (avoid GC)
        private byte[] _r16Bytes;   // w*h*2
        private int _w, _h;
        public override void Init()
        {
            _camera.fieldOfView = _fov;
            _camera.nearClipPlane = _minRange;
            _camera.farClipPlane = _maxRange;

            float f = m_camera.farClipPlane;
            mat.SetFloat("_F", f);

            var desc = new RenderTextureDescriptor(_resolution.x, _resolution.y)
            {
                graphicsFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.R16_UNorm,
                depthBufferBits = 0,
                msaaSamples = 1,
                sRGB = false
            };
            _rt = new RenderTexture(desc);
            _camera.targetTexture = _rt;
            _camera.depthTextureMode |= DepthTextureMode.Depth;

            _w = _resolution.x; _h = _resolution.y;
            _r16Bytes = new byte[_w * _h * 2];           // staged buffer

            SetupDirections();
            SetupJob();
            Initialized = true;
        }

        private void SetupDirections()
        {
            _pointsNum = _resolution.x * _resolution.y;

            _directions = new NativeArray<float3>(_pointsNum, Allocator.Persistent);

            float z = _resolution.y * 0.5f / Mathf.Tan(m_camera.fieldOfView * 0.5f * Mathf.Deg2Rad);
            for (int y = 0; y < _resolution.y; y++)
            {
                for (int x = 0; x < _resolution.x; x++)
                {
                    Vector3 vec = new Vector3(-_resolution.x / 2 + x, -_resolution.y / 2 + y, z);
                    _directions[y * _resolution.x + x] = vec.normalized;
                }
            }
        }

        private void SetupJob()
        {
            if (!getPointCloud) return;
            _pointCloud = new PointCloud<PointXYZ>()
            {
                points = new NativeArray<PointXYZ>(_pointsNum, Allocator.Persistent)
            };
            _depthMeters = new NativeArray<float>(_pointsNum, Allocator.Persistent);
            _noises = new NativeArray<float>(pointsNum, Allocator.Persistent);
            if (_gaussianNoiseSigma == 0f)
            {
                for (int i = 0; i < pointsNum; i++)
                {
                    _noises[i] = 0f;
                }
            }
            else
            {
                _updateGaussianNoisesJob = new IUpdateGaussianNoisesJob()
                {
                    sigma = _gaussianNoiseSigma,
                    random = new Random((uint)Environment.TickCount),
                    noises = _noises
                };
            }

            _depthMetersToPointsJob = new DepthMetersToPointsJob()
            {
                near = _camera.nearClipPlane,
                far = _camera.farClipPlane,
                directions = _directions,
                depthMeters = _depthMeters,
                noises = _noises,
                hasNoises = (_gaussianNoiseSigma != 0f),
                points = _pointCloud.points
            };
        }

        public override void UpdateSensor()
        {
            if (!LoadTexture()) return;

            if (getPointCloud)
            {
                // Convert R16 -> meters (NO vertical flip here)
                int W = _w, H = _h;
                float farM = _camera.farClipPlane;

                int idx = 0;
                for (int y = 0; y < H; y++)
                {
                    int rowOff = y * W * 2;
                    for (int x = 0; x < W; x++, idx++)
                    {
                        int off = rowOff + (x << 1);
                        int u16 = (_r16Bytes[off + 1] << 8) | _r16Bytes[off]; // little-endian
                        float norm = u16 * (1.0f / 65535.0f);
                        float dMeters = (1.0f - norm) * farM; // undo shader's normalization
                        _depthMeters[idx] = dMeters;
                    }
                }

                JobHandle jh = default;
                if (_gaussianNoiseSigma != 0f)
                    jh = _updateGaussianNoisesJob.Schedule(_pointsNum, 128);

                _jobHandle = _depthMetersToPointsJob.Schedule(_pointsNum, 128, jh);
                JobHandle.ScheduleBatchedJobs();
                _jobHandle.Complete();
            }

            if (onSensorUpdated != null)
                onSensorUpdated.Invoke();
        }

        private bool LoadTexture()
        {
            bool result = false;
            AsyncGPUReadback.Request(_rt, 0, TextureFormat.R16, request =>
            {
                if (!request.hasError)
                { 
                    var data = request.GetData<byte>();                 // tightly packed R16
                                                                   // copy into our reusable managed buffer (NativeArray becomes invalid later)
                    data.CopyTo(_r16Bytes);
                    result = true;
                }
            });
            AsyncGPUReadback.WaitAllRequests();
            return result;
        }

        protected override void OnSensorDestroy()
        {
        }

        private void OnRenderImage(RenderTexture source, RenderTexture dest)
        {
            Graphics.Blit(source, dest, mat);
        }

        public void DisposeSensor()
        {
            _jobHandle.Complete();
            _pointCloud.Dispose();
            _noises.Dispose();
            _directions.Dispose();
            _rt.Release();

            if (_depthMeters.IsCreated) _depthMeters.Dispose();
        }

        // Expose the latest packed R16 bytes to the publisher
        public ReadOnlySpan<byte> LatestR16Bytes() => _r16Bytes;
        public int Width => _w;
        public int Height => _h;

        public bool Initialized { get; private set; }

    }
}