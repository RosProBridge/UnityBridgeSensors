// Copyright [2020-2024] Ryodo Tanaka (groadpg@gmail.com) and Akiro Harada
// SPDX-License-Identifier: Apache-2.0

using System;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.UI;
using UnitySensors.Data.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.Utils.Noise;

using Random = Unity.Mathematics.Random;

namespace UnitySensors.Sensor.Camera
{
    [AddComponentMenu("")]
    public class DepthCameraSensor : CameraSensor, ITextureInterface, IPointCloudInterface<PointXYZ>
    {
        [SerializeField]
        public float _minRange = 0.05f;
        [SerializeField]
        public float _maxRange = 100.0f;
        [SerializeField]
        public float _gaussianNoiseSigma = 0.0f;


        public UnityEngine.Camera _camera;
        private RenderTexture _rt = null;
        private Texture2D _texture;

        private Material _mat;

        private JobHandle _jobHandle;

        private IUpdateGaussianNoisesJob _updateGaussianNoisesJob;
        private ITextureToPointsJob _textureToPointsJob;

        private NativeArray<float> _noises;
        private NativeArray<float3> _directions;

        private PointCloud<PointXYZ> _pointCloud;
        private int _pointsNum;

        public override UnityEngine.Camera m_camera { get => _camera; }
        public Texture2D texture0 { get => _texture; }
        public Texture2D texture1 { get => _texture; }
        public PointCloud<PointXYZ> pointCloud { get => _pointCloud; }
        public int pointsNum { get => _pointsNum; }
        
        

        public override void Init()
        {
            _camera.fieldOfView = _fov;
            _camera.nearClipPlane = _minRange;
            _camera.farClipPlane = _maxRange;

            _rt = new RenderTexture(_resolution.x, _resolution.y, 32, RenderTextureFormat.ARGBFloat);
            _camera.targetTexture = _rt;

            _texture = new Texture2D(_resolution.x, _resolution.y, TextureFormat.RGBAFloat, false);

            _mat = new Material(Shader.Find("UnitySensors/Color2Depth"));
            float f = m_camera.farClipPlane;
            _mat.SetFloat("_F", f);

            SetupDirections();
            SetupJob();
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
            
            _pointCloud = new PointCloud<PointXYZ>()
            {
                points = new NativeArray<PointXYZ>(_pointsNum, Allocator.Persistent)
            };

            _noises = new NativeArray<float>(pointsNum, Allocator.Persistent);
            if(_gaussianNoiseSigma==0f)
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

            _textureToPointsJob = new ITextureToPointsJob()
            {
                near= m_camera.nearClipPlane,
                far = m_camera.farClipPlane,
                directions = _directions,
                noises = _noises,
                points = _pointCloud.points
            };
        }

        public override void UpdateSensor()
        {
            if (!LoadTexture()) return;

            JobHandle jobHandle = default;

            if (_gaussianNoiseSigma != 0f)
            {
                jobHandle = _updateGaussianNoisesJob.Schedule(_pointsNum, 1);
            }

            _textureToPointsJob.depthPixels = _texture.GetPixelData<Color>(0);
            _jobHandle = _textureToPointsJob.Schedule(_pointsNum, 1, jobHandle);

            JobHandle.ScheduleBatchedJobs();
            _jobHandle.Complete();


            if (onSensorUpdated != null)
                onSensorUpdated.Invoke();
        }

        private bool LoadTexture()
        {
            bool result = false;
            AsyncGPUReadback.Request(_rt, 0, request => {
                if (request.hasError)
                {
                }
                else
                {
                    var data = request.GetData<Color>();
                    _texture.LoadRawTextureData(data);
                    _texture.Apply();
                    result = true;
                }
            });
            AsyncGPUReadback.WaitAllRequests();
            return result;
        }

        protected override void OnSensorDestroy()
        {
            _jobHandle.Complete();
            _pointCloud.Dispose();
            _noises.Dispose();
            _directions.Dispose();
            _rt.Release();
        }

        private void OnRenderImage(RenderTexture source, RenderTexture dest)
        {
            Graphics.Blit(source, dest, _mat);
        }
    }
}
