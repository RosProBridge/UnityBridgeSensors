using System;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// Renders a camera on demand into its target texture and reads the frame back to the CPU.
    /// While the capture exists the camera is disabled, so it costs nothing between captures.
    /// Several captures of one camera (e.g. raw and compressed publishers) share its target texture.
    /// Rows of a frame go bottom-up, as in texture data.
    /// </summary>
    internal sealed class CameraCapture : IDisposable
    {
        /// <summary>Target texture and saved camera state, shared by all captures of one camera.</summary>
        private sealed class Target
        {
            public RenderTexture texture;
            public bool ownsTexture;
            public bool cameraWasEnabled;
            public int users;
            public int renderedFrame = -1;
        }

        private static readonly Dictionary<Camera, Target> Targets = new Dictionary<Camera, Target>();

        private readonly Camera _camera;
        private readonly Target _target;
        private readonly TextureFormat _format;
        private readonly Action<NativeArray<byte>> _onFrame;

        private NativeArray<byte> _buffer;
        private bool _pending;
        private bool _disposed;

        public int Width { get; }
        public int Height { get; }

        /// <summary>A capture is in flight: the previous frame has not been read back yet.</summary>
        public bool Busy => _pending;

        private double _period;
        private double _nextCapture;

        /// <summary>
        /// Sets the capture period (seconds of game time). The first capture gets a random phase, so cameras with
        /// the same period don't render in the same frame.
        /// </summary>
        public void SetPeriod(float period)
        {
            _period = period;
            _nextCapture = Time.timeAsDouble + UnityEngine.Random.Range(0f, period);
        }

        /// <summary>
        /// The next capture is due and the previous one has been read back. A slot that comes while a capture
        /// is in flight is not lost: the capture happens as soon as the previous one completes.
        /// </summary>
        public bool IsDue => !_disposed && !_pending && Time.timeAsDouble >= _nextCapture;

        /// <summary>Returns null and logs an error if the camera cannot be captured.</summary>
        public static CameraCapture Create(Camera camera, int width, int height, TextureFormat format,
            Action<NativeArray<byte>> onFrame, UnityEngine.Object context)
        {
            if (camera == null)
            {
                Debug.LogWarning("Render camera is not set.", context);
                return null;
            }

            var texture = Targets.TryGetValue(camera, out var shared) ? shared.texture : camera.targetTexture;
            if (texture != null && (texture.width != width || texture.height != height))
            {
                Debug.LogError($"RenderTexture dimensions are incorrect. Expected {width}x{height}, " +
                               $"but got {texture.width}x{texture.height}.", context);
                return null;
            }

            if (shared == null && texture != null && !texture.sRGB && QualitySettings.activeColorSpace == ColorSpace.Linear)
                Debug.LogWarning($"Target texture '{texture.name}' of camera '{camera.name}' is not sRGB: in the linear " +
                                 "color space the published image will be darker than on screen. Clear the camera's " +
                                 "Target Texture (the publisher creates an sRGB one) or enable sRGB on the texture.", context);

            int bytesPerPixel;
            switch (format)
            {
                case TextureFormat.RGB24: bytesPerPixel = 3; break;
                case TextureFormat.RGBA32: bytesPerPixel = 4; break;
                default: throw new ArgumentException($"Unsupported readback format {format}", nameof(format));
            }

            return new CameraCapture(camera, width, height, format, bytesPerPixel, onFrame);
        }

        private CameraCapture(Camera camera, int width, int height, TextureFormat format, int bytesPerPixel,
            Action<NativeArray<byte>> onFrame)
        {
            _camera = camera;
            _format = format;
            _onFrame = onFrame;
            Width = width;
            Height = height;

            if (!Targets.TryGetValue(camera, out _target))
            {
                _target = new Target { cameraWasEnabled = camera.enabled };
                if (camera.targetTexture == null)
                {
                    _target.texture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
                    _target.texture.Create();
                    camera.targetTexture = _target.texture;
                    _target.ownsTexture = true;
                }
                else
                {
                    _target.texture = camera.targetTexture;
                }

                camera.enabled = false;
                Targets.Add(camera, _target);
            }
            _target.users++;

            _buffer = new NativeArray<byte>(width * height * bytesPerPixel, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);
        }

        /// <summary>Renders the camera and requests the readback. The frame comes to onFrame on the main thread.</summary>
        public bool Capture()
        {
            if (_disposed || _pending)
                return false;

            // Captures of one camera in the same frame read back one render.
            if (_target.renderedFrame != Time.frameCount)
            {
                _camera.Render();
                _target.renderedFrame = Time.frameCount;
            }

            _pending = true;
            AsyncGPUReadback.RequestIntoNativeArray(ref _buffer, _target.texture, 0, _format, OnReadback);

            // Keep the average rate, but when falling behind don't queue up several captures.
            double now = Time.timeAsDouble;
            _nextCapture += _period;
            if (_nextCapture < now)
                _nextCapture = now;
            return true;
        }

        private void OnReadback(AsyncGPUReadbackRequest request)
        {
            _pending = false;
            if (_disposed)
            {
                // Can arrive after the owner was disabled (e.g. after leaving play mode): only free the buffer.
                _buffer.Dispose();
                return;
            }

            if (!request.hasError)
                _onFrame(_buffer);
        }

        public void Dispose()
        {
            if (_disposed)
                return;
            _disposed = true;

            if (--_target.users == 0)
                ReleaseTarget();

            // A pending readback still writes into the buffer; it is freed in OnReadback.
            if (!_pending)
                _buffer.Dispose();
        }

        private void ReleaseTarget()
        {
            Targets.Remove(_camera);

            // The camera may already be destroyed (scene unload); Unity's == null covers that.
            if (_camera != null)
            {
                _camera.enabled = _target.cameraWasEnabled;
                if (_target.ownsTexture && _camera.targetTexture == _target.texture)
                    _camera.targetTexture = null;
            }

            var texture = _target.texture;
            if (_target.ownsTexture && texture != null)
            {
                texture.Release();
                if (Application.isPlaying)
                    UnityEngine.Object.Destroy(texture);
                else
                    UnityEngine.Object.DestroyImmediate(texture);
            }
        }
    }
}
