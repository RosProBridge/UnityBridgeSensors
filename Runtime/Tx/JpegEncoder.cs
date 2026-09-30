// TurboJpeg поставляется с нативными библиотеками только для Windows и Linux.
// В редакторе на macOS сборка обёртки компилируется, но нативной библиотеки нет.
#if (UNITY_STANDALONE_WIN || UNITY_STANDALONE_LINUX) && !UNITY_EDITOR_OSX
#define PROBRIDGE_TURBOJPEG
#endif

using System;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
#if PROBRIDGE_TURBOJPEG
using TurboJpegWrapper;
#endif

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// Сжимает кадр RGBA32 в JPEG. Строки кадра идут снизу вверх, как в данных текстуры и AsyncGPUReadback.
    /// На Windows и Linux используется TurboJpeg, на остальных платформах — ImageConversion (медленнее).
    /// Экземпляр не потокобезопасен: используйте отдельный кодировщик на поток.
    /// </summary>
    public sealed class JpegEncoder : IDisposable
    {
#if PROBRIDGE_TURBOJPEG
        public const bool UsesTurboJpeg = true;
        private TJCompressor _compressor = new TJCompressor();
#else
        public const bool UsesTurboJpeg = false;
        private byte[] _grayBuffer;
#endif

        /// <param name="rgba">Пиксели RGBA32, width * height * 4 байт.</param>
        /// <param name="quality">Качество JPEG, 1..100.</param>
        /// <param name="grayscale">Сохранить одноканальное изображение по каналу R.</param>
        public byte[] Encode(byte[] rgba, int width, int height, int quality, bool grayscale = false)
        {
#if PROBRIDGE_TURBOJPEG
            return _compressor.Compress(rgba, 0, width, height,
                TJPixelFormats.TJPF_RGBA,
                grayscale ? TJSubsamplingOptions.TJSAMP_GRAY : TJSubsamplingOptions.TJSAMP_444,
                quality,
                TJFlags.FASTDCT | TJFlags.BOTTOMUP);
#else
            if (!grayscale)
                return ImageConversion.EncodeArrayToJPG(rgba, GraphicsFormat.R8G8B8A8_UNorm,
                    (uint)width, (uint)height, 0, quality);

            int pixels = width * height;
            if (_grayBuffer == null || _grayBuffer.Length != pixels)
                _grayBuffer = new byte[pixels];
            for (int i = 0; i < pixels; i++)
                _grayBuffer[i] = rgba[i * 4];

            return ImageConversion.EncodeArrayToJPG(_grayBuffer, GraphicsFormat.R8_UNorm,
                (uint)width, (uint)height, 0, quality);
#endif
        }

        public void Dispose()
        {
#if PROBRIDGE_TURBOJPEG
            _compressor?.Dispose();
            _compressor = null;
#endif
        }
    }
}
