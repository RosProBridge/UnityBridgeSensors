using System;
using UnityEngine;

namespace ProBridge.Tx.Sensor
{
    /// <summary>
    /// A camera image publisher (CompressedImageTx, ImageTx) that <see cref="CameraInfoTx"/> can follow:
    /// camera_info is then sent for every published frame, with its stamp, frame_id and resolution.
    /// </summary>
    public interface ICameraImageSource
    {
        Camera ImageCamera { get; }
        int ImageWidth { get; }
        int ImageHeight { get; }
        string ImageFrameId { get; }

        /// <summary>A frame with this stamp is being published (main thread).</summary>
        event Action<TimeSpan> FramePublished;
    }
}
