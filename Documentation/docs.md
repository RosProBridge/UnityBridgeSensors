# ProBridge Sensors Documentation

**Note:** Before proceeding, ensure you have reviewed the [ProBridge documentation](https://github.com/RosProBridge/UnityBridge/blob/master/Documentation/docs.md). This document assumes you are familiar with ProBridge and provides sensor-specific information.

<details>
  <summary><strong>Table of Contents</strong></summary>

1. [CompressedImage](#compressedimage)
2. [CameraInfo](#camerainfo)
3. [Imu](#imu)
4. [NavSatFix](#navsatfix)
5. [RayCast Lidar](#raycast-lidar)
   - [Adding Scan Patterns](#adding-scan-patterns)
      - [By CSV](#by-csv)
      - [Manually](#manually)
      - [Prebuilt Patterns](#prebuilt-patterns)

</details>

## CompressedImage

The `CompressedImage` publisher sends `sensor_msgs.msg.CompressedImage` messages. The following fields are available:

- **Format:**
  - **JPEG**: The fastest option with support for adjustable compression quality.
  - **PNG**: Slower than JPEG but offers higher quality.
- **Render Camera:**
  - This is where you reference the camera whose output will be published.
  - **Note**: The camera used here should not be the scene's main camera, as it will not render to the screen when referenced.
- **Texture Width:**
  - Specifies the width of the output image.
- **Texture Height:**
  - Specifies the height of the output image.
- **Compression Quality:**
  - A slider (0–100) where 0 is the fastest (lowest quality) and 100 is the slowest (highest quality).
  - **Note**: This option is applicable only for the JPEG format.

For the `CompressedImage` output to be usable, it typically requires a `CameraInfo` publisher with the same `frame_id`. Follow this naming convention:
- **CompressedImage:** `/<CameraName>/compressed`
- **CameraInfo:** `/<CameraName>/camera_info`

## CameraInfo

The `CameraInfo` publisher sends `sensor_msgs.msg.CameraInfo` messages. It has the following field:

- **Camera:** The camera for which the information will be sent. This is usually the same camera connected to the `CompressedImage` publisher.

## Imu

The `Imu` publisher sends `sensor_msgs.msg.Imu` messages. It requires the GameObject to have a `Rigidbody` component.

## NavSatFix

The `NavSatFix` publisher sends `sensor_msgs.msg.NavSatFix` messages. It includes the following field:

- **Start LLA:** Represents the initial latitude, longitude, and altitude (LLA) in `Vector3` format.

## RayCast Lidar

The `RayCast Lidar` publisher sends `sensor_msgs.msg.PointCloud2` messages and has the following fields:

- **Scan Pattern:** Select a scriptable object asset that defines the scan pattern. See the section below on creating or obtaining scan patterns.
- **Points Num Per Scan:** The maximum number of points in each scan.
  - If the number of points in the scan pattern exceeds `Points Num Per Scan`, the scan will be divided and sent in smaller parts, each containing at most this number.
  - If the number of points is less than `Points Num Per Scan`, the full scan is sent in a single message.
- **Min Range:** The minimum detection range.
- **Max Range:** The maximum detection range.
- **Gaussian Noise Sigma:** The standard deviation for Gaussian noise, controlling the noise spread.
- **Max Intensity:** The highest possible intensity value for a point in the LiDAR sensor data.
- **Down Sample Scale:** The percentage by which the number of points in the scan is reduced.
  - `0` means no downsampling; all points are scanned and sent.
  - `0.99` means only 1% of the points are scanned and sent.

### Adding Scan Patterns

To add scan patterns, go to the Scan Pattern menu via **ProBridge > Sensors > Add Scan Pattern**.

Scan patterns can be added in three ways: from a CSV file, manually, or by downloading prebuilt patterns.

#### By CSV
1. Go to the CSV tab in the Scan Pattern Menu.
2. Select the CSV file (for reference on the CSV format, see [this file](https://raw.githubusercontent.com/RosProBridge/SensorFiles/refs/heads/main/ScanPatterns/RawData/LivoxScanPattern/avia.csv)).
3. Set the desired zenith angle offset (if needed).
4. Click **Generate** and wait for completion.
5. The generated scan pattern will be saved to `Assets/ScanPatterns`.

#### Manually
1. Go to the Manual tab in the Scan Pattern Menu.
2. Enter the required details for the scan pattern.
3. Click **Generate** and wait for completion.
4. The generated scan pattern will be saved to `Assets/ScanPatterns`.

#### Prebuilt Patterns
1. Go to the Prebuilt tab in the Scan Pattern Menu.
2. Browse for the desired pattern.
3. Click **Download** next to the pattern and wait for the process to complete.
4. The downloaded scan pattern will be saved to `Assets/ScanPatterns`.
