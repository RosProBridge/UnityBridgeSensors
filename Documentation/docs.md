# ProBridge Sensors Documentation

**Note:** Before proceeding, ensure you have reviewed the [ProBridge documentation](https://github.com/RosProBridge/UnityBridge/blob/master/Documentation/docs.md). This document assumes you are familiar with ProBridge and provides sensor-specific information.

<details>
  <summary><strong>Table of Contents</strong></summary>

1. [CompressedImage](#compressedimage)
2. [Image](#image)
3. [CameraInfo](#camerainfo)
4. [Imu](#imu)
5. [NavSatFix](#navsatfix)
6. [RayCast Lidar](#raycast-lidar)
   - [Non-repeating Scan Pattern](#non-repeating-scan-pattern)
   - [Intensity](#intensity)
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
  - The camera is disabled while the publisher is enabled and rendered on demand, once per sent frame (`sendRate`), so it costs nothing between frames. A new frame is rendered only after the previous one has been read back and encoded.
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

## Image

The `Image` publisher sends the camera image uncompressed as `sensor_msgs.msg.Image` (`rgb8`, rows top-down). There is no encoding cost, but the traffic is large: 640x480 at 10 Hz is about 9.2 MB/s (74 Mbit/s), so use it on a gigabit local network and keep `Compression Level` at `0`.

- **Render Camera, Texture Width, Texture Height:** same as in [CompressedImage](#compressedimage); the camera is rendered on demand at `sendRate`.

Topic naming: `/<CameraName>/image_raw` with a `CameraInfo` publisher on `/<CameraName>/camera_info`.

## CameraInfo

The `CameraInfo` publisher sends `sensor_msgs.msg.CameraInfo` messages. It has the following field:

- **Camera:** The camera for which the information will be sent. This is usually the same camera connected to the `CompressedImage` publisher.

## Imu

The `Imu` publisher sends `sensor_msgs.msg.Imu` messages. It requires the GameObject to have a `Rigidbody` component.

Default QoS (ROS 2) of a new IMU component: `Dict`, `BEST_EFFORT`, `KEEP_LAST`, depth `1`, `VOLATILE`, liveliness `SYSTEM_DEFAULT`.

## NavSatFix

The `NavSatFix` publisher sends `sensor_msgs.msg.NavSatFix` messages (WGS84). The Unity scene is treated as a plane tangent to the ellipsoid at the origin: local `x` = east, `y` = up, `z` = north. The position is converted Unity → ECEF (EPSG:4978) → LLA (EPSG:4979).

- **Init Origin:** `Transform` that marks the origin of the local frame; its rotation sets the east/north axes. If empty, the position of the sensor at the first enable is used, with world axes.
- **Start Latitude / Longitude / Altitude:** LLA of the origin (degrees, meters above the ellipsoid).
- **Apply Noise / Noise Std Dev:** Gaussian noise added to latitude, longitude (degrees) and altitude (meters).

## RayCast Lidar

The `RayCast Lidar` publisher sends `sensor_msgs.msg.PointCloud2` messages (`x`, `y`, `z` and optionally `intensity`, all `float32`). Rays are cast with `RaycastCommand` in Burst jobs only when a message is sent: nothing is computed while the component is disabled or the host is disconnected (see `Use Without Connect` in the ProBridge docs).

Default QoS (ROS 2) of a new lidar component: `Dict`, `RELIABLE`, `KEEP_LAST`, depth `5`, `VOLATILE`, liveliness `SYSTEM_DEFAULT`.

Lidar params:

- **Scan Pattern:** Select a scriptable object asset that defines the scan pattern. See the section below on creating or obtaining scan patterns.
- **Min Range / Max Range:** The detection range, in meters.
- **Gaussian Noise Sigma:** The standard deviation of the range noise, in meters.
- **Min / Max Azimuth Angle:** Only pattern points within this azimuth range are used (`0..360` = full pattern).
- **Points Per Second:** Point rate as in the lidar datasheet (Livox Mid-70: `100000`, Mid-360: `200000`, Velodyne VLP-16: `300000`); points per scan = this × `Send Rate`, reduced in proportion to the `Min / Max Azimuth Angle` crop, so the angular density stays as in the datasheet (a VLP-16 cropped to -90..90° gives 14 400 points at 0.2°). `0` (default): use `Down Sample Scale`.
- **Down Sample Scale:** The share of the pattern points skipped in every scan, `0..0.99` (when `Points Per Second` is `0`). Default `0.9`: every scan casts 10% of the pattern. Note that pattern assets are long (Mid-70 ~444k points, Mid-360 ~900k), so this can give far more points than the real sensor.
- **Pattern Shift:** Which pattern points are taken in the next scan, see [Non-repeating Scan Pattern](#non-repeating-scan-pattern). Default `None`.

Intensity params (see [Intensity](#intensity)):

- **Include Intensity:** Adds the `intensity` field to the point cloud.
- **Max Intensity:** Intensity of a retroreflector at zero range. Default `255`.
- **Diffuse Max Intensity:** Intensity of a 100% diffuse surface hit straight on at zero range. Default `150` (Livox); use `100` for Velodyne.
- **Default Reflectivity:** Reflectivity of colliders without a `LidarReflectivity` component and without a material, `0..1`. Default `0.3`.
- **Range Falloff:** Intensity drop at max range: `0` = none (calibrated reflectivity, as Livox/Velodyne/Ouster report it), `1` = down to zero. Default `0.2`.

### Performance

A scan is cast in slices, one per physics step between two messages (5 slices for `Send Rate = 0.1` and a 0.02 s physics step), each from the lidar pose at its step, like a real scanning lidar (moving the sensor distorts the cloud the same way). Each slice is scheduled as Burst jobs chained after the previous one, and the chain is completed only when the scan is sent, so the rays are cast on worker threads while the frame goes on (also when several physics steps run in one frame). Packing into `PointCloud2` is a single Burst job; serialization, compression and the socket send of all publishers run on the ProBridge host sender thread. Profiler markers: `RaycastLiDAR.*`, `ProBridgeTx.GetMsg <Type>`, `ProBridge.Serialize`, and `ProBridge.BuildFrame` / `ProBridge.SocketSend` on the `ProBridge` sender threads.

The cost grows with the number of rays: prefer `Points Per Second` matching the real sensor.

### Non-repeating Scan Pattern

With `Down Sample Scale > 0` every scan uses only a part of the pattern. `Pattern Shift` selects which part:

- **Sequential:** every scan takes the next consecutive chunk of the pattern. Livox patterns are recorded in time order, so this plays the real rosette back: a single scan looks like the real sensor output for that period, and accumulated scans fill the whole field of view.
- **Interleaved:** every scan takes every N-th point with a phase shifted by one each scan. Each scan covers the full field of view sparsely; after N scans the whole pattern is covered.
- **None** (default): the same points every scan (static picture).

All modes cost the same: only the pattern index of each ray changes.

Thinning keeps whole columns: consecutive pattern points with the same azimuth (one firing of all rings of a rotating lidar, e.g. 16 for VLP-16; 1 for Livox) are detected from the pattern, so a VLP-16 keeps all 16 rings and loses only azimuth resolution. Use `None` or `Interleaved` for rotating lidars (one revolution per pattern) and `Sequential` for time-ordered Livox patterns.

### Intensity

```
diffuse:          intensity = DiffuseMaxIntensity * reflectivity * cos(incidence) * falloff(range)
retroreflective:  intensity = DiffuseMaxIntensity + (MaxIntensity - DiffuseMaxIntensity) * reflectivity * falloff(range)
falloff(range)  = 1 - RangeFalloff * (range - MinRange) / (MaxRange - MinRange)
```

`reflectivity` is the diffuse (Lambertian) reflectivity `0..1` at the lidar wavelength, the same unit lidar datasheets use ("range at 10% / 80% reflectivity"). It is resolved per collider, in this order:

1. **`LidarReflectivity` component** on the collider's object or any parent (**ProBridge > Sensors > Lidar Reflectivity**): `Reflectivity` (`0..1`) and `Retroreflective` for road signs, reflectors and license plates. Typical values: asphalt ~0.1, concrete ~0.3, vegetation ~0.5, white paint ~0.8.
2. **Material:** luminance of the linear albedo, `_BaseColor` (or `_Color`) multiplied by the average color of `_BaseMap` (or the main texture), clamped to `0.02..0.9`. Computed once per material on the GPU, so textures do not need to be readable.
3. **Terrain:** the average of the terrain layers' diffuse textures (one value for the whole terrain).
4. **Default Reflectivity** of the lidar.

Values are cached per collider; a collider hit for the first time gets `Default Reflectivity` in that scan and its own value from the next one. Changing any `LidarReflectivity` resets the cache.

> **Note:** visible albedo is only an approximation of the near-infrared reflectivity (e.g. vegetation is much brighter in NIR). Use `LidarReflectivity` where it matters.

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
