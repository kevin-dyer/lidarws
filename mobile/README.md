# 3D Lidar Scanner mobile host

Cross-platform React Native app for iOS and Android. It replaces the ROS/PC
host during capture: the phone discovers the Nano ESP32 over UDP, streams
RPLIDAR and tilt data directly, turns each scan ray into an XYZ point, and
saves a standard ASCII `.ply` point cloud in the app's document storage.
The existing ROS post-processing and mesh-generation workflow remains
available for desktop use; mesh reconstruction and multi-scan registration
are intentionally not performed on the phone.

## Development build

This app uses native UDP sockets and Wi-Fi network joining, so it needs an
Expo development build rather than Expo Go.

Requires Node.js 20.19 or newer (Node 22 recommended; see `.nvmrc`). Ubuntu's
`apt` `nodejs` package is too old and fails with `SyntaxError: Unexpected
token '?'`. Install a current version, for example with nvm:

```sh
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
# restart the shell, then:
cd mobile
nvm install   # reads .nvmrc
node --version
```

Then:

```sh
npm install
npx expo prebuild
npx expo run:ios
# or
npx expo run:android
```

Use macOS with Xcode to build/run iOS. Android Studio and an Android SDK are
needed for Android. The app requests local network access, camera access for
optional scan photos, and Wi-Fi permission when joining the scanner access
point. Native development builds are required on both platforms.

## Scanner network setup

1. Compile/upload `firmware/tilt_stage/tilt_stage.ino` to the Nano ESP32.
   Create `firmware/tilt_stage/wifi_config.h` from the example for the initial
   home network and fallback access-point credentials.
2. On a home network, connect the phone to the same Wi-Fi as the scanner and
   use **Discover nearby scanner**.
3. Away from home, join the fallback AP from **Scanner setup**. Enter its SSID
   and password there; the fallback remains enabled when home Wi-Fi is saved.
4. To provision a home network, enter its SSID and password in the app and
   choose **Save to scanner**. The ESP32 stores these values in NVS and tries
   them on startup. The phone does not persist the Wi-Fi password.

The firmware and app use the bridge's CRC-16 UDP frames on board port `4210`
and phone port `4211`. The app uses the existing tilt text commands and
RPLIDAR A1 standard scan stream. Keep this local, untrusted networks are not
appropriate for the bridge's unauthenticated LiDAR/control protocol.

## Capture and files

The angle selector defaults to −45° and +45°. The duration slider ranges from
5 to 120 seconds and sets tilt speed proportionally. The stage firmware
limits scanning to 10°/s; durations shorter than the selected angular span
allows are raised to the physically achievable minimum. At the default 90°
span this minimum is 9 seconds.

Opening the Start Scan page homes the tilt stage. Dragging a start or stop
angle moves the stage to that angle so it can be checked by eye; after 3
seconds without a change it returns home. A name, photo and group (existing or
new) can be set before scanning and edited afterwards. **Advanced options**
crop the saved cloud by minimum distance, maximum distance and horizontal
sweep (centred on the front; 180° equals *front half only*, and any other value
overrides that checkbox). If the scanner stops responding during a scan the app
shows an error and returns to Start Scan.

After capture, the user can optionally take a reference photo, save the point
cloud, or discard it. Saved `.ply` files can be shared using the iOS or Android
system share sheet (including AirDrop or email). The app keeps scans and
optional photos in its private document directory. For final meshing or
registration, share the point cloud to a desktop post-processing pipeline or
cloud service.

The mobile point conversion follows `tilt_coordinator/tilt_math.py`: LiDAR
azimuth is offset by 90° and the tilt stage rotates the ray about the Y axis.

## Mobile mapping (hand-held LiDAR-inertial mapping)

Two modes:

### Mode 1: Local (LiDAR + Inertial only) — on-device
Default mode. Attach the phone to the front of the scanner (rear camera facing forward), choose **Mobile mapping**, configure sweep period/tilt/range/orientation, and press **Start mapping**. Gyro + accelerometer provide orientation; LiDAR provides pose via scan-to-map ICP. No desktop needed.

- Hold still for the first sweep to seed the map, then move slowly.
- If **Tracking lost** appears, the app pauses adding scans to the map and
  continues trying to recover. When there are too few matches, it gathers a
  longer batch before retrying; hold the rig still while it retries. Longer
  batches reduce how often pose updates arrive, so move slowly after recovery.
  The banner shows the latest match count and RMSE for persistent loss.
- Video is recorded with the trajectory for offline post-processing.
- Saved: `*_raw_cloud.ply`, `*_video.mp4`, `*_trajectory.json`.

### Mode 2: Desktop FAST-LIVO2 (Android, experimental)

This mode streams phone IMU, camera JPEG frames, and tilted RPLIDAR points to
the desktop. The phone still shows its **local** map; FAST-LIVO2's result is
shown in desktop RViz, not returned to the phone. Unlike local mode, desktop
mode does not record video. The camera uses repeated still captures; its frame
rate is device-dependent and may be too low for reliable visual odometry.

1. Recompile and flash `firmware/tilt_stage/tilt_stage.ino` to the Nano ESP32,
   then rebuild the Android app (`npx expo run:android`). Both updates are
   needed for the timed LiDAR protocol. A native rebuild and installation
   (not just a JavaScript reload) is also required to include the Android
   high-sampling-rate sensor permission.
2. Connect the scanner, Android phone, and desktop on a reachable private
   network. Obtain the desktop's address with `hostname -I`. This protocol
   has no authentication; do not expose UDP port 5005 to untrusted networks.
3. Use ROS 2 Humble with Python 3.10, OpenCV, NumPy and `cv_bridge`. Stop the
   old `mobile_data_bridge.py` if it is using port 5005. In a desktop
   terminal, start the new bridge:

   ```sh
   source /opt/ros/humble/setup.bash
   cd ~/dev/lidarws
   /usr/bin/python3.10 src/lidar_ros2_bridge/lidar_ros2_bridge/fast_livo_desktop.py
   ```
   Use the same `ROS_DOMAIN_ID` as FAST-LIVO2. The bridge logs the actual
   camera-frame dimensions when the first image arrives.

4. In the app, connect the scanner, select **Mobile mapping** → **Desktop
   FAST-LIVO2**, enter the desktop IPv4 address, choose the actual phone mount
   orientation, and start mapping. The app requires a bridge response and
   reports missing timed-firmware support, lost LiDAR packets, or a lost
   bridge heartbeat. Brief LiDAR send delays are buffered in order; if the
   bounded queue fills, the app stops and reports that it cannot keep up.
   The phone also tolerates brief reordering of scanner packets, but stops
   with sequence details if a timed packet remains missing.
5. In another ROS-sourced terminal, check `ros2 topic hz /livox/imu`,
   `ros2 topic hz /livox/lidar`, and `ros2 topic hz /left_camera/image`.
   The phone requests IMU updates every 20 ms, although the actual rate
   depends on the device and workload. If the IMU stream pauses for more
   than 0.2 seconds, this FAST-LIVO2 version rejects subsequent IMU samples;
   restart the FAST-LIVO2 launch before starting another mapping run.
   The LiDAR topic must be `sensor_msgs/msg/PointCloud2`; check with
   `ros2 topic info -v /livox/lidar`. The bridge provides the `t` field in
   nanoseconds and the other fields expected by FAST-LIVO2's Ouster decoder.
   The old synthetic-data bridge's XYZ-only `PointCloud2` does not provide
   this layout and is not interchangeable with this bridge.

Before attempting FAST-LIVO2 mapping, provide calibration for your phone and
rig. The default Avia camera and extrinsics are for different hardware. Copy
the FAST-LIVO2 `avia.yaml` and `camera_pinhole.yaml` to your own files and
configure the actual camera intrinsics, distortion and streamed image size;
camera-to-LiDAR and IMU-to-LiDAR extrinsics; and measured time offsets. The
bridge rotates IMU vectors into the scanner FLU axes, so the IMU rotation in
the configuration must be consistent with that choice. Set
`preprocess.lidar_type: 3` to select FAST-LIVO2's Ouster-format `PointCloud2`
decoder (this is only a data format; the scanner is still an RPLIDAR A1),
`preprocess.scan_line: 1` for this single-line scanner, and start with
`time_offset.img_time_offset: 0.0` until the camera delay is measured. The
tilt projection assumes the LiDAR origin is at the tilt pivot; account for
any physical offset before relying on the map.

Launch FAST-LIVO2 in a separate terminal with your calibrated files:

```sh
source /opt/ros/humble/setup.bash
source ~/dev/ros2_fast_livo_ws/install/setup.bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True \
  avia_params_file:=/absolute/path/to/your_lidar_config.yaml \
  camera_params_file:=/absolute/path/to/your_camera_config.yaml
```

The A1 standard measurement has no per-point sensor timestamp. The ESP32
timestamps UART reads and estimates each byte's arrival using the 115200-baud
serial duration. In timed mode, the firmware waits up to 5 ms to group small
UART reads into fewer UDP packets; this may add timing uncertainty. The app
estimates measurement time from the first byte,
synchronizes the ESP32 and phone clocks with a round-trip exchange, and uses
timestamped tilt samples. UART buffering, Wi-Fi and scheduling introduce
uncertainty; clock drift is not corrected during a run. IMU timestamps use the
phone sensor clock aligned to phone time at the first sample; camera timestamps
are capture-request times, not exposure times. Validate timing and calibration
on the actual device before trusting any resulting trajectory.

### Experimental Kalibr bag from the existing desktop stream

This is a diagnostic export, **not** a validated calibration capture. The
current app sends repeated JPEG stills with request-time timestamps, unlocked
camera controls, and phone IMU values rotated into scanner axes. Low frame
rate, variable camera latency, UDP loss and the IMU-to-phone axis rotation
can make the resulting calibration inaccurate. Do not treat its results as
calibration of the phone's native IMU frame.

With the bridge running and desktop mapping active, record on the desktop:

```sh
source /opt/ros/humble/setup.bash
ros2 bag record -o kalibr_ros2 /left_camera/image /livox/imu
```

Stop recording with Ctrl+C. Convert the resulting ROS 2 bag directory to a
ROS 1 bag using the `rosbags-convert` command from the `rosbags` Python
package (for example, in a Python virtual environment):

```sh
python3 -m venv ~/kalibr-bag-tools
~/kalibr-bag-tools/bin/python -m pip install rosbags
~/kalibr-bag-tools/bin/rosbags-convert --src kalibr_ros2 --dst imu_april.bag \
  --include-topic /left_camera/image /livox/imu
```

The Kalibr camera-chain `rostopic` must be `/left_camera/image` and the IMU
configuration `rostopic` must be `/livox/imu`. Supply a measured AprilGrid
configuration, camera intrinsics calibrated for these exact image settings,
and measured phone IMU noise parameters. The ADIS example IMU parameters are
not valid for the phone. Camera–IMU calibration does not determine the
LiDAR-to-phone transform; that requires a separate rig calibration.
