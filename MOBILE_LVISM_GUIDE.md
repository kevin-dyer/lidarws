# LVI-SAM Mobile Mapping with ros2-lvi

> This is a legacy guide and does not describe the current Android desktop
> mapping implementation. See [mobile setup](mobile/README.md#mode-2-desktop-fast-livo2-android-experimental)
> for the experimental FAST-LIVO2 stream.

Full LiDAR-Visual-Inertial Simultaneous Localization and Mapping (LVI-SAM) pipeline for your mobile scanning system.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│ MOBILE DEVICE (Phone)                                           │
├─────────────────────────────────────────────────────────────────┤
│ • LiDAR frames @ 10 Hz (from scanner)  ──► UDP 5000             │
│ • IMU (accel + gyro) @ 20 Hz            ──► UDP 5001             │
│ • Camera video (30 fps)                 ──► RTSP server :8554    │
│ • Tilt angle @ 10 Hz                    ──► UDP (in LiDAR)       │
└─────────────────────────────────────────────────────────────────┘
              │
              │ WiFi (home network)
              ▼
┌─────────────────────────────────────────────────────────────────┐
│ DESKTOP (Ubuntu 22.04 + ROS 2 Humble)                           │
├─────────────────────────────────────────────────────────────────┤
│ 1. lidar_ros2_bridge (UDP receivers)                            │
│    • udp_lidar_receiver    → /lidar/points                      │
│    • udp_imu_receiver      → /imu/data                          │
│    • live_cloud_publisher  → /lidar/mapping/cloud (+ UDP 5002)  │
│                                                                 │
│ 2. lvi_sam_mobile (data prep)                                   │
│    • camera_bridge        → /camera/image_raw (from RTSP)       │
│    • tilt_sync            → /joint_states                       │
│                                                                 │
│ 3. lvi_sam (ros2-lvi backend)                                   │
│    • Subscribes: /lidar/points, /imu/data, /camera/image_raw    │
│    • Outputs:   /lvi_sam/cloud_registered, /lvi_sam/path        │
│                                                                 │
│ 4. Visualization                                                │
│    • desktop_viewer       → Open3D 3D view (live)               │
│    • rviz2                → ROS visualization                   │
└─────────────────────────────────────────────────────────────────┘
              │
              │ UDP 5002 (live cloud to phone)
              ▼
┌─────────────────────────────────────────────────────────────────┐
│ MOBILE APP (React Native)                                       │
├─────────────────────────────────────────────────────────────────┤
│ • Displays live cloud from desktop on phone screen              │
│ • Shows mapping status and trajectory in real-time              │
│ • Saves final point cloud + trajectory after mapping stops      │
└─────────────────────────────────────────────────────────────────┘
```

## Prerequisites

### Desktop (Ubuntu 22.04)
- **ROS 2 Humble** installed and sourced
- **Python 3.10+**
- **OpenCV** (`python3-opencv`)
- **NumPy** (`python3-numpy`)
- **Open3D** (for desktop visualization): `pip install open3d`
- **GStreamer** (for RTSP): `sudo apt install gstreamer1.0-tools libgstreamer1.0-dev`

### Mobile Device
- Android phone with camera and IMU
- Expo development client installed
- Attached to scanner (rear camera facing forward)
- Connected to same WiFi as desktop

### Hardware Setup
- ESP32 scanner with tilt stage (firmware flashed with oscillation command `W<min>,<max>,<period>`)
- Phone mounted securely on scanner front

## Installation

### 1. Build ROS 2 packages

```bash
cd ~/dev/lidarws
colcon build --packages-up-to lidar_ros2_bridge lvi_sam_mobile
source install/setup.bash
```

### 2. Install ros2-lvi

Follow the [ros2-lvi installation](https://github.com/ryinin/ros2-lvi):

```bash
mkdir -p ~/dev/ros2_lvi_ws/src
cd ~/dev/ros2_lvi_ws/src
git clone https://github.com/ryinin/ros2-lvi.git
cd ~/dev/ros2_lvi_ws
colcon build --packages-up-to lvi_sam
source install/setup.bash
```

Ensure `lvi_sam` package is built and sourced before launching.

### 3. Update mobile app

The phone app has been modified to stream:
- **IMU data** (accel + gyro @ 20 Hz) via UDP port 5001
- **Tilt angle** (in LiDAR UDP packet)
- **Camera feed** via RTSP server (or HTTP stream)

Rebuild the development client:
```bash
cd mobile
npx expo prebuild --clean
npm run ios  # or npm run android
```

New dependencies: `expo-camera` ~17.0.10, `expo-sensors` ~15.0.8.

## Configuration

### Network Setup

1. **Find your desktop IP** on the home network:
   ```bash
   hostname -I
   # e.g., 192.168.1.50
   ```

2. **Find your phone's IP**:
   - Open phone settings → About → Status
   - Note the IP address (e.g., 192.168.1.100)

3. **Configure launch file**:
   ```bash
   ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
     mobile_ip:=192.168.1.100 \
     desktop_ip:=192.168.1.50 \
     show_rviz:=true \
     show_viewer:=true
   ```

### LVI-SAM Tuning

Edit `src/lvi_sam_mobile/lvi_sam_mobile/config/lvism_mobile.yaml` to adjust:
- Camera calibration (intrinsics, distortion)
- Feature extraction method (ORB, SIFT, AKAZE)
- LiDAR range and point density
- Bundle adjustment parameters
- Loop closure settings

## Usage

### Step 1: Launch the desktop pipeline

```bash
# Terminal 1: ROS 2 bridge + LVI-SAM + visualization
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:=192.168.1.100 \
  show_rviz:=true \
  show_viewer:=true
```

This starts:
- UDP receivers for LiDAR, IMU, tilt
- LVI-SAM node processing the streams
- Desktop Open3D viewer (real-time 3D visualization)
- RViz2 (point cloud + trajectory)
- Live cloud publisher (sends downsampled cloud to phone via UDP 5002)

### Step 2: Start mobile mapping on the phone

1. Open the app on your phone → **Mobile mapping**
2. Set sweep period, tilt range, max range, phone orientation
3. Press **Start mapping**
4. **Hold the scanner still for 5 seconds** (the tilt stage sweeps up and down; the app shows "Hold still")
5. Slowly walk the scanner through the scene, pointing the camera forward

### Step 3: Monitor on desktop

**Open3D viewer** (left window):
- Real-time 3D point cloud
- Trajectory (red line)
- Updates at ~10 Hz

**RViz2** (middle/right window):
- Aligned point cloud
- Camera frustums
- IMU arrows
- Localization uncertainty

**Terminal output**:
- LVI-SAM tracking state (tracking/lost)
- Number of features tracked
- Pose covariance

### Step 4: Stop and save

On the phone:
- Press the red **Stop mapping** button
- Wait for "Finishing..." to complete
- The app displays the final point cloud and offers to save

On desktop:
- The final trajectory and registered cloud are printed to terminal
- Outputs remain visible in RViz2 and the Open3D viewer

## Output Files

On the **phone**, after saving:
- `{id}_raw_cloud.ply` — final registered point cloud (FRD frame: x forward, y right, z down)
- `{id}_video.mp4` — synchronized camera video
- `{id}_trajectory.json` — pose trajectory (timestamps, positions, orientations)

On the **desktop**, in ROS topics:
- `/lvi_sam/cloud_registered` — dense registered point cloud
- `/lvi_sam/path` — camera pose trajectory
- `/lvi_sam/key_frames` — keyframe poses

## Troubleshooting

### UDP receiver not getting data
1. Check phone IP: `adb shell netstat -i` (Android) or Settings (iOS)
2. Verify WiFi connection (both on same network)
3. Check firewall: `sudo ufw allow 5000:5002/udp`
4. Restart app and bridge

### Camera not appearing in `/camera/image_raw`
1. Ensure phone has RTSP server running (app should start it automatically)
2. Test RTSP stream: `ffplay rtsp://192.168.1.100:8554/stream`
3. Check `camera_bridge` logs in terminal

### LVI-SAM not tracking
1. Ensure enough visual features in scene (textured walls, not blank surfaces)
2. Move slowly (< 0.5 m/s) to allow feature matching
3. Keep camera pointed forward
4. Check LiDAR range isn't too short (set `maxRangeM: 12`)

### Desktop viewer not showing
1. Install Open3D: `pip install open3d`
2. Check viewer process: `ps aux | grep desktop_viewer`
3. Start manually: `ros2 run lidar_ros2_bridge desktop_viewer`

### Large latency / dropped frames
1. Reduce downsampling rate in `live_cloud_publisher`:
   ```bash
   ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
     ... downsample_rate:=5
   ```
2. Lower LVI-SAM update rate or reduce feature count
3. Move closer to desktop (reduce WiFi latency)

## Performance Notes

- **Latency**: ~500 ms from capture to phone display (UDP round-trip + processing)
- **Point cloud density**: depends on LiDAR scan rate and range
- **CPU usage**: LVI-SAM ~40–60% on modern desktop CPU
- **Memory**: ~500 MB–1 GB depending on map size
- **WiFi bandwidth**: ~2–5 Mbps for LiDAR + IMU + compressed video

## Next Steps

1. **Camera calibration**: Provide intrinsics and distortion to LVI-SAM for better visual tracking
2. **Loop closure**: Enable in config and test on larger spaces (e.g., full rooms)
3. **Post-processing**: Use the saved trajectory JSON to colorize the final cloud by camera frames
4. **Mobile visualization improvements**: Stream full resolution cloud for better detail

## References

- **ros2-lvi**: https://github.com/ryinin/ros2-lvi
- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **RPLIDAR A1**: https://www.slamtec.com/en/Lidar/A1
- **Tilt coordinator**: `src/tilt_coordinator/doc/udp_lidar_bridge.md`

---

**Questions?** Check app logs (`mobile/src/mapping/run.ts`) and desktop logs (`ros2 launch ... show logs`).
