# Fast LIVO2 Integration with Mobile Scanner

> The mobile desktop integration described below is outdated. For the current
> Android FAST-LIVO2 bridge, timed sensor protocol, and setup, see
> [Mobile mapping](mobile/README.md#mode-2-desktop-fast-livo2-android-experimental).

This guide shows how to stream data from your 3D LiDAR scanner + mobile app into FAST-LIVO2 for simultaneous localization and mapping.

## Architecture

```
Mobile Device (Scanner + Phone)
  ├─ LiDAR frames → UDP:5000 (/lidar/points)
  ├─ IMU data → UDP:5001 (/imu/data)
  └─ Camera RTSP → rtsp://IP:8554 (/camera/image_raw)
        ↓
   [Mobile Bridge]
        ↓
   [Topic Adapter] (republishes to Livox topics)
        ↓
   [FAST-LIVO2]
   ├─ LiDAR: /livox/lidar
   ├─ IMU: /livox/imu
   └─ Camera: /left_camera/image
```

## Setup

### 1. Ensure both workspaces are built

**lidarws** (mobile bridge + adapter):
```bash
cd ~/dev/lidarws
colcon build --packages-select lidar_ros2_bridge lvi_sam_mobile
source install/setup.bash
```

**ros2_fast_livo_ws** (FAST-LIVO2):
```bash
cd ~/dev/ros2_fast_livo_ws
colcon build --packages-select fast_livo
source install/setup.bash
```

### 2. Source both workspaces

```bash
# In your shell (or add to ~/.bashrc)
source /opt/ros/humble/setup.bash
source ~/dev/lidarws/install/setup.bash
source ~/dev/ros2_fast_livo_ws/install/setup.bash
```

## Usage

### Recommended: Launch mobile bridge + adapter in lidarws

```bash
# Terminal 1: Start mobile bridge + topic adapter
cd ~/dev/lidarws && source install/setup.bash
ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py mobile_ip:=192.168.1.100
```

This starts:
- UDP LiDAR receiver (port 5000)
- UDP IMU receiver (port 5001)
- Topic adapter (republishes to Livox topics)

### Then: Launch FAST-LIVO2 in separate workspace

```bash
# Terminal 2: Source FAST-LIVO2 workspace and launch
cd ~/dev/ros2_fast_livo_ws && source install/setup.bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True
```

Both terminals can run independently. FAST-LIVO2 subscribes to the Livox topics that the adapter is publishing.

### Optional: Launch camera bridge separately

If you want RTSP camera integration, you may need to launch it in a compatible Python environment due to version mismatches. This is optional for pure LiDAR-IMU SLAM:

```bash
# In a terminal with compatible Python (system Python, not miniforge):
cd ~/dev/lidarws && source install/setup.bash
python3 -m lvi_sam_mobile.camera_bridge
```

## Usage (Alternative: Step-by-Step)

If you prefer finer control:

```bash
# Terminal 1: UDP LiDAR receiver
ros2 run lidar_ros2_bridge udp_lidar_receiver

# Terminal 2: UDP IMU receiver
ros2 run lidar_ros2_bridge udp_imu_receiver

# Terminal 3: Topic adapter
ros2 run lidar_ros2_bridge fast_livo_adapter

# Terminal 4: FAST-LIVO2 (from ros2_fast_livo_ws workspace)
source ~/dev/ros2_fast_livo_ws/install/setup.bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True
```

## Verify Data Flow

Check that all topics are being published:

```bash
# List topics
ros2 topic list

# Should show:
#   /livox/lidar
#   /livox/imu
#   /left_camera/image
#   ... (other topics)

# Verify LiDAR data rate
ros2 topic hz /livox/lidar

# Verify IMU data rate (should be ~20 Hz)
ros2 topic hz /livox/imu

# Verify camera frame rate
ros2 topic hz /left_camera/image
```

## Troubleshooting

### FAST-LIVO2 complains about missing topics

Check topic names with `ros2 topic list`. If you see `/lidar/points` but not `/livox/lidar`, ensure the adapter is running:
```bash
ros2 run lidar_ros2_bridge fast_livo_adapter
```

### No LiDAR data arriving

1. Verify mobile app is connected and sending UDP packets:
   ```bash
   ros2 topic hz /lidar/points
   ```
   Should show >0 Hz if data is flowing.

2. Check UDP port is not blocked:
   ```bash
   netstat -un | grep 5000  # LiDAR port
   netstat -un | grep 5001  # IMU port
   ```

3. Verify mobile device IP is correct and reachable:
   ```bash
   ping 192.168.1.100  # Replace with your phone's IP
   ```

### No camera frames

1. Verify RTSP stream is accessible:
   ```bash
   gst-launch-1.0 rtspsrc location="rtsp://192.168.1.100:8554/stream" ! fakesink
   ```

2. Check camera topic is publishing:
   ```bash
   ros2 topic hz /camera/image_raw
   ```

3. Ensure your phone IP is correct in the launch command.

### FAST-LIVO2 runs but produces poor results

Check the `avia.yaml` config:
- `extrinsic_T` and `extrinsic_R`: These calibrate the camera-to-LiDAR offset. Adjust if your setup is non-standard.
- `img_time_offset`: Camera exposure time (default 0.1). Adjust if images are blurry.
- `imu_time_offset`: IMU timing offset (default 0.0). Adjust if visual-inertial tracking diverges.
- `scan_line`: For RPLIDAR, usually set to the actual number of laser lines.

For RPLIDAR (typically 8-16 lines):
```yaml
preprocess:
  scan_line: 8  # Adjust based on your scanner
```

## Data Format Details

### LiDAR (/livox/lidar)
- Type: `sensor_msgs/PointCloud2`
- Frame: Mobile bridge publishes to `/lidar/points` with frame_id `lidar`
- Adapter republishes to `/livox/lidar` (same data, different topic)

### IMU (/livox/imu)
- Type: `sensor_msgs/Imu`
- Rate: ~20 Hz from phone accelerometer/gyroscope
- Covariance: Set by UDP receiver (tunable in `udp_imu_receiver.py`)

### Camera (/left_camera/image)
- Type: `sensor_msgs/Image`
- Encoding: RGB8 (converted from RTSP BGR)
- Rate: Depends on phone camera (typically 20-30 Hz)

## Next Steps

1. **Calibrate sensor extrinsics**: Use [FAST-Calib](https://github.com/hku-mars/FAST-Calib) for accurate camera-LiDAR calibration
2. **Tune parameters**: Edit `~/dev/ros2_fast_livo_ws/src/FAST-LIVO2/config/avia.yaml` for your environment
3. **Export results**: FAST-LIVO2 can output dense point clouds and trajectories (see RViz output)
4. **Record data**: Use `ros2 bag record` to capture runs for offline processing

## Files Modified

- `lidar_ros2_bridge/fast_livo_adapter.py` — New topic adapter
- `lidar_ros2_bridge/launch/fast_livo_mobile.launch.py` — New launch file
- `lidar_ros2_bridge/setup.py` — Updated entry points and data files

## References

- [FAST-LIVO2 Repository](https://github.com/hku-mars/FAST-LIVO2)
- [Sensor Message Types](http://docs.ros.org/en/humble/api/sensor_msgs/index.html)
- [ROS 2 Remapping](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html#remapping-arguments)
