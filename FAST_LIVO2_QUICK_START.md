# FAST-LIVO2 Quick Start

> For the current Android desktop-mapping stream and bridge, follow the
> [mobile setup](mobile/README.md#mode-2-desktop-fast-livo2-android-experimental).
> The launch commands below describe a different, older bridge protocol.

## One-Minute Setup

You have two ROS 2 workspaces:
1. **lidarws** (~dev/lidarws) — Mobile bridge + topic adapter
2. **ros2_fast_livo_ws** (~dev/ros2_fast_livo_ws) — FAST-LIVO2 SLAM system

### Step 1: Build Both Workspaces

```bash
# Build mobile bridge workspace
cd ~/dev/lidarws
colcon build --packages-select lidar_ros2_bridge lvi_sam_mobile

# Build FAST-LIVO2 workspace
cd ~/dev/ros2_fast_livo_ws
colcon build --packages-select fast_livo
```

### Step 2: Launch in Two Terminals

**Terminal 1: Mobile Bridge + Topic Adapter**
```bash
cd ~/dev/lidarws && source install/setup.bash
ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py mobile_ip:=192.168.1.100
```
(Replace `192.168.1.100` with your phone's IP)

**Terminal 2: FAST-LIVO2 Mapping**
```bash
cd ~/dev/ros2_fast_livo_ws && source install/setup.bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True
```

### Step 3: Verify Topics

```bash
# In a third terminal, check topics are flowing
ros2 topic list | grep -E "livox|camera"

# Expected output:
#   /livox/lidar      (LiDAR point cloud)
#   /livox/imu        (IMU acceleration + angular velocity)
#   /left_camera/image (Camera frames)
```

## Data Flow

```
Your Mobile Scanner (UDP)
         ↓
[Terminal 1] Mobile Bridge (UDP receivers on ports 5000, 5001)
         ↓
[Terminal 1] Topic Adapter (republishes to /livox/* topics)
         ↓
[Terminal 2] FAST-LIVO2 (subscribes to /livox/*)
         ↓
RViz2: 3D point clouds + trajectory
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Launch fails with "libexec directory does not exist" | This is already fixed. Rebuild: `colcon build --packages-select lidar_ros2_bridge` |
| No topics appearing | Check mobile device IP: `ping 192.168.1.100` |
| LiDAR data not flowing | Check UDP ports: `netstat -un \| grep 500[01]` |
| Poor SLAM results | Tune `~/dev/ros2_fast_livo_ws/src/FAST-LIVO2/config/avia.yaml` (see FAST_LIVO2_INTEGRATION.md) |

## Next Steps

- See **FAST_LIVO2_INTEGRATION.md** for detailed configuration
- See **FAST_LIVO2_TOPIC_MAP.md** for message type specifications
- Calibrate sensor extrinsics using FAST-Calib for best results
