# LVI-SAM Mobile Mapping: Quick Reference

## One-Line Summary
**Full LiDAR-Visual-Inertial SLAM pipeline**: phone streams to desktop (ROS 2 Humble + ros2-lvi), with real-time 3D visualization on both devices.

## Quick Start (Assuming Setup Complete)

### Terminal 1: Launch Desktop Pipeline
```bash
source /opt/ros/humble/setup.bash ~/dev/ros2_lvi_ws/install/setup.bash
cd ~/dev/lidarws && source install/setup.bash
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:=192.168.1.100 show_rviz:=true show_viewer:=true
```

### Terminal 2: App on Phone
1. Open app → **Mobile mapping**
2. Set parameters (defaults OK):
   - Period: 5 s
   - Min/max: -20° / +20°
   - Range: 12 m
   - Orientation: portrait (or as mounted)
3. Press **Start mapping**
4. **Hold still 5 seconds** (tilt sweeping, map seeding)
5. Walk scanner through space, camera forward
6. Press red **Stop mapping**
7. Save point cloud

### Terminal 3: Monitor (Optional)
```bash
# Watch LiDAR points
ros2 topic echo /lidar/points --once | head -10

# Watch IMU
ros2 topic echo /imu/data --once

# Watch LVI-SAM output
ros2 topic echo /lvi_sam/cloud_registered --once | head -5

# List all nodes
ros2 node list

# Show active topics
ros2 topic list -v
```

## File Locations (Quick Lookup)

| Purpose | File |
|---------|------|
| Desktop UDP bridge | `src/lidar_ros2_bridge/` |
| LVI-SAM wrapper | `src/lvi_sam_mobile/` |
| Mobile UI component | `mobile/src/components/MobileMapping.tsx` |
| Mobile mapping engine | `mobile/src/mapping/run.ts` |
| Firmware oscillation | `firmware/tilt_stage/tilt_stage.ino` (command: `W<min>,<max>,<period>`) |
| Full guide | `MOBILE_LVISM_GUIDE.md` |
| Setup checklist | `SETUP_CHECKLIST.md` |

## Key Configuration Parameters

### Launch Arguments (`mobile_lidar_bridge.launch.py`)
```bash
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:=192.168.1.100 \           # Phone IP
  desktop_ip:=192.168.1.50 \           # Desktop IP
  lidar_port:=5000 \                   # UDP port for LiDAR
  imu_port:=5001 \                     # UDP port for IMU
  cloud_port:=5002 \                   # UDP port for live cloud (to phone)
  downsample_rate:=10 \                # Cloud downsampling (10 = every 10th point)
  show_rviz:=true \                    # Show RViz2
  show_viewer:=true                    # Show Open3D viewer
```

### LVI-SAM Config (`src/lvi_sam_mobile/lvi_sam_mobile/config/lvism_mobile.yaml`)
- `featureMaxCount`: 300 (lower for speed, higher for accuracy)
- `baMaxIterations`: 10 (bundle adjustment iterations)
- `mapLocalMapRadius`: 20.0 m (local map size)
- `loopClosureEnabled`: true (enable loop closure detection)

### Mobile App Config (`mobile/src/mapping/settings.ts`)
```typescript
DEFAULT_MAPPING_SETTINGS = {
  sweepPeriodSeconds: 5,      // Tilt sweep up-and-down period
  minAngleDeg: -20,            // Lowest tilt angle
  maxAngleDeg: 20,             // Highest tilt angle
  maxRangeM: 12,               // LiDAR max range cutoff
  phoneMount: 'portrait'       // Phone orientation: portrait, landscape-left, landscape-right
}
```

## Troubleshooting (TL;DR)

| Problem | Solution |
|---------|----------|
| **UDP port blocked** | `sudo ufw allow 5000:5002/udp` |
| **No LiDAR data** | Verify phone IP matches `mobile_ip:=...`; test ping |
| **Camera not appearing** | Check RTSP: `ffplay rtsp://192.168.1.100:8554/stream` |
| **LVI-SAM not tracking** | Move slower, ensure texture in scene, check feature count in logs |
| **High latency** | Reduce `downsample_rate` to 5; lower LVI-SAM feature count |
| **Desktop viewer frozen** | Verify Open3D installed: `pip show open3d`; restart bridge |
| **App crashes on start** | Check camera/motion permissions on phone; rebuild app |

## ROS 2 Topics Reference

### Inputs (from phone)
| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | 10 Hz | Phone scanner (UDP 5000) |
| `/imu/data` | `sensor_msgs/Imu` | 20 Hz | Phone accelerometer/gyro (UDP 5001) |
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Phone camera (RTSP) |
| `/tilt/angle` | `std_msgs/Float32` | 10 Hz | Tilt stage (in LiDAR UDP) |

### Outputs (from LVI-SAM)
| Topic | Type | Rate | Use |
|-------|------|------|-----|
| `/lvi_sam/cloud_registered` | `sensor_msgs/PointCloud2` | 10 Hz | Dense mapped point cloud |
| `/lvi_sam/path` | `nav_msgs/Path` | 10 Hz | Camera pose trajectory |
| `/lvi_sam/key_frames` | `sensor_msgs/PointCloud2[]` | (varying) | Keyframe point clouds |

### Republished (for phone/viz)
| Topic | Type | Rate | Destination |
|-------|------|------|-------------|
| `/lidar/mapping/cloud` | `sensor_msgs/PointCloud2` | 10 Hz | RViz2 + UDP 5002 to phone |
| `/lidar/mapping/trajectory` | `nav_msgs/Path` | 10 Hz | RViz2 + visualizers |

## Node Dependency Graph

```
udp_lidar_receiver ──┐
                     ├──► lvi_sam ──┬──► live_cloud_publisher ──► UDP to phone
udp_imu_receiver ────┤              │
                     ├──► camera_bridge
camera_bridge ───────┤
                     ├──► tilt_sync
tilt_sync ───────────┘
                     └──► /lvi_sam/cloud_registered, /lvi_sam/path
                          ├──► rviz2
                          ├──► desktop_viewer (Open3D)
                          └──► live_cloud_publisher
```

## Code Entry Points

- **Mobile mapping UI**: `mobile/src/components/MobileMapping.tsx` (line 1, main component)
- **Mobile mapping engine**: `mobile/src/mapping/run.ts` (class `MappingRun`, line 42)
- **UDP bridge orchestration**: `src/lidar_ros2_bridge/lidar_ros2_bridge/launch/mobile_lidar_bridge.launch.py`
- **LVI-SAM backend**: `src/lvi_sam_mobile/lvi_sam_mobile/launch/mobile_lvism.launch.py`
- **Desktop visualization**: `src/lidar_ros2_bridge/lidar_ros2_bridge/desktop_viewer.py` (class `DesktopViewer`)

## Performance Targets

| Metric | Target | Typical |
|--------|--------|---------|
| LiDAR rate | 10 Hz | 10 Hz (RPLIDAR A1) |
| IMU rate | 20 Hz | 20 Hz (phone) |
| Camera rate | 30 fps | 30 fps (configurable) |
| LVI-SAM update | 10 Hz | 8–12 Hz (depends on scene) |
| Cloud to phone latency | < 1 s | ~500 ms (UDP RTT) |
| Point cloud density | varies | ~500–2000 pts/scan |
| Memory (desktop) | < 2 GB | ~500 MB–1 GB active |

## Common Commands

### Build Everything
```bash
cd ~/dev/lidarws
colcon build --packages-up-to lidar_ros2_bridge lvi_sam_mobile
source install/setup.bash
```

### Build Only Bridge
```bash
colcon build --packages-select lidar_ros2_bridge
```

### Clean Build
```bash
rm -rf build install log
colcon build --packages-up-to lidar_ros2_bridge lvi_sam_mobile
```

### Test Mobile App Types
```bash
cd mobile && npx tsc --noEmit
```

### Test Mobile App Unit Tests
```bash
cd mobile && npx jest
```

### View ros2-lvi Source (if installed)
```bash
cat ~/dev/ros2_lvi_ws/src/ros2-lvi/lvi_sam/src/lvi_sam.cpp | head -50
```

### Monitor All LVI-SAM Messages
```bash
ros2 topic list | grep lvi_sam | xargs -I {} ros2 topic hz {}
```

## Useful Debugging Exports

```bash
# Enable ROS 2 detailed logging
export RCL_LOGGING_LEVEL_rcl:=DEBUG
export RCL_LOGGING_LEVEL_rclcpp:=DEBUG

# Verify node communication
ros2 run rqt_graph rqt_graph

# Record bag for offline playback
ros2 bag record -a -o my_mapping_run

# Play back recorded bag
ros2 bag play my_mapping_run
```

## References (One Click)

- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **ros2-lvi GitHub**: https://github.com/ryinin/ros2-lvi
- **RPLIDAR A1 SDK**: https://github.com/Slamtec/rplidar_sdk
- **Expo Docs**: https://docs.expo.dev/
- **Open3D Docs**: http://www.open3d.org/

---

**Last Updated**: 2025-11-04  
**Status**: ✅ Ready for integration testing  
**Questions?** See MOBILE_LVISM_GUIDE.md for full troubleshooting.
