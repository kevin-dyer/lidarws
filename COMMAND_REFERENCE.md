# Command Reference Card

## Your Setup
```
ros2-lvi:  ~/dev/ros2_lvi_ws/src/ros2-lvi
lidarws:   ~/dev/lidarws
```

## Build (Do This First)

```bash
# Build ros2-lvi
cd ~/dev/ros2_lvi_ws
colcon build --packages-up-to lvi_sam
echo "✓ ros2-lvi built"

# Build lidarws packages
cd ~/dev/lidarws
colcon build --packages-up-to lidar_ros2_bridge lvi_sam_mobile
echo "✓ lidarws built"
```

## Launch (Choose One)

### Option A: Automatic (Recommended)
```bash
cd ~/dev/lidarws
./launch_mobile_lvism.sh 192.168.1.100 true true
# Arguments: <phone_ip> [show_rviz] [show_viewer]
# Defaults: phone_ip=192.168.1.100, show_rviz=true, show_viewer=true
```

### Option B: Manual
```bash
# Terminal 1: Source and launch
source /opt/ros/humble/setup.bash
source ~/dev/ros2_lvi_ws/install/setup.bash
source ~/dev/lidarws/install/setup.bash

ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:=192.168.1.100 \
  show_rviz:=true \
  show_viewer:=true
```

## Verify Installation

```bash
# Check builds
ls ~/dev/ros2_lvi_ws/build/lvi_sam >/dev/null && echo "✓ ros2-lvi built"
ls ~/dev/lidarws/build/lidar_ros2_bridge >/dev/null && echo "✓ lidarws built"

# Check packages can be found
source /opt/ros/humble/setup.bash
source ~/dev/ros2_lvi_ws/install/setup.bash
source ~/dev/lidarws/install/setup.bash
ros2 pkg list | grep lvi_sam && echo "✓ lvi_sam found"
ros2 pkg list | grep lidar_ros2_bridge && echo "✓ lidar_ros2_bridge found"
```

## Monitor During Mapping

```bash
# Terminal 2: Watch LiDAR points
ros2 topic echo /lidar/points --once

# Terminal 2: Watch IMU data
ros2 topic echo /imu/data --once

# Terminal 2: Watch LVI-SAM output
ros2 topic echo /lvi_sam/cloud_registered --once

# Terminal 2: List all active nodes
ros2 node list

# Terminal 2: Show all topics
ros2 topic list -v

# Terminal 2: Monitor node communication graph (requires rqt)
ros2 run rqt_graph rqt_graph
```

## On Phone

```
App → Mobile mapping → Configure → Start mapping → Hold still 5s → Walk
```

## After Mapping

**On desktop** (automatic):
- Check terminal for LVI-SAM tracking messages
- Open3D viewer shows final cloud + trajectory
- RViz2 displays full visualization
- Files remain in ROS topics for post-processing

**On phone**:
- Final cloud displayed in app
- Press "Save point cloud"
- Check gallery for saved scan

**Saved files on phone**:
- `{scan_id}_raw_cloud.ply` — final point cloud
- `{scan_id}_video.mp4` — synchronized video
- `{scan_id}_trajectory.json` — pose trajectory

## Clean / Rebuild

```bash
# Clean ros2-lvi build
cd ~/dev/ros2_lvi_ws
rm -rf build install log
colcon build --packages-up-to lvi_sam

# Clean lidarws build
cd ~/dev/lidarws
rm -rf build install log
colcon build --packages-up-to lidar_ros2_bridge lvi_sam_mobile

# Clean just one package (e.g., lidar_ros2_bridge)
cd ~/dev/lidarws
colcon build --packages-select lidar_ros2_bridge
```

## Troubleshooting

### "lvi_sam not found"
```bash
cd ~/dev/ros2_lvi_ws
colcon build --packages-up-to lvi_sam
source install/setup.bash
ros2 pkg list | grep lvi_sam  # Should find it
```

### "lidar_ros2_bridge not found"
```bash
cd ~/dev/lidarws
colcon build --packages-up-to lidar_ros2_bridge
source install/setup.bash
ros2 pkg list | grep lidar_ros2_bridge  # Should find it
```

### "UDP port blocked" (no LiDAR data)
```bash
sudo ufw allow 5000:5002/udp
sudo ufw status numbered | grep 5000
```

### "Camera not appearing"
```bash
# Test RTSP stream (replace IP with phone IP)
ffplay rtsp://192.168.1.100:8554/stream
```

### High latency
```bash
# Reduce downsampling rate for more frequent phone updates
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:=192.168.1.100 \
  downsample_rate:=5  # More frequent (default 10)
```

## Documentation

- **Setup**: [SETUP_CHECKLIST.md](./SETUP_CHECKLIST.md)
- **Usage**: [MOBILE_LVISM_GUIDE.md](./MOBILE_LVISM_GUIDE.md)
- **Reference**: [QUICK_REFERENCE.md](./QUICK_REFERENCE.md)
- **Build**: [BUILD_INSTRUCTIONS.md](./BUILD_INSTRUCTIONS.md)

## Quick Network Info

Find your IPs:
```bash
# Desktop IP
hostname -I

# Phone IP
# On Android: Settings → About → IP address
# On iOS: Settings → Wi-Fi → current network → IP address
```

---

**Save this for quick reference during development!**
