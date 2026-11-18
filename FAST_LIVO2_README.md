# FAST-LIVO2 Integration with Mobile 3D Scanner

🎯 **Quick Start:** 
```bash
ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py mobile_ip:=192.168.1.100 use_rviz:=True
```

---

## 📚 Documentation Index

Navigate to the appropriate guide based on your needs:

### 🚀 **Getting Started**
- **[FAST_LIVO2_SETUP_CHECKLIST.md](FAST_LIVO2_SETUP_CHECKLIST.md)** — Start here!
  - Pre-launch verification
  - Build verification
  - Launch execution
  - Data flow verification
  - Troubleshooting checklist

### 📋 **Main Documentation**
- **[FAST_LIVO2_INTEGRATION.md](FAST_LIVO2_INTEGRATION.md)** — Comprehensive guide
  - Architecture overview
  - Setup instructions (both workspaces)
  - Usage (one-command vs manual)
  - Verification commands
  - Troubleshooting with solutions
  - Data format details

### 🔍 **Technical Details**
- **[FAST_LIVO2_TOPIC_MAP.md](FAST_LIVO2_TOPIC_MAP.md)** — Topic specifications
  - Complete data flow diagram
  - Topic specification table
  - PointCloud2, IMU, and Image message formats
  - Launch file remapping details
  - Configuration tuning guide
  - Debugging checklist

- **[FAST_LIVO2_DATA_FLOW.txt](FAST_LIVO2_DATA_FLOW.txt)** — Visual data flow
  - 5-stage data flow explanation
  - Latency and timing analysis
  - Key design points
  - Data rate examples
  - Troubleshooting checklist

### ⚡ **Quick Reference**
- **[FAST_LIVO2_QUICK_COMMANDS.sh](FAST_LIVO2_QUICK_COMMANDS.sh)** — Copy/paste commands
  - Prerequisites
  - Quick launch
  - Manual launch
  - Verification commands
  - Recording and troubleshooting

- **[FAST_LIVO2_SUMMARY.txt](FAST_LIVO2_SUMMARY.txt)** — Executive summary
  - Problem statement
  - Solution overview
  - New files created
  - Quick start
  - Architecture diagram

---

## 🎯 What This Solution Provides

### Problem
Your mobile 3D LiDAR scanner publishes to:
- `/lidar/points` (UDP 5000)
- `/imu/data` (UDP 5001)  
- `/camera/image_raw` (RTSP 8554)

FAST-LIVO2 expects:
- `/livox/lidar`
- `/livox/imu`
- `/left_camera/image`

### Solution
A lightweight **topic adapter** that republishes your data to FAST-LIVO2 topics without modifying either system.

### New Files Created

| File | Purpose |
|------|---------|
| `fast_livo_adapter.py` | ROS 2 node that republishes mobile bridge data to Livox topics |
| `fast_livo_mobile.launch.py` | One-command launch orchestrating entire pipeline |
| `setup.py` | Updated entry points for new executable |

### Documentation Created

| File | Purpose |
|------|---------|
| `FAST_LIVO2_INTEGRATION.md` | Complete setup and usage guide |
| `FAST_LIVO2_TOPIC_MAP.md` | Detailed topic specifications and data formats |
| `FAST_LIVO2_DATA_FLOW.txt` | Visual data flow and architecture |
| `FAST_LIVO2_SETUP_CHECKLIST.md` | Step-by-step verification checklist |
| `FAST_LIVO2_QUICK_COMMANDS.sh` | Quick reference commands |
| `FAST_LIVO2_SUMMARY.txt` | Executive overview |
| `FAST_LIVO2_README.md` | This file |

---

## ✨ Key Advantages

✅ **No Code Changes**
- Adapter is standalone
- Mobile bridge unchanged
- FAST-LIVO2 unchanged

✅ **Zero Latency Overhead**
- Direct message passthrough
- No data transformation
- Minimal CPU impact

✅ **Independent Systems**
- Each workspace maintains its own build
- Easy to debug individually
- Can swap components without rebuilding

✅ **Standards-Based**
- Uses ROS 2 standard message types
- Compatible with any LiDAR/camera source
- Future-proof design

---

## 🚀 Quick Start Paths

### Path 1: Everything Works (Typical Case)
1. Read: [FAST_LIVO2_SETUP_CHECKLIST.md](FAST_LIVO2_SETUP_CHECKLIST.md)
2. Run: `ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py mobile_ip:=YOUR_IP use_rviz:=True`
3. Done! Watch RViz2 for point cloud and trajectory.

### Path 2: I Need to Troubleshoot
1. Read: [FAST_LIVO2_INTEGRATION.md](FAST_LIVO2_INTEGRATION.md) Troubleshooting section
2. Use: `ros2 topic hz` and `ros2 topic echo` commands from [FAST_LIVO2_QUICK_COMMANDS.sh](FAST_LIVO2_QUICK_COMMANDS.sh)
3. Check: [FAST_LIVO2_TOPIC_MAP.md](FAST_LIVO2_TOPIC_MAP.md) for expected data formats

### Path 3: I Want Complete Understanding
1. Read: [FAST_LIVO2_SUMMARY.txt](FAST_LIVO2_SUMMARY.txt) for overview
2. Study: [FAST_LIVO2_DATA_FLOW.txt](FAST_LIVO2_DATA_FLOW.txt) for architecture
3. Review: [FAST_LIVO2_TOPIC_MAP.md](FAST_LIVO2_TOPIC_MAP.md) for technical details

---

## 📁 File Locations

### Source Code
```
src/lidar_ros2_bridge/
├── lidar_ros2_bridge/
│   ├── fast_livo_adapter.py              ← Topic adapter (NEW)
│   ├── launch/
│   │   └── fast_livo_mobile.launch.py    ← Main launch file (NEW)
│   ├── udp_lidar_receiver.py
│   ├── udp_imu_receiver.py
│   └── ... (existing files)
└── setup.py                               ← Updated (NEW entry points)
```

### Documentation
```
~/dev/lidarws/
├── FAST_LIVO2_README.md                  ← This file (index)
├── FAST_LIVO2_SETUP_CHECKLIST.md         ← Start here!
├── FAST_LIVO2_INTEGRATION.md             ← Complete guide
├── FAST_LIVO2_TOPIC_MAP.md               ← Technical specs
├── FAST_LIVO2_DATA_FLOW.txt              ← Architecture
├── FAST_LIVO2_QUICK_COMMANDS.sh          ← Commands
├── FAST_LIVO2_SUMMARY.txt                ← Overview
└── ... (existing files)
```

### FAST-LIVO2 Config
```
~/dev/ros2_fast_livo_ws/
└── src/FAST-LIVO2/
    └── config/
        ├── avia.yaml                     ← Main config (RPLIDAR compatible)
        ├── camera_pinhole.yaml           ← Camera intrinsics
        └── ... (other sensor configs)
```

---

## 🔄 Data Flow Overview

```
Mobile Device (3D Scanner + Phone)
    ↓ UDP 5000 (LiDAR) + UDP 5001 (IMU) + RTSP 8554 (Camera)
Mobile Bridge (UDP receivers + RTSP decoder)
    ↓ /lidar/points, /imu/data, /camera/image_raw
Topic Adapter (NEW: fast_livo_adapter)
    ↓ /livox/lidar, /livox/imu, /left_camera/image
FAST-LIVO2 (LiDAR-Inertial-Visual SLAM)
    ↓ /tf, /odom, /cloud_pcd
RViz2 (3D Map + Trajectory Visualization)
```

---

## ⚙️ System Requirements

- **OS**: Ubuntu 22.04 (ROS 2 Humble)
- **Hardware**: 
  - Desktop with 4+ GB RAM
  - WiFi network or USB tether
  - GPU optional (FAST-LIVO2 runs on CPU)
- **ROS 2**: Humble
- **Both Workspaces Built**:
  - `~/dev/lidarws` (your mobile bridge)
  - `~/dev/ros2_fast_livo_ws` (FAST-LIVO2)

---

## 🧪 Verification Commands

```bash
# Verify adapter is installed
ros2 run lidar_ros2_bridge fast_livo_adapter --help

# Verify launch file is installed
ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py --show-args

# Verify FAST-LIVO2 is built
ros2 run fast_livo fastlivo_mapping --help
```

---

## 📝 Common Tasks

### Launch everything at once
```bash
ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py mobile_ip:=192.168.1.100 use_rviz:=True
```

### Launch mobile bridge only (without FAST-LIVO2)
```bash
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py mobile_ip:=192.168.1.100
```

### Launch FAST-LIVO2 only (assuming mobile bridge is already running)
```bash
ros2 run lidar_ros2_bridge fast_livo_adapter  # Terminal 1
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True  # Terminal 2
```

### Record data to rosbag
```bash
ros2 bag record -a -o my_scan_$(date +%Y%m%d_%H%M%S)
```

### Export point cloud map
```bash
# Map is published on /cloud_pcd topic
# Use RViz2 or:
ros2 topic echo /cloud_pcd > map.pcl
```

---

## 🐛 Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| "No topics appearing" | Check mobile IP, verify mobile app is running |
| "adapter node not found" | Run `colcon build --packages-select lidar_ros2_bridge` |
| "FAST-LIVO2 not starting" | Source `~/dev/ros2_fast_livo_ws/install/setup.bash` |
| "No map in RViz2" | Ensure `/cloud_pcd` display is added in RViz2 |
| "Poor tracking quality" | Check camera-LiDAR calibration in `avia.yaml` |

→ See [FAST_LIVO2_INTEGRATION.md](FAST_LIVO2_INTEGRATION.md) Troubleshooting for detailed solutions

---

## 📖 Additional Resources

- [FAST-LIVO2 GitHub Repository](https://github.com/hku-mars/FAST-LIVO2)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [sensor_msgs Package](http://docs.ros.org/en/humble/api/sensor_msgs/index.html)

---

## ✅ Next Steps

1. **Read** [FAST_LIVO2_SETUP_CHECKLIST.md](FAST_LIVO2_SETUP_CHECKLIST.md)
2. **Verify** all prerequisites are met
3. **Launch** with your mobile device IP
4. **Verify** topics are active (`ros2 topic hz`)
5. **Visualize** in RViz2

**You're ready to go!** 🚀

