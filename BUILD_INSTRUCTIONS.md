# LVI-SAM Build Instructions for Your Setup

Your ros2-lvi is cloned at `~/dev/ros2_lvi_ws/src/ros2-lvi`.

## Build Order

```bash
# Terminal 1: Build ros2-lvi
cd ~/dev/ros2_lvi_ws
colcon build --packages-up-to lvi_sam
source install/setup.bash

# Terminal 2: Build lidarws packages
cd ~/dev/lidarws
colcon build --packages-up-to lidar_ros2_bridge lvi_sam_mobile
source install/setup.bash
```

## Launch Commands

**Always source BOTH workspaces before launching:**

```bash
# Source both
source /opt/ros/humble/setup.bash
source ~/dev/ros2_lvi_ws/install/setup.bash
source ~/dev/lidarws/install/setup.bash

# Launch (with your phone IP)
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:=192.168.1.100 \
  show_rviz:=true \
  show_viewer:=true
```

## Verification

```bash
# Check ros2-lvi built successfully
ls ~/dev/ros2_lvi_ws/install/lib/ | grep lvi_sam

# Check lidarws packages built
ls ~/dev/lidarws/install/lib/ | grep lidar_ros2_bridge

# List ros2-lvi packages
ros2 pkg list | grep lvi

# List lidarws packages
ros2 pkg list | grep lidar_ros2_bridge
ros2 pkg list | grep lvi_sam_mobile
```

## Troubleshooting

**Error: "lvi_sam not found"**
- Ensure ros2-lvi workspace is built: `colcon build --packages-up-to lvi_sam`
- Verify you sourced it: `source ~/dev/ros2_lvi_ws/install/setup.bash`

**Error: "Package 'lvi_sam' not found"**
- Build order matters: build ros2-lvi first, then lidarws
- Both must be sourced before launching

**Cannot find lidar_ros2_bridge**
- Build lidarws: `colcon build --packages-up-to lidar_ros2_bridge`
- Verify: `ros2 run lidar_ros2_bridge udp_lidar_receiver --help`

---

For full details, see [SETUP_CHECKLIST.md](./SETUP_CHECKLIST.md) Phase 1 and Phase 2.
