#!/bin/bash
# Quick reference commands for FAST-LIVO2 + mobile scanner integration

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== FAST-LIVO2 Mobile Scanner Setup ===${NC}"
echo ""

# Check prerequisites
echo -e "${YELLOW}1. Prerequisites${NC}"
echo "   Setup: Both workspaces must be built and sourced"
echo ""
echo "   # Build lidarws"
echo "   cd ~/dev/lidarws && colcon build --packages-select lidar_ros2_bridge lvi_sam_mobile"
echo ""
echo "   # Build FAST-LIVO2 workspace"
echo "   cd ~/dev/ros2_fast_livo_ws && colcon build --packages-select fast_livo"
echo ""

# Quick launch
echo -e "${YELLOW}2. Quick Launch (All-in-one)${NC}"
echo "   # Replace 192.168.1.100 with your phone IP"
echo "   source ~/.bashrc  # Ensure both workspaces are sourced"
echo "   ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py mobile_ip:=192.168.1.100 use_rviz:=True"
echo ""

# Manual launch
echo -e "${YELLOW}3. Manual Launch (Separate terminals)${NC}"
echo ""
echo "   # Terminal 1: Mobile bridge"
echo "   cd ~/dev/lidarws && source install/setup.bash"
echo "   ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py mobile_ip:=192.168.1.100 show_rviz:=false show_viewer:=false"
echo ""
echo "   # Terminal 2: Topic adapter"
echo "   cd ~/dev/lidarws && source install/setup.bash"
echo "   ros2 run lidar_ros2_bridge fast_livo_adapter"
echo ""
echo "   # Terminal 3: FAST-LIVO2"
echo "   cd ~/dev/ros2_fast_livo_ws && source install/setup.bash"
echo "   ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True"
echo ""

# Verification
echo -e "${YELLOW}4. Verify Data Flow${NC}"
echo "   # Check topics are active"
echo "   ros2 topic list"
echo ""
echo "   # Monitor LiDAR rate"
echo "   ros2 topic hz /livox/lidar"
echo ""
echo "   # Monitor IMU rate (should be ~20 Hz)"
echo "   ros2 topic hz /livox/imu"
echo ""
echo "   # Monitor camera rate"
echo "   ros2 topic hz /left_camera/image"
echo ""

# Recording
echo -e "${YELLOW}5. Record Data${NC}"
echo "   # Record all topics to rosbag"
echo "   ros2 bag record -a -o fast_livo_run_$(date +%Y%m%d_%H%M%S)"
echo ""

# Troubleshooting
echo -e "${YELLOW}6. Troubleshooting${NC}"
echo "   # Check what topics adapter is seeing"
echo "   ros2 topic echo /lidar/points"
echo ""
echo "   # Debug adapter"
echo "   ros2 run --prefix 'gdb -ex run --args' lidar_ros2_bridge fast_livo_adapter"
echo ""
echo "   # Check FAST-LIVO2 config"
echo "   cat ~/dev/ros2_fast_livo_ws/src/FAST-LIVO2/config/avia.yaml"
echo ""

echo -e "${GREEN}Setup complete! For detailed docs, see FAST_LIVO2_INTEGRATION.md${NC}"
