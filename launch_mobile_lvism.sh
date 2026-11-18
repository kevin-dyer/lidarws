#!/bin/bash
# Quick launch script for LVI-SAM mobile mapping
# Usage: ./launch_mobile_lvism.sh <phone_ip> [show_rviz] [show_viewer]

set -e

PHONE_IP="${1:-192.168.1.100}"
SHOW_RVIZ="${2:-true}"
SHOW_VIEWER="${3:-true}"

echo "=========================================="
echo "LVI-SAM Mobile Mapping Launcher"
echo "=========================================="
echo "Phone IP:     $PHONE_IP"
echo "Show RViz2:   $SHOW_RVIZ"
echo "Show Viewer:  $SHOW_VIEWER"
echo ""

# Source ROS 2 and workspaces
echo "Sourcing ROS 2 and workspaces..."
source /opt/ros/humble/setup.bash
source ~/dev/ros2_lvi_ws/install/setup.bash
source ~/dev/lidarws/install/setup.bash

echo "✓ ROS 2 Humble sourced"
echo "✓ ros2-lvi workspace sourced"
echo "✓ lidarws workspace sourced"
echo ""

# Verify packages are available
echo "Verifying packages..."
if ! ros2 pkg list | grep -q lvi_sam; then
    echo "❌ Error: lvi_sam not found. Build ros2-lvi first:"
    echo "   cd ~/dev/ros2_lvi_ws && colcon build --packages-up-to lvi_sam"
    exit 1
fi
echo "✓ lvi_sam found"

if ! ros2 pkg list | grep -q lidar_ros2_bridge; then
    echo "❌ Error: lidar_ros2_bridge not found. Build lidarws first:"
    echo "   cd ~/dev/lidarws && colcon build --packages-up-to lidar_ros2_bridge"
    exit 1
fi
echo "✓ lidar_ros2_bridge found"

echo ""
echo "=========================================="
echo "Starting LVI-SAM pipeline..."
echo "=========================================="
echo ""
echo "Desktop displays:"
echo "  • Open3D viewer (3D point cloud + trajectory)"
echo "  • RViz2 (full ROS visualization)"
echo ""
echo "Phone displays:"
echo "  • Live cloud from desktop"
echo "  • Mapping stats (tracking state, elapsed time, point count)"
echo ""
echo "Press Ctrl-C to stop."
echo ""

# Launch the mobile mapping pipeline
ros2 launch lidar_ros2_bridge mobile_lidar_bridge.launch.py \
  mobile_ip:="$PHONE_IP" \
  show_rviz:="$SHOW_RVIZ" \
  show_viewer:="$SHOW_VIEWER"
