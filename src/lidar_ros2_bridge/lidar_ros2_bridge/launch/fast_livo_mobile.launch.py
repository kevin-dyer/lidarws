#!/usr/bin/env python3
"""
Launch file: mobile LIDAR scanner + topic adapter.

Starts the mobile bridge (UDP receivers) + topic adapter that republishes
data to FAST-LIVO2 expected topics.

Optional: Camera bridge must be run separately if needed (different Python env issue).

Usage:
  ros2 launch lidar_ros2_bridge fast_livo_mobile.launch.py mobile_ip:=192.168.1.100
"""

import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    mobile_ip = LaunchConfiguration("mobile_ip", default="192.168.1.100")

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "mobile_ip",
                default_value="192.168.1.100",
                description="Mobile device IP address",
            ),
            # =========== Mobile Bridge (UDP receivers) ===========
            ExecuteProcess(
                cmd=[shutil.which("udp_lidar_receiver")],
                name="udp_lidar_receiver",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[shutil.which("udp_imu_receiver")],
                name="udp_imu_receiver",
                output="screen",
            ),
            # =========== Topic Adapter (mobile bridge → FAST-LIVO2 topics) ===========
            # Republishes:
            #   /lidar/points → /livox/lidar
            #   /imu/data → /livox/imu
            #   /camera/image_raw → /left_camera/image
            ExecuteProcess(
                cmd=[shutil.which("fast_livo_adapter")],
                name="fast_livo_adapter",
                output="screen",
            ),
        ]
    )
