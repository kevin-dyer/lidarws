#!/usr/bin/env python3
"""
FAST-LIVO2 standalone launch file.

This assumes the topic adapter is already running, providing:
  - /livox/lidar (PointCloud2)
  - /livox/imu (Imu)
  - /left_camera/image (Image)

Launch this AFTER starting the mobile bridge + adapter.

Usage:
  ros2 launch lidar_ros2_bridge fast_livo_standalone.launch.py use_rviz:=True
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz", default="True")

    # Try to find FAST-LIVO2 config files
    try:
        fast_livo_pkg = FindPackageShare("fast_livo").find("fast_livo")
        avia_config = os.path.join(fast_livo_pkg, "config", "avia.yaml")
        camera_config = os.path.join(fast_livo_pkg, "config", "camera_pinhole.yaml")
        rviz_config = os.path.join(fast_livo_pkg, "rviz_cfg", "fast_livo2.rviz")
    except Exception:
        # Fallback to common installation paths
        fast_livo_root = os.path.expanduser("~/dev/ros2_fast_livo_ws/install/fast_livo")
        avia_config = os.path.join(fast_livo_root, "share", "fast_livo", "config", "avia.yaml")
        camera_config = os.path.join(fast_livo_root, "share", "fast_livo", "config", "camera_pinhole.yaml")
        rviz_config = os.path.join(fast_livo_root, "share", "fast_livo", "rviz_cfg", "fast_livo2.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="True",
                description="Whether to launch RViz2",
            ),
            # Parameter blackboard (if available)
            Node(
                package="demo_nodes_cpp",
                executable="parameter_blackboard",
                name="parameter_blackboard",
                parameters=[camera_config] if os.path.exists(camera_config) else [],
                output="screen",
            ),
            # Image republisher for compatibility
            Node(
                package="image_transport",
                executable="republish",
                name="republish",
                arguments=["compressed", "raw"],
                remappings=[
                    ("in", "/left_camera/image"),
                    ("out", "/left_camera/image"),
                ],
                output="screen",
            ),
            # FAST-LIVO2 main node
            Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="laserMapping",
                parameters=[avia_config] if os.path.exists(avia_config) else [],
                output="screen",
            ),
            # RViz2 visualization (optional)
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
                output="screen",
            ),
        ]
    )
