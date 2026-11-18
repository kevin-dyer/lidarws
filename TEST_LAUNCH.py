#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lidar_ros2_bridge",
            executable="udp_lidar_receiver",
            name="test",
            output="screen",
        ),
    ])
