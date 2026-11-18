"""
ROS 2 launch file for mobile LiDAR-inertial mapping pipeline.

Orchestrates:
1. UDP LiDAR receiver (from phone scanner)
2. UDP IMU receiver (from phone accelerometer/gyroscope)
3. LVI-SAM node (from ros2-lvi)
4. Live cloud publisher (LVI-SAM output → ROS topics + UDP to phone)
5. Desktop viewer (Open3D visualization)
6. RViz2 (optional visualization)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Arguments
    lidar_port = LaunchConfiguration("lidar_port", default="5000")
    imu_port = LaunchConfiguration("imu_port", default="5001")
    cloud_port = LaunchConfiguration("cloud_port", default="5002")
    mobile_ip = LaunchConfiguration("mobile_ip", default="192.168.1.100")
    desktop_ip = LaunchConfiguration("desktop_ip", default="0.0.0.0")
    show_rviz = LaunchConfiguration("show_rviz", default="true")
    show_viewer = LaunchConfiguration("show_viewer", default="true")

    # Get package share directory
    pkg_share = FindPackageShare("lidar_ros2_bridge").find("lidar_ros2_bridge")
    rviz_config = os.path.join(pkg_share, "config", "rviz_mobile_mapping.rviz")

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument("lidar_port", default_value="5000", description="UDP port for LiDAR frames"),
            DeclareLaunchArgument("imu_port", default_value="5001", description="UDP port for IMU data"),
            DeclareLaunchArgument("cloud_port", default_value="5002", description="UDP port for live cloud (to phone)"),
            DeclareLaunchArgument("mobile_ip", default_value="192.168.1.100", description="Mobile device IP address"),
            DeclareLaunchArgument("desktop_ip", default_value="0.0.0.0", description="Desktop IP for broadcasting"),
            DeclareLaunchArgument("show_rviz", default_value="true", description="Show RViz2 visualization"),
            DeclareLaunchArgument("show_viewer", default_value="true", description="Show Open3D desktop viewer"),
            # UDP receivers
            Node(
                package="lidar_ros2_bridge",
                executable="udp_lidar_receiver",
                name="udp_lidar_receiver",
                parameters=[
                    {"lidar_port": lidar_port},
                    {"frame_id": "lidar"},
                ],
                output="screen",
            ),
            Node(
                package="lidar_ros2_bridge",
                executable="udp_imu_receiver",
                name="udp_imu_receiver",
                parameters=[
                    {"imu_port": imu_port},
                    {"frame_id": "phone_imu"},
                ],
                output="screen",
            ),
            # LVI-SAM node (from ros2-lvi package)
            Node(
                package="lvi_sam",
                executable="lvi_sam_node",
                name="lvi_sam",
                parameters=[
                    {"lidar_topic": "/lidar/points"},
                    {"imu_topic": "/imu/data"},
                    {"camera_topic": "/camera/image_raw"},
                    {"extrinsic_T": [0.0, 0.0, 0.0]},
                    {"extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
                ],
                output="screen",
            ),
            # Live cloud publisher
            Node(
                package="lidar_ros2_bridge",
                executable="live_cloud_publisher",
                name="live_cloud_publisher",
                parameters=[
                    {"mobile_ip": mobile_ip},
                    {"udp_port": cloud_port},
                    {"downsample_rate": 10},
                ],
                output="screen",
            ),
            # Desktop viewer (Open3D)
            Node(
                package="lidar_ros2_bridge",
                executable="desktop_viewer",
                name="desktop_viewer",
                condition=LaunchConfiguration("show_viewer"),
                output="screen",
            ),
            # RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=LaunchConfiguration("show_rviz"),
                output="screen",
            ),
        ]
    )
