"""
Launch file for mobile LVI-SAM.
Orchestrates the full pipeline with ros2-lvi as the backend.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mobile_ip = LaunchConfiguration("mobile_ip", default="192.168.1.100")
    camera_rtsp = LaunchConfiguration("camera_rtsp", default="rtsp://192.168.1.100:8554/stream")

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument("mobile_ip", default_value="192.168.1.100", description="Mobile device IP"),
            DeclareLaunchArgument(
                "camera_rtsp",
                default_value="rtsp://192.168.1.100:8554/stream",
                description="RTSP URL for phone camera",
            ),
            # Camera bridge (RTSP → ROS Image)
            Node(
                package="lvi_sam_mobile",
                executable="camera_bridge",
                name="camera_bridge",
                parameters=[{"rtsp_url": camera_rtsp}],
                output="screen",
            ),
            # Tilt synchronizer (tilt angle → joint state)
            Node(
                package="lvi_sam_mobile",
                executable="tilt_sync",
                name="tilt_sync",
                output="screen",
            ),
            # ros2-lvi node (main LVI-SAM backend)
            Node(
                package="lvi_sam",
                executable="lvi_sam_node",
                name="lvi_sam",
                parameters=[
                    {
                        "lidar_topic": "/lidar/points",
                        "imu_topic": "/imu/data",
                        "camera_topic": "/camera/image_raw",
                        "use_imu_orientation": True,
                        "visualize_trajectory": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
