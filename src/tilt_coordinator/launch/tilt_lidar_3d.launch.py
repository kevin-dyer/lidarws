"""Launch the RPLIDAR driver and tilt-coordinated 3D capture node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_params_default = os.path.join(
        get_package_share_directory('rplidar_driver'),
        'param',
        'rplidar.yaml',
    )

    lidar_params_arg = DeclareLaunchArgument(
        'rplidar_params_file',
        default_value=lidar_params_default,
        description='Path to the RPLIDAR driver parameters YAML file.',
    )
    board_ip_arg = DeclareLaunchArgument(
        'board_ip',
        default_value='255.255.255.255',
        description=(
            'Nano UDP discovery target. Use 192.168.4.1 when connected '
            'directly to its fallback access point.'
        ),
    )
    board_udp_port_arg = DeclareLaunchArgument(
        'board_udp_port',
        default_value='4210',
        description='UDP port used by the Nano ESP32 firmware.',
    )
    host_udp_port_arg = DeclareLaunchArgument(
        'host_udp_port',
        default_value='4211',
        description='Local UDP port for Nano ESP32 responses.',
    )
    tilt_serial_port_arg = DeclareLaunchArgument(
        'tilt_serial_port',
        default_value='/tmp/tilt_lidar_udp/tilt',
        description='PTY path for tilt commands and telemetry from the UDP bridge.',
    )
    tilt_baud_arg = DeclareLaunchArgument(
        'tilt_baud',
        default_value='115200',
        description='Serial baud rate for the Arduino tilt stage.',
    )
    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/tmp/tilt_lidar_udp/lidar',
        description='PTY path for the RPLIDAR UART tunneled over UDP.',
    )
    tilt_speed_arg = DeclareLaunchArgument(
        'tilt_speed_deg_s',
        default_value='2.0',
        description='Maximum tilt sweep speed in degrees per second (0, 10].',
    )
    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.05',
        description='Minimum LiDAR range in meters accepted into the point cloud.',
    )
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='30.0',
        description='Maximum LiDAR range in meters accepted into the point cloud.',
    )
    lidar_azimuth_offset_arg = DeclareLaunchArgument(
        'lidar_azimuth_offset_deg',
        default_value='90.0',
        description='Yaw offset of LiDAR azimuth zero in the point-cloud frame.',
    )
    capture_on_start_arg = DeclareLaunchArgument(
        'capture_on_start',
        default_value='true',
        description='Start one upward tilt sweep automatically.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Start RViz with the live 3D point cloud display.',
    )
    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='~/dev/lidarws/lidar_scans',
        description='Directory for saved raw point clouds and surface meshes.',
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_driver'),
            '/launch/rplidar.launch.py',
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('rplidar_params_file'),
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': '115200',
            'serial_dtr_asserted': 'true',
        }.items(),
    )

    tilt_node = Node(
        package='tilt_coordinator',
        executable='tilt_coordinator_node',
        name='tilt_coordinator',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('tilt_serial_port'),
            'baud': ParameterValue(
                LaunchConfiguration('tilt_baud'), value_type=int),
            'tilt_speed_deg_s': ParameterValue(
                LaunchConfiguration('tilt_speed_deg_s'), value_type=float),
            'min_range': ParameterValue(
                LaunchConfiguration('min_range'), value_type=float),
            'max_range': ParameterValue(
                LaunchConfiguration('max_range'), value_type=float),
            'lidar_azimuth_offset_deg': ParameterValue(
                LaunchConfiguration('lidar_azimuth_offset_deg'),
                value_type=float),
            'capture_on_start': ParameterValue(
                LaunchConfiguration('capture_on_start'), value_type=bool),
        }],
    )

    mesh_postprocessor_node = Node(
        package='tilt_coordinator',
        executable='mesh_postprocessor_node',
        name='tilt_mesh_postprocessor',
        output='screen',
        parameters=[{
            'output_directory': LaunchConfiguration('output_directory'),
        }],
    )

    tilt_stage_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tilt_stage_static_transform',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'tilt_stage',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='tilt_lidar_3d_rviz',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('tilt_coordinator'),
                'config',
                'tilt_lidar_3d.rviz',
            ),
        ],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    udp_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tilt_coordinator', 'udp_serial_bridge',
            '--board-ip', LaunchConfiguration('board_ip'),
            '--board-port', LaunchConfiguration('board_udp_port'),
            '--local-port', LaunchConfiguration('host_udp_port'),
            '--lidar-pty', LaunchConfiguration('lidar_serial_port'),
            '--tilt-pty', LaunchConfiguration('tilt_serial_port'),
        ],
        name='udp_serial_bridge',
        output='screen',
    )

    started_after_udp = {'value': False}

    def launch_scan_stack_when_ready(event):
        if started_after_udp['value'] or 'UDP_READY' not in str(event.text):
            return []
        started_after_udp['value'] = True
        return [
            rplidar_launch,
            tilt_stage_tf,
            tilt_node,
            mesh_postprocessor_node,
            rviz_node,
        ]

    udp_ready_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=udp_bridge,
            on_stdout=launch_scan_stack_when_ready,
        )
    )

    return LaunchDescription([
        lidar_params_arg,
        board_ip_arg,
        board_udp_port_arg,
        host_udp_port_arg,
        tilt_serial_port_arg,
        tilt_baud_arg,
        lidar_serial_port_arg,
        tilt_speed_arg,
        min_range_arg,
        max_range_arg,
        lidar_azimuth_offset_arg,
        capture_on_start_arg,
        launch_rviz_arg,
        output_directory_arg,
        udp_ready_handler,
        udp_bridge,
    ])
