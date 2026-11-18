from setuptools import setup

package_name = 'lidar_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'lidar_ros2_bridge/launch/mobile_lidar_bridge.launch.py',
            'lidar_ros2_bridge/launch/fast_livo_mobile.launch.py',
            'lidar_ros2_bridge/launch/fast_livo_standalone.launch.py',
        ]),
        ('share/' + package_name + '/config', ['lidar_ros2_bridge/config/rviz_mobile_mapping.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='kevin',
    author_email='kevin@todo.todo',
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    url='https://github.com/kevin-dyer/lidarws',
    description='UDP bridge for mobile LiDAR-inertial mapping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_lidar_receiver = lidar_ros2_bridge.udp_lidar_receiver:main',
            'udp_imu_receiver = lidar_ros2_bridge.udp_imu_receiver:main',
            'live_cloud_publisher = lidar_ros2_bridge.live_cloud_publisher:main',
            'desktop_viewer = lidar_ros2_bridge.desktop_viewer:main',
            'fast_livo_adapter = lidar_ros2_bridge.fast_livo_adapter:main',
        ],
    },
)
