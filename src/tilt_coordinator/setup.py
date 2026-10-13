from glob import glob
from setuptools import find_packages, setup

package_name = 'tilt_coordinator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/doc',
            glob('doc/*.md')),
    ],
    install_requires=['setuptools', 'numpy', 'pyserial', 'bleak'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='Synchronizes RPLidar scans with an Arduino tilt stage.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tilt_coordinator_node = tilt_coordinator.tilt_coordinator_node:main',
            'mesh_postprocessor_node = tilt_coordinator.mesh_postprocessor_node:main',
            'ble_serial_bridge = tilt_coordinator.ble_serial_bridge:main',
            'usb_serial_bridge = tilt_coordinator.usb_serial_bridge:main',
        ],
    },
)
