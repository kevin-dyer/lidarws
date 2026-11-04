from glob import glob
from setuptools import find_packages, setup

package_name = 'scan_postprocess'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'pillow', 'opencv-python'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='Desktop post-processing for tilt lidar scans: colouring and registration.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colorize_scan = scan_postprocess.colorize_cli:main',
            'register_scans = scan_postprocess.register_cli:main',
            'mesh_scan = scan_postprocess.mesh_cli:main',
            'process_group = scan_postprocess.group_cli:main',
        ],
    },
)
