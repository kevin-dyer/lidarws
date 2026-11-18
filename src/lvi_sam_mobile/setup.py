from setuptools import setup

package_name = 'lvi_sam_mobile'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['lvi_sam_mobile/launch/mobile_lvism.launch.py']),
        ('share/' + package_name + '/config', ['lvi_sam_mobile/config/lvism_mobile.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='kevin',
    author_email='kevin@todo.todo',
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    url='https://github.com/kevin-dyer/lidarws',
    description='LVI-SAM for mobile LiDAR-visual-inertial mapping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_bridge = lvi_sam_mobile.camera_bridge:main',
            'tilt_sync = lvi_sam_mobile.tilt_sync:main',
        ],
    },
)
