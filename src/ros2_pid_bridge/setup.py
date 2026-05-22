import os
from glob import glob
from setuptools import setup

package_name = 'ros2_pid_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 bridge for ESP32 PID wheel controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge       = ros2_pid_bridge.serial_bridge:main',
            'position_controller = ros2_pid_bridge.position_controller:main',
        ],
    },
)
