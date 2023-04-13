#!/usr/bin/env python3

import os

from glob import glob
from setuptools import setup

package_name = 'ros2bot_lidar'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abcampbellb',
    maintainer_email='abcampbellb@gmail.com',
    description='ros2bot lidar package for S2 RPLIDAR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_avoidance_node = ros2bot_lidar.ros2bot_lidar_avoidance_node:main',
            'lidar_tracking_node = ros2bot_lidar.ros2bot_lidar_tracking_node:main',
            'lidar_warning_node = ros2bot_lidar.ros2bot_lidar_warning_node:main'
        ],
    },
)
