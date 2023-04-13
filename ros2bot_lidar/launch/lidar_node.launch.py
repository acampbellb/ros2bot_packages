#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # included slamtec s2 rplidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("sllidar_ros2"), '/launch', '/sllidar_s2.launch.py'])
    )

    # launch description action sequence
    ld.add_action(lidar_launch)

    return ld