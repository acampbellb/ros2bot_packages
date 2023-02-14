#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
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

    # included ros2bot master driver launch
    master_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/master_driver.launch.py'])
    )

    # included ros2bot joy control launch
    joy_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/joy_control.launch.py'])
    )

    # add nodes to launch description
    ld.add_action(lidar_launch)
    ld.add_action(master_driver_launch)
    ld.add_action(joy_control_launch)

    # return launch description
    return ld