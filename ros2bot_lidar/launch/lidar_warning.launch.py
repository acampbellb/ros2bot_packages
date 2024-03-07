#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # included ros2bot base lidar launch
    lidar_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_lidar"), '/launch', '/lidar_base.launch.py'])
    )

    # ros2bot lidar warning node
    lidar_warning_node = Node(
        name="lidar_warning",        
        package="ros2bot_lidar",
        executable="ros2bot_lidar_warning_node",
        parameters=[
            {"switch" : False},
            {"laser_angle" : 40.0},
            {"response_dist" : 0.55},
            {"angle_kp" : 3.0},
            {"angle_ki" : 0.0},
            {"angle_kd" : 5.0}
        ]
    )

    # launch description action sequence
    ld.add_action(lidar_base_launch)
    ld.add_action(lidar_warning_node)

    return ld