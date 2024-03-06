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

    # ros2bot lidar avoidance node
    lidar_avoidance_node = Node(
        name="lidar_avoidance",        
        package="ros2bot_lidar",
        executable="lidar_avoidance_node",
        parameters=[
            {"switch" : False},
            {"linear" : 0.5},
            {"angular" : 1.0},
            {"laser_angle" : 40.0},
            {"response_dist" : 0.55}
        ]
    )

    # launch description action sequence
    ld.add_action(lidar_base_launch)
    ld.add_action(lidar_avoidance_node)

    return ld