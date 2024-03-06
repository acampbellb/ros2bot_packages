#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # include ros2bot base lidar launch
    lidar_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_lidar"), '/launch', '/lidar_base.launch.py'])
    )

    # ros2bot lidar tracker node
    lidar_tracker_node = Node(
        name="lidar_tracker",        
        package="ros2bot_lidar",
        executable="lidar_tracker_node",
        parameters=[
            {"switch" : False},
            {"priority_angle" : 30},
            {"angular" : 1.0},
            {"laser_angle" : 40.0},
            {"response_dist" : 0.55},
            {"linear_kp" : 2.0},
            {"linear_ki" : 0.0},
            {"linear_kd" : 2.0},
            {"angle_kp" : 3.0},
            {"angle_ki" : 0.0},
            {"angle_kd" : 5.0}
        ]
    )

    # launch description action sequence
    ld.add_action(lidar_base_launch)
    ld.add_action(lidar_tracker_node)

    return ld