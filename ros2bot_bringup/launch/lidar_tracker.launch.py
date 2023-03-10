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
            FindPackageShare("ros2bot_bringup"), '/launch', '/lidar_base.launch.py'])
    )

    # ros2bot lidar tracker node
    lidar_tracker_node = Node(
        package="ros2bot_lidar",
        executable="lidar_tracker_node",
        name="lidar_tracker_node",
        parameters=[
            {"switch" : False},
            {"priority_angle" : 30},
            {"angular" : 1.0},
            {"laser_angle" : 65},
            {"response_dist" : 1.0},
            {"linear_kp" : 2.0},
            {"linear_ki" : 0.0},
            {"linear_kd" : 2.0},
            {"angle_kp" : 3.0},
            {"angle_ki" : 0.0},
            {"angle_kd" : 5.0}
        ]
    )

    # add nodes to launch description
    ld.add_action(lidar_base_launch)
    ld.add_action(lidar_tracker_node)

    # return launch description
    return ld