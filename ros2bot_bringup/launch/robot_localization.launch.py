#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # extended kalman data fusion    
    localization_config = os.path.join(
        get_package_share_directory('ros2bot_bringup'),
        'config',
        'robot_localization.yaml'
    )

    localization_node = Node(
        name="robot_localization",
        package="robot_localization",
        executable="ekf_localization_node",
        output="screen",
        parameters=[
            {"odom_frame": "/odom"},
            {"world_frame": "/odom"},
            {"base_link_frame": "/base_footprint"},
            localization_config
        ],
        remapping=[
            ("odometry/filtered", "odom"),
            ("/imu0", "/imu/imu_data"),
            ("/odom0", "odom_raw"),
        ]
    )

    # launch description action sequence
    ld.add_action(localization_node)

    return ld
