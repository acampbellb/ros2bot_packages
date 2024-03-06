#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # imu filter configuration parameters   
    imu_filter_config = os.path.join(
        get_package_share_directory('ros2bot_bringup'),
        'config',
        'imu_filter.yaml'
    )    

    # filter and fuse imu data
    imu_filter_node = Node(
        name="imu_filter",
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[
            {"fixed_frame": "base_link"},
            {"use_mag": False},
            {"publish_tf": False},
            {"use_magnetic_field_msg": False},
            {"world_frame": "enu"},
            {"orientation_stddev": 0.05},
            {"angular_scale": 1.05},
            imu_filter_config
        ]
        # ,
        # remapping=[
        #     ("/sub_imu", "/imu/imu_raw"),
        #     ("/sub_mag", "/mag/mag_raw"),
        #     ("/pub_imu", "/imu/imu_data"),
        #     ("/pub_mag", "/mag/mag_field"),
        # ]
    )

    # launch description action sequence
    ld.add_action(imu_filter_node)

    return ld
