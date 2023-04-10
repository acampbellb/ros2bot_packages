#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # filter and fuse imu data
    imu_filter_node = Node(
        name="imu_filter",
        package="imu_filter_madgwick",
        executable="imu_filter_node",
        parameters=[
            {"fixed_frame": "base_link"},
            {"use_mag": False},
            {"publish_tf": False},
            {"use_magnetic_field_msg": False},
            {"world_frame": "enu"},
            {"orientation_stddev": 0.05},
            {"angular_scale": 1.05},
        ],
        remapping=[
            ("/sub_imu", "/imu/imu_raw"),
            ("/sub_mag", "/mag/mag_raw"),
            ("/pub_imu", "/imu/imu_data"),
            ("/pub_mag", "/mag/mag_field"),
        ]
    )

    # launch description action sequence
    ld.add_action(imu_filter_node)

    return ld
