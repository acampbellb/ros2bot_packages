#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # robot low-level driver node
    master_driver_node = Node(
        package="ros2bot_drivers",
        executable="master_driver_node",
        parameters=[
            {"xlinear_speed_limit" : 1.0},
            {"ylinear_speed_limit" : 1.0},
            {"angular_speed_limit" : 5.0},
            {"imu_link" : "imu_link"}
        ],
        remappings=[
            ("/pub_vel", "/vel_raw"),
            ("/pub_imu", "/imu/imu_raw"),    
            ("/pub_mag", "/mag/mag_raw")        
        ]
    )

    # add nodes to launch description
    ld.add_action(master_driver_node)

    # return launch description
    return ld
