#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # robot low-level master driver node
    master_driver_node = Node(
        name="master_driver",
        package="ros2bot_drivers",
        executable="master_driver_node",
        parameters=[
            {"xlinear_limit" : 1.0},
            {"ylinear_limit" : 1.0},
            {"angular_limit" : 5.0},
            {"imu_link" : "imu_link"}
        ]
        # ,
        # remappings=[
        #     ("/pub_vel", "/vel_raw"),
        #     ("/pub_imu", "/imu/imu_raw"),    
        #     ("/pub_mag", "/mag/mag_raw")        
        # ]
    )

    # launch description action sequence
    ld.add_action(master_driver_node)

    return ld
