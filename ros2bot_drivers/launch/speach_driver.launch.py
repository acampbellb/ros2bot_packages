#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # robot low-level speach driver node
    speach_driver_node = Node(
        name="speach_driver",
        package="ros2bot_drivers",
        executable="ros2bot_speach_driver_node",
        parameters=[
            {"process_cmd_freq", 0.2}
        ]
        # ,
        # remappings=[
        #     ("/pub_vel", "/vel_raw"),
        #     ("/pub_imu", "/imu/imu_raw"),
        #     ("/pub_mag", "/mag/mag_raw")
        # ]
    )

    # launch description action sequence
    ld.add_action(speach_driver_node)

    return ld