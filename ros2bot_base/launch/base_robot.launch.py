#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # robot odometry publisher
    base_robot_node = Node(
        package="ros2bot_base",
        executable="ros2bot_base_node",
        parameters=[
            {"odom_frame": "odom"},
            {"base_footprint_frame": "base_footprint"},
            {"linear_scale_x": 1.0},
            {"linear_scale_y": 1.0},
            {"pub_odom_tf", True}
        ]
        # ,
        # remappings=[
        #     ("/sub_vel", "/vel_raw"),
        #     ("/pub_odom", "/odom_raw"),
        # ]
    )

    # launch description action sequence
    ld.add_action(base_robot_node)

    return ld
