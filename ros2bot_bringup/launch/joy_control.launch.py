#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # robot joy control node
    joy_control_node = Node(
        package="ros2bot_controls",
        executable="joy_node",
        name="joy_control_node",
        parameters=[
            {"linear_speed_limit" : 1.0},
            {"angular_speed_limit" : 5.0}
        ]
    )

    # joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node"
    )

    # add nodes to launch description
    ld.add_action(joy_control_node)
    ld.add_action(joy_node)

    # return launch description
    return ld