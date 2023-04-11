#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # launch description action sequence
    #ld.add_action(base_robot_node)

    return ld