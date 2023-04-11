#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('ros2bot_calibrate'),
        'config',
        'calibrate_linear_params.yaml'
        )  
    
    calibrate_linear_node = Node(
        name="calibrate_linear_node",
        package="ros2bot_calibrate",
        executable="calibrate_linear.py",
        parameters = [config]
    )

    # launch description action sequence
    ld.add_action(calibrate_linear_node)

    return ld