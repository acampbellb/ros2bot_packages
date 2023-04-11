#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    imu_calib_file = os.path.join(
        get_package_share_directory('ros2bot_imu_calib'),
        'config',
        'imu_calib.yaml'        
    )

    # imu calibration node
    imu_calib_node = Node(
        name="imu_do_calib",
        package="ros2bot_imu_calib",
        executable="do_calib",
        output="screen",
        parameters=[
            {"output_file": str(imu_calib_file)}
        ],
        remappings=[
            ("/sub_imu", "/imu/imu_raw"),
        ]        
    )      

    # add nodes to launch description
    ld = LaunchDescription([
        imu_calib_node
    ])

    # return launch description
    return ld      