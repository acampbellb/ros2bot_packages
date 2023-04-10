#!/usr/bin/env python3

import os

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # include gazebo empty world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), '/launch', '/empty_world.launch.py'])
    ) 

    # robot odometry publisher
    tf_footprint_base = Node(
        name="tf_footprint_base",
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screnn",
        arguments=[
            '0',
            '0',
            '0',
            '0',
            '0',
            '0',
            'base_link',
            'base_footptint'
        ]
    )   

    urdf_model =  os.path.join(
        get_package_share_directory('ros2bot_urdf'),
        'urdf',
        'ros2bot.urdf'
    )

    spawn_model = Node(
        name="spawn_model",
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            '-entity','ros2bot',
            '-x', '0',
            '-y', '0',
            '-z', '0',
            '-file', urdf_model
        ]
    )

    # joint calibaration 
    fake_joint_calibration = Node(
        name="fake_joint_calibration",
        package="ros2topic",
        executable="ros2topic",
        arguments=[
            "pub -r 10 /calibrated std_msgs/msg/Bool '{data:true}'"
        ]
    ) 

    ld = LaunchDescription([
        gazebo_launch,
        tf_footprint_base,
        spawn_model,
        fake_joint_calibration
    ])

    return ld    