#!/usr/bin/env python3

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
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

    # joint calibaration 
    fake_joint_calibration = Node(
        name="fake_joint_calibration",
        package="ros2topic",
        executable="ros2topic",
        arguments=[
            'pub -r 10 /calibrated std_msgs/msg/Bool "{data: true}"'        
        ]
    ) 

    ld = LaunchDescription([
        gazebo_launch,
        tf_footprint_base,
        fake_joint_calibration
    ])

    return ld    