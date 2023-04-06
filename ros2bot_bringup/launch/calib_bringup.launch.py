#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    pkg_path = get_package_share_path('ros2bot_urdf')
    model_path = pkg_path / 'urdf/ros2bot.urdf' 
    rviz_config_path = pkg_path / 'rviz/model.rviz'

    gui_arg = DeclareLaunchArgument(name='use_gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable GUI')    
    rviz_arg = DeclareLaunchArgument(name='use_rviz', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable GUI')
    urdf_arg = DeclareLaunchArgument(name='urdf_model', default_value=str(model_path),
                                      description='Absolute path to robot urdf file')  
    rviz_config_arg = DeclareLaunchArgument(name='rviz_config', default_value=str(rviz_config_path),
                                     description='Absolute path to rviz config file')      

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_model')]),
                                       value_type=str)    
  

    # robot odometry publisher
    base_robot_node = Node(
        package="ros2bot_base",
        executable="ros2bot_base_node",
        parameters=[
            {"odom_frame": "odom"},
            {"base_footprint_frame": "base_footprint"},
            {"linear_scale_x": 1.1},
            {"linear_scale_y": 1.0}
        ],
        remappings=[
            ("/sub_vel", "/vel_raw"),
            ("/pub_odom", "/odom_raw"),
        ]
    )

    # robot low-level driver node
    master_driver_node = Node(
        package="ros2bot_drivers",
        executable="master_driver_node",
        output="screen",
        parameters=[
            {"xlinear_limit" : 1.0},
            {"ylinear_limit" : 1.0},
            {"angular_limit" : 5.0},
            {"imu_link" : "imu_link"}
        ],
        remappings=[
            ("/pub_vel", "/imu/vel_raw"),
            ("/pub_imu", "/imu/imu_raw"),    
            ("/pub_mag", "/mag/mag_raw")        
        ]
    )

    # filter and fuse imu data
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_node",
        output="screen",
        parameters=[
            {"fixed_frame": "base_link"},
            {"use_mag": True},
            {"publish_tf": False},
            {"use_magnetic_field_msg": True},
            {"world_frame": "enu"},
            {"orientation_stddev": 0.05},
            {"angular_scale": 1.08},
        ],
        remapping=[
            ("/sub_imu", "/imu/imu_calib"),
            ("/sub_mag", "/mag/mag_calib"),
            ("/pub_imu", "/imu/imu_data"),
            ("/pub_mag", "/mag/mag_field"),
        ]
    )   

    # extended kalman data fusion    
    localization_config = os.path.join(
        get_package_share_directory('ros2bot_bringup'),
        'config',
        'robot_localization.yaml'
        )

    localization_node = Node(
        package="robot_localization",
        executable="ekf_localization_node",
        output="screen",
        parameters=[
            {"odom_frame": "/odom"},
            {"world_frame": "/odom"},
            {"base_link_frame": "/base_footprint"},
            localization_config
        ],
        remapping=[
            ("odometry/filtered", "odom"),
            ("/imu0", "/imu/imu_data"),
            ("/odom0", "odom_raw"),
        ]
    )   

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="join_state_publisher",
        condition=UnlessCondition(LaunchConfiguration('use_gui'))     
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )    

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )      

    # included joy control node launch
    joy_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/joy_control.launch.py'])
    )    

    # add nodes to launch description
    ld = LaunchDescription([
        gui_arg,
        rviz_arg,
        urdf_arg,
        rviz_config_arg,
        base_robot_node,
        master_driver_node,
        imu_filter_node,
        localization_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        joy_control_launch
    ])

    # return launch description
    return ld    