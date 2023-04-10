#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_model_path = os.path.join(
        get_package_share_directory('ros2bot_urdf'),
        'urdf',
        'ros2bot.urdf'
    ) 

    rviz_config_path = os.path.join(
        get_package_share_directory('ros2bot_bringup'),
        'config',
        'robot.rviz'
    )

    use_gui = DeclareLaunchArgument(name='use_gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable GUI')    
    use_rviz = DeclareLaunchArgument(name='use_rviz', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable GUI')
    urdf_model = DeclareLaunchArgument(name='urdf_model', default_value=str(urdf_model_path),
                                      description='Absolute path to robot urdf file')  
    rviz_config = DeclareLaunchArgument(name='rviz_config', default_value=str(rviz_config_path),
                                     description='Absolute path to rviz config file')      

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_model')]),
                                       value_type=str)  
    
    # include speach driver launch
    speach_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/speach_driver.launch.py'])
    ) 

    # robot odometry publisher
    base_robot_node = Node(
        name="ros2bot_base",
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

    # if use_gui parameter is true launch gui joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        name="joint_state_publisher_gui",        
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )    

    joint_state_publisher_node = Node(
        name="robot_state_publisher",        
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration('use_gui')) 
    )    

    robot_state_publisher_node = Node(
        name="robot_state_publisher",        
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # filter and fuse imu data
    imu_filter_node = Node(
        name="imu_filter",
        package="imu_filter_madgwick",
        executable="imu_filter_node",
        parameters=[
            {"fixed_frame": "base_link"},
            {"use_mag": False},
            {"publish_tf": False},
            {"use_magnetic_field_msg": False},
            {"world_frame": "enu"},
            {"orientation_stddev": 0.05},
            {"angular_scale": 1.05},
        ],
        remapping=[
            ("/sub_imu", "/imu/imu_raw"),
            ("/sub_mag", "/mag/mag_raw"),
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
        name="robot_localization",
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

    # included joy control node launch
    joy_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/joy_control.launch.py'])
    )

    rviz_node = Node(
        name='rviz2',        
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_reviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )    

    # launch description action sequence
    ld = LaunchDescription([
        use_gui,
        use_rviz,
        urdf_model,
        rviz_config,
        base_robot_node,
        speach_driver_launch,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        imu_filter_node,
        localization_node,
        joy_control_launch,
        rviz_node
    ])

    return ld
