#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

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

    # robot low-level speach driver node
    speach_driver_node = Node(
        package="ros2bot_drivers",
        executable="speach_driver_node",
        parameters=[
            {"xlinear_speed_limit": 1.0},
            {"ylinear_speed_limit": 1.0},
            {"angular_speed_limit": 5.0},
            {"imu_link": "imu_link"}
        ],
        remappings=[
            ("/pub_vel", "/vel_raw"),
            ("/pub_imu", "/imu/imu_raw"),
            ("/pub_mag", "/mag/mag_raw")
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher"
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="join_state_publisher"
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher"
    )

    # filter and fuse imu data
    imu_filter_node = Node(
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

    # extended Kalman data fusion
    localization_node = Node(
        package="robot_localization",
        executable="ekf_localization_node",
        parameters=[
            {"odom_frame": "/odom"},
            {"world_frame": "/odom"},
            {"base_link_frame": "/base_footprint"}
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

    # add nodes to launch description
    ld.add_action(base_robot_node)
    ld.add_action(speach_driver_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(imu_filter_node)
    ld.add_action(localization_node)
    ld.add_action(joy_control_launch)

    # return launch description
    return ld
