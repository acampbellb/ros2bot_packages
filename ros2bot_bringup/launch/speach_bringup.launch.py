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
    
    # include base robot launch
    base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_base"), '/launch', '/base_robot.launch.py'])
    )

    # include speach driver launch
    speach_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_controls"), '/launch', '/speach_driver.launch.py'])
    ) 

    # include robot localization launch
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/robot_localization.launch.py'])
    )  

    # included imu filter node launch
    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/imu_filter.launch.py'])
    )     

    # included joy control node launch
    joy_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_controls"), '/launch', '/joy_control.launch.py'])
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
        base_robot_launch,
        speach_driver_launch,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        imu_filter_launch,
        robot_localization_launch,
        joy_control_launch,
        rviz_node
    ])

    return ld
