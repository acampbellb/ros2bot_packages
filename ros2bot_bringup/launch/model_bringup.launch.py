#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
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

    use_gui = DeclareLaunchArgument(name='use_gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    urdf_model = DeclareLaunchArgument(name='urdf_model', default_value=str(urdf_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_config = DeclareLaunchArgument(name='rviz_config', default_value=str(rviz_config_path),
                                     description='Absolute path to rviz config file')  
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_model')]),
                                       value_type=str)  

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # if use_gui parameter is true launch gui joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        name="joint_state_publisher_gui",
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )
    
    joint_state_publisher_node = Node(
        name="joint_state_publisher",
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )

    rviz_node = Node(
        name='rviz2',        
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    # launch description action sequence
    return LaunchDescription([
        use_gui,
        urdf_model,
        rviz_config,
        joint_state_publisher_gui_node,        
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])           