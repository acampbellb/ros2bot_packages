#!/usr/bin/env python3

from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    package_path = get_package_share_path('ros2bot_urdf')   
    model_path = package_path / 'urdf/ros2bot.urdf'
    rviz_config_path = package_path / 'config/model.rviz'     

    gui_arg = DeclareLaunchArgument(name='use_gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')    
    rviz_config_arg = DeclareLaunchArgument(name='rviz_config', default_value=str(rviz_config_path),
                                     description='Absolute path to rviz config file')     
    urdf_arg = DeclareLaunchArgument(name='urdf_model', default_value=str(model_path),
                                      description='Absolute path to robot urdf file')      

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_model')]),
                                       value_type=str)   

    joint_state_publisher_node = Node(
        name="join_state_publisher",
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration('use_gui'))         
    )

    joint_state_publisher_gui_node = Node(
        name="joint_state_publisher_gui",
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )  

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )           
    
    rviz_node = Node(
        name="rviz2",
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )     

    ld = LaunchDescription([
        gui_arg,
        rviz_config_arg,
        urdf_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])

    return ld        