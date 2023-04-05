#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_path('ros2bot_urdf')
    urdf_path = pkg_path / 'urdf/ros2bot.urdf'
    rviz_config_path = pkg_path / 'rviz/model.rviz'

    gui_arg = DeclareLaunchArgument(name='use_gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    urdf_arg = DeclareLaunchArgument(name='urdf_model', default_value=str(urdf_path),
                                      description='Absolute path to robot urdf file')
    rviz_config_arg = DeclareLaunchArgument(name='rviz_config', default_value=str(rviz_config_path),
                                     description='Absolute path to rviz config file')  
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_model')]),
                                       value_type=str)  

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )     

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        gui_arg,
        urdf_arg,
        rviz_config_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])           