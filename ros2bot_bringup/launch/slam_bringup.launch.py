#!/usr/bin/env python3

import os

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config_path = os.path.join(
        get_package_share_directory('ros2bot_slam'),
        'config',
        'slam_toolbox.rviz'
    ) 

    online_async_config_path = os.path.join(
        get_package_share_directory('ros2bot_slam'),
        'config',
        'mapper_params_online_async.yaml'
    )    

    localization_config_path = os.path.join(
        get_package_share_directory('ros2bot_slam'),
        'config',
        'mapper_params_localization.yanl'
    )       

    use_rviz = DeclareLaunchArgument(name='use_rviz', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable GUI')
    rviz_config = DeclareLaunchArgument(name='rviz_config', default_value=str(rviz_config_path),
                                        description='Absolute path to rviz config file')     
    localization_mode = DeclareLaunchArgument(name='localization_mode', default_value='false', choices=['true', 'false'],
                                            description='Flag to start slam in localization mode')  
    use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                                        description='Flag to use simulation / gazebo clock')     
    
    # include master bringup launch
    master_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/master_bringup.launch.py'])
    )   

    # include lidar node launch
    lidar_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_lidar"), '/launch', '/lidar_node.launch.py'])
    )  

    # slam toolbox mapping node
    slam_toolbox_mapping_node = Node(
        name='slam_toolbox_mapping',        
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('localization_mode')),   
        parameters=[
        	online_async_config_path,
            {'use_sim_time':LaunchConfiguration('use_sim_time')}
        ]
    )

    # slam toolbox localization node
    slam_toolbox_localization_node = Node(
        name='slam_toolbox_localization',        
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('localization_mode')),
        parameters=[
        	localization_config_path,
            {'use_sim_time':LaunchConfiguration('use_sim_time')}
        ]
    )

    # rviz node
    rviz_node = Node(
        name='rviz2',        
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )  

    # launch description action sequence
    ld = LaunchDescription([
        use_rviz,
        localization_mode,
        use_sim_time,
        rviz_config,
        master_bringup_launch,
        lidar_node_launch,
        slam_toolbox_mapping_node,
        slam_toolbox_localization_node,
        rviz_node
    ])

    return ld