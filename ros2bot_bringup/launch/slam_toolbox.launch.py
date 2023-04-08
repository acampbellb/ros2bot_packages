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

    rviz_arg = DeclareLaunchArgument(name='use_rviz', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable GUI')
    localization_arg = DeclareLaunchArgument(name='localization_mode', default_value='false', choices=['true', 'false'],
                                            description='Flag to start slam in localization mode')  
    sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                                        description='Flag to use simulation / gazebo clock') 
    rviz_config_path = get_package_share_directory("ros2bot_slam") + '/config/slam_toolbox.yaml'
    rviz_config_arg = DeclareLaunchArgument(name='rviz_config', default_value=str(rviz_config_path),
                                            description='Absolute path to rviz config file') 
    
    # include master bringup launch
    master_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/master_bringup.launch.py'])
    )   

    # include lidar node launch
    lidar_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros2bot_bringup"), '/launch', '/lidar_node.launch.py'])
    )  

    # slam toolbox mapping node
    slam_toolbox_mapping_node = Node(
        name='slam_toolbox_mapping',        
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('localization_mode')),   
        parameters=[
        	get_package_share_directory("ros2bot_slam") + '/config/mapper_params_online_async.yaml'
        ]
    )

    # slam toolbox localization node
    slam_toolbox_localization_node = Node(
        name='slam_toolbox_localization',        
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('localization_mode')),
        parameters=[
        	get_package_share_directory("ros2bot_slam") + '/config/mapper_params_localization.yaml',
            {'use_sim_time' : LaunchConfiguration('use_sim_time')}
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

    ld = LaunchDescription([
        rviz_arg,
        localization_arg,
        sim_time_arg,
        rviz_config_arg,
        master_bringup_launch,
        lidar_node_launch,
        slam_toolbox_mapping_node,
        slam_toolbox_localization_node,
        rviz_node
    ])

    return ld