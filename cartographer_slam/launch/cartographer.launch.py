#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    turtlebot3_cartographer_launch_dir =  os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch')

    # map_file = '~/demo_map.yaml'
    # cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    # configuration_basename = 'cartographer.lua'

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
        default=os.path.join(get_package_share_directory('cartographer_slam'), 'config'))

    configuration_basename = LaunchConfiguration('configuration_basename',
        default='cartographer.lua')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # map_dir = LaunchConfiguration(
    #     'map',
    #     default=os.path.join(os.path.expanduser('~'), 'turtlebot3_world_map.yaml'))

    # move_robot = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('multithreading_odom'), 'launch',
    #             'multithreading_odom_launch_file.launch.py'),
    #     )
    # )

    # how can I pass map_dir to navigation2.launch.py ?
    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot3_cartographer_launch_dir, '/cartographer.launch.py']),
        launch_arguments = {'cartographer_config_dir': cartographer_config_dir,
                            'configuration_basename': configuration_basename}.items(),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
            
        start_rviz,

    ]) 

# import os
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node

# def generate_launch_description():

#     cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
#     configuration_basename = 'cartographer.lua'

#     return LaunchDescription([
        
#         Node(
#             package='cartographer_ros', 
#             executable='cartographer_node', 
#             name='cartographer_node',
#             output='screen',
#             parameters=[{'use_sim_time': True}],
#             arguments=['-configuration_directory', cartographer_config_dir,
#                        '-configuration_basename', configuration_basename]),

#         Node(
#             package='cartographer_ros',
#             executable='occupancy_grid_node',
#             output='screen',
#             name='occupancy_grid_node',
#             parameters=[{'use_sim_time': True}],
#             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
#         ),
#     ]) 
