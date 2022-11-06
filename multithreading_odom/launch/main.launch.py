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

    turtlebot3_launch_dir =  os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')

    # map_file = '~/demo_map.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(os.path.expanduser('~'), 'turtlebot3_world_map.yaml'))

    move_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('multithreading_odom'), 'launch',
                'multithreading_odom_launch_file.launch.py'),
        )
    )

    # how can I pass map_dir to navigation2.launch.py ?
    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot3_launch_dir, '/navigation2.launch.py']),
        launch_arguments = {
            'map': map_dir,
            'use_sim_time': use_sim_time,}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        move_robot,
        start_rviz,

    ])
