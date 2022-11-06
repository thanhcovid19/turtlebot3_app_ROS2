import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    move_robot_node = Node(
        package='multithreading_odom',
        executable='move_robot_zigzag',
        output='screen',
        name='move_robot_node',
        parameters=[{'use_sim_time': True}])

    # create and return launch description object
    return LaunchDescription(
        [
            move_robot_node
        ]
    )
