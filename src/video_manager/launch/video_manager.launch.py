#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='VideoManager',
            executable='video_manager_node',
            name='video_manager',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        )
    ])