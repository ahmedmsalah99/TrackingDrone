#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch argument for mode
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Launch mode: sim (simulation) or real (real hardware)',
        choices=['sim', 'real']
    )

    # Get the mode configuration
    mode = LaunchConfiguration('mode')

    # Find package shares
    detect_and_track_bringup_share = FindPackageShare('detect_and_track_bringup')
    stream_manager_share = FindPackageShare('stream_manager')
    video_manager_share = FindPackageShare('video_manager')

    stream_manager_filename = PythonExpression([
        "'stream_manager_' + str('", mode, "') + '_params.yaml'"
    ])
    
    # Define parameter files based on mode
    stream_manager_params = PathJoinSubstitution([
        detect_and_track_bringup_share,
        'config',
        stream_manager_filename
    ])

    detection_and_mot_params = PathJoinSubstitution([
        detect_and_track_bringup_share,
        'config',
        'detection_and_mot_params.yaml'
    ])

    video_manager_params = PathJoinSubstitution([
        detect_and_track_bringup_share,
        'config',
        'video_manager_params.yaml'
    ])

    moc_video_publisher_params = PathJoinSubstitution([
        detect_and_track_bringup_share,
        'config',
        'moc_video_publisher_params.yaml'
    ])

    # Stream Manager Node
    stream_manager_node = Node(
        package='stream_manager',
        executable='stream_manager_node',
        name='stream_manager',
        output='screen',
        parameters=[stream_manager_params]
    )

    # Detection and MOT Node
    detection_and_mot_node = Node(
        package='detection_and_mot',
        executable='detection_and_mot_node',
        name='detection_and_mot',
        output='screen',
        parameters=[detection_and_mot_params]
    )

    # Video Manager Node
    video_manager_node = Node(
        package='video_manager',
        executable='video_manager_node',
        name='video_manager',
        output='screen',
        parameters=[video_manager_params]
    )
    # Moc Video Publisher Node
    moc_video_pub_node = Node(
        package='moc_video_publisher',
        executable='moc_video_publisher_node',
        name='moc_video_publisher',
        output='screen',
        parameters=[moc_video_publisher_params],
        condition=IfCondition(
        PythonExpression(["'", LaunchConfiguration('mode'), "' == 'sim'"])
    )
    )

    return LaunchDescription([
        mode_arg,
        stream_manager_node,
        detection_and_mot_node,
        video_manager_node,
        # moc_video_pub_node
    ])
