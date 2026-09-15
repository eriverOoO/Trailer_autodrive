#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory('simulation_pkg')
    gazebo_share = get_package_share_directory('gazebo_ros')
    world = os.path.join(simulation_share, 'worlds', 'track.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start Gazebo client in addition to gzserver.'),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Enable verbose Gazebo server logging.'),
        DeclareLaunchArgument(
            'autonomy', default_value='false',
            description='Start the unchanged perception/planning/control pipeline.'),
        gazebo,
        Node(
            package='simulation_pkg',
            executable='load_towing_system_node',
            output='screen'),
        Node(
            package='camera_perception_pkg', executable='yolov8_node',
            output='screen', condition=IfCondition(LaunchConfiguration('autonomy'))),
        Node(
            package='debug_pkg', executable='yolov8_visualizer_node',
            output='screen', condition=IfCondition(LaunchConfiguration('autonomy'))),
        Node(
            package='debug_pkg', executable='path_visualizer_node',
            output='screen', condition=IfCondition(LaunchConfiguration('autonomy'))),
        Node(
            package='camera_perception_pkg', executable='lane_info_extractor_node',
            output='screen', condition=IfCondition(LaunchConfiguration('autonomy'))),
        Node(
            package='decision_making_pkg', executable='path_planner_node',
            output='screen', condition=IfCondition(LaunchConfiguration('autonomy'))),
        Node(
            package='decision_making_pkg', executable='motion_planner_node',
            output='screen', condition=IfCondition(LaunchConfiguration('autonomy'))),
        Node(
            package='simulation_pkg', executable='sim_simulation_sender_node',
            output='screen', condition=IfCondition(LaunchConfiguration('autonomy'))),
    ])
