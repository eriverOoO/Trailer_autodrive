"""Observe externally running SLAM. Starts no serial driver or control node."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('mvsim_slam_real_bringup'))/'config/wallless.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('output_file', default_value=''),
        DeclareLaunchArgument('ego_frame', default_value='base_footprint'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='real_parking', executable='wallless_validation', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'output_file': LaunchConfiguration('output_file'),
                          'ego_frame': LaunchConfiguration('ego_frame'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}])])
