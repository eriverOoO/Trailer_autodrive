"""Explicit TEST/SIMULATED preview; never starts real serial, SLAM or control."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('tractor_trailer_rviz_2d'))
    return LaunchDescription([
        DeclareLaunchArgument('output_directory',default_value=''),
        Node(package='tractor_trailer_rviz_2d',executable='tt2d_visualizer',output='screen',
             parameters=[{'config_file':str(share/'config/test.yaml'),
                          'output_directory':LaunchConfiguration('output_directory')}]),
        Node(package='tractor_trailer_rviz_2d',executable='tt2d_test_source',output='screen',parameters=[{'confirm_test':True}]),
        Node(package='rviz2',executable='rviz2',name='vehicle_viz_2d_TEST_rviz',
             arguments=['-d',str(share/'rviz/test_preview.rviz')],output='screen')])
