from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    c=Path(get_package_share_directory('mvsim_slam_real_bringup'))/'config'
    return LaunchDescription([Node(package='cartographer_ros',executable='cartographer_node',name='cartographer_node',arguments=['-configuration_directory',str(c),'-configuration_basename','reference_lidar_only.lua'],output='screen'),Node(package='cartographer_ros',executable='cartographer_occupancy_grid_node',arguments=['-resolution','0.05','-publish_period_sec','1.0'],output='screen')])
