"""Read the connected 11-field EBIMU into sensor_msgs/Imu without configuration writes."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mvsim_slam_real_bringup.g6_launch import check_port, critical


def nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    port = LaunchConfiguration('imu_port').perform(context)
    if not port:
        port = yaml.safe_load(Path(params_file).read_text())['tractor_imu']['ros__parameters']['port']
    check_port(port, 'IMU')
    return [SetEnvironmentVariable('PYTHONNOUSERSITE', '1')] + critical(Node(
        package='ebimu_driver', executable='ebimu_node', name='tractor_imu',
        output='screen', parameters=[params_file, {'port': port, 'enable_device': True}]))


def generate_launch_description():
    share = Path(get_package_share_directory('mvsim_slam_real_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('imu_port', default_value='', description='Empty uses stable port from params_file'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/ebimu_slam.yaml')),
        OpaqueFunction(function=nodes),
    ])
