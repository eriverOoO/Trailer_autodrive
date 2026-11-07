"""Read both registered IMUs with matching profiles; no motor or calibration nodes."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mvsim_slam_real_bringup.g6_launch import check_port, critical


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    ports = {role: value(role + '_imu_port') for role in ('tractor', 'trailer')}
    if os.path.realpath(ports['tractor']) == os.path.realpath(ports['trailer']):
        raise RuntimeError('Tractor and trailer IMUs need distinct physical devices')
    for role, port in ports.items():
        check_port(port, role + ' IMU')
    flag = value('publish_sensor_tf').lower()
    if flag not in ('true', 'false'):
        raise RuntimeError('publish_sensor_tf must be true or false')
    out = [SetEnvironmentVariable('PYTHONNOUSERSITE', '1'), LogInfo(msg=(
        'Matching IMU profiles; raw sensor frames. Trailer yaw +pi is in mount TF only. '
        'Absolute sensor-axis alignment remains unverified.'))]
    for role, port in ports.items():
        out += critical(Node(package='ebimu_driver', executable='ebimu_node',
                             name=role + '_imu', output='screen', parameters=[
                                 value('params_file'), {'port': port, 'enable_device': True}]))
    if flag == 'true':
        out += critical(Node(package='real_vehicle_bridge', executable='sensor_tf',
                             name='sensor_tf', output='screen',
                             parameters=[{'geometry_file': value('geometry_file')}]))
    return out


def generate_launch_description():
    config = Path(get_package_share_directory('mvsim_slam_real_bringup')) / 'config'
    defaults = {
        'tractor_imu_port': '/dev/imu_tractor',
        'trailer_imu_port': '/dev/imu_trailer',
        'params_file': str(config / 'imu.yaml'),
        'geometry_file': str(config / 'vehicle_geometry.yaml'),
        'publish_sensor_tf': 'true',
    }
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ] + [OpaqueFunction(function=nodes)])
