"""G6 sensing and SLAM only; no vehicle control processes."""
import math
import os
from pathlib import Path
import stat
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def value(context, name):
    return LaunchConfiguration(name).perform(context)


def boolean(context, name):
    text = value(context, name).lower()
    if text not in ('true', 'false'):
        raise RuntimeError(f'{name} must be true or false')
    return text == 'true'


def measured_mount(path):
    mount = yaml.safe_load(Path(path).read_text())
    if mount.get('measured') is not True:
        raise RuntimeError('Vehicle SLAM requires measured: true in mount_file; '
                           'use frame_mode:=sensor until extrinsics are measured')
    if (mount.get('parent_frame'), mount.get('child_frame')) != (
            'base_footprint', 'laser_frame'):
        raise RuntimeError('Mount frames must be base_footprint -> laser_frame')
    for key in ('xyz', 'rpy'):
        values = mount.get(key)
        if not isinstance(values, list) or len(values) != 3 or not all(
                type(v) in (int, float) and math.isfinite(v) for v in values):
            raise RuntimeError(f'Mount {key} needs three measured finite numbers')
    return mount


def check_port(port):
    if not port:
        raise RuntimeError('Set lidar_port explicitly, preferably /dev/serial/by-id/...')
    path = Path(port)
    if not path.exists() or not stat.S_ISCHR(path.stat().st_mode):
        raise RuntimeError(f'LiDAR serial character device not found: {port}')
    if not os.access(path, os.R_OK | os.W_OK):
        raise RuntimeError(f'No read/write access to {port}; check dialout membership')
    # Read-only check: never steal a serial port from another process.
    result = subprocess.run(['fuser', str(path.resolve())], capture_output=True,
                            text=True, check=False)
    if result.returncode == 0:
        raise RuntimeError(f'LiDAR port is already occupied (PID {result.stdout.strip()})')
    if result.returncode != 1:
        raise RuntimeError(f'Could not check LiDAR port ownership: {result.stderr}')


def critical(node):
    return [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[Shutdown(reason='G6 sensing/SLAM process exited')])), node]


def nodes(context, slam):
    share = Path(get_package_share_directory('mvsim_slam_real_bringup'))
    config = share / 'config'
    out = [SetEnvironmentVariable('PYTHONNOUSERSITE', '1')]
    mode = value(context, 'frame_mode') if slam else 'sensor'
    if mode not in ('sensor', 'vehicle'):
        raise RuntimeError('frame_mode must be sensor or vehicle')
    use_imu = boolean(context, 'use_imu') if slam else False
    if use_imu and mode != 'vehicle':
        raise RuntimeError('IMU mode requires vehicle frames and measured IMU extrinsics')
    if slam and mode == 'vehicle' and boolean(context, 'publish_mount_tf'):
        mount = measured_mount(value(context, 'mount_file'))
        xyz, rpy = mount['xyz'], mount['rpy']
        out += [Node(package='tf2_ros', executable='static_transform_publisher',
                     name='g6_mount_tf', arguments=[
                         '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                         '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                         '--frame-id', 'base_footprint', '--child-frame-id', 'laser_frame']),
                Node(package='tf2_ros', executable='static_transform_publisher',
                     name='g6_base_link_tf', arguments=[
                         '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'])]
    if boolean(context, 'start_driver'):
        port = value(context, 'lidar_port')
        check_port(port)
        driver = Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                      name='ydlidar_ros2_driver_node', output='screen',
                      parameters=[value(context, 'params_file'), {'port': port}],
                      remappings=[('scan', value(context, 'scan_topic'))])
        out += critical(driver)
    if slam:
        lua = ('g6_imu_optional.lua' if use_imu else
               ('g6_vehicle.lua' if mode == 'vehicle' else 'g6_lidar_only.lua'))
        message = ('Sensor-origin SLAM: map -> odom -> laser_frame; vehicle pose is unmeasured'
                   if mode == 'sensor' else
                   'Vehicle SLAM: run only one Cartographer and one publisher per TF edge')
        out.append(LogInfo(msg=message))
        if mode == 'vehicle' and not boolean(context, 'publish_mount_tf'):
            out.append(LogInfo(msg='Using EXTERNAL measured sensor TF; do not run duplicate publishers'))
        if use_imu:
            out.append(LogInfo(msg='IMU fusion requires externally verified /tractor/imu/data '
                               'with measured gravity acceleration and tractor_imu_link TF'))
        cartographer = Node(
            package='cartographer_ros', executable='cartographer_node',
            name='cartographer_node', output='screen',
            parameters=[{'use_sim_time': False}], arguments=[
                '-configuration_directory', str(config), '-configuration_basename', lua],
            remappings=[('scan', value(context, 'scan_topic')), ('imu', '/tractor/imu/data')])
        out += critical(cartographer)
        grid = Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                    name='cartographer_occupancy_grid_node', output='screen',
                    parameters=[{'use_sim_time': False}],
                    arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])
        out += critical(grid)
    if boolean(context, 'enable_rviz'):
        out.append(Node(package='rviz2', executable='rviz2', name='g6_rviz', output='screen',
                        arguments=['-d', value(context, 'rviz_config'),
                                   '-f', 'map' if slam else 'laser_frame'],
                        parameters=[{'use_sim_time': False}]))
    return out


def description(slam=False):
    share = Path(get_package_share_directory('mvsim_slam_real_bringup'))
    defaults = {
        'lidar_port': ('', 'Explicit G6 port; never inferred from ttyUSB numbering'),
        'params_file': (str(share / 'config/g6.yaml'), 'G6 driver ROS parameters'),
        'scan_topic': ('/scan', 'Driver output and Cartographer LaserScan input'),
        'enable_rviz': ('false', 'Display actual scans and map in RViz2'),
        'rviz_config': (str(share / 'rviz/g6.rviz'), 'RViz display and camera configuration'),
        'start_driver': ('true', 'false subscribes to an already running driver'),
    }
    if slam:
        defaults.update({
            'frame_mode': ('sensor', 'sensor: LiDAR-origin test; vehicle: measured extrinsics'),
            'mount_file': (str(share / 'config/g6_mount.yaml'), 'Measured LiDAR extrinsics'),
            'publish_mount_tf': ('true', 'vehicle only; false uses existing external TF'),
            'use_imu': ('false', 'Future fusion; requires externally verified IMU and TF'),
        })
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=doc)
        for name, (default, doc) in defaults.items()
    ] + [OpaqueFunction(function=lambda context: nodes(context, slam))])
