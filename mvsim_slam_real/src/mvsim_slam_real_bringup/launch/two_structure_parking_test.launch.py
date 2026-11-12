"""Separate two-structure test; automatic search is started by a service command."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def base_launch():
    path = Path(get_package_share_directory('mvsim_slam_real_bringup'))/'launch/real_parking.launch.py'
    spec = importlib.util.spec_from_file_location('_two_structure_base_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def nodes(context):
    base = base_launch()
    value = lambda name: LaunchConfiguration(name).perform(context)
    if base.enabled(context, 'enable_vision'):
        raise RuntimeError('Use this test camera_path/image_topic, not the legacy enable_vision launch')
    if value('camera_path') and base.enabled(context, 'development_mode'):
        raise RuntimeError('Opening a camera device requires development_mode:=false; '
                           'recorded image_topic works in development mode')
    parameters = [value('test_params_file'), {
        'manual_survey': ParameterValue(LaunchConfiguration('manual_survey'), value_type=bool),
        'require_camera': ParameterValue(LaunchConfiguration('require_camera'), value_type=bool),
    }]
    if value('slot_width'):
        parameters.append({'slot_width': float(value('slot_width'))})
    if base.enabled(context, 'start_stack'):
        actions = base.launch_nodes(context, parking_executable='two_structure_parking_test',
                                    parking_parameters=parameters)
    else:
        share = Path(get_package_share_directory('mvsim_slam_real_bringup'))
        actions = [Node(package='real_parking', executable='two_structure_parking_test',
                        parameters=[str(share/'config/parking.yaml'), *parameters,
                                    {'world_file': value('geometry_file'),
                                     'blocked_debug_directory': value('output_directory')}],
                        output='screen')]
    if base.enabled(context, 'start_g6_driver'):
        if base.enabled(context, 'development_mode'):
            raise RuntimeError('G6 hardware requires development_mode:=false')
        if not base.enabled(context, 'start_stack'):
            raise RuntimeError('start_stack:=false uses the already running /scan driver')
        if value('lidar_driver') != 'external':
            raise RuntimeError('start_g6_driver uses lidar_driver:=external and /lidar_raw gateway')
        port = value('lidar_port')
        if not port or port in [value(name) for name in ('tractor_imu_port', 'trailer_imu_port', 'arduino_port')]:
            raise RuntimeError('G6 needs an explicit port distinct from IMU and Arduino ports')
        share = Path(get_package_share_directory('mvsim_slam_real_bringup'))
        actions.append(GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/g6_lidar.launch.py')),
            launch_arguments={'lidar_port': port, 'scan_topic': '/lidar_raw',
                              'enable_rviz': 'false'}.items())]))
    share = Path(get_package_share_directory('mvsim_slam_real_bringup'))
    actions.append(GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch/parking_vision_diagnostic.launch.py')),
        launch_arguments={name: value(name) for name in
                          ('camera_path', 'image_topic', 'calibration_file', 'detector_params')}.items())]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('mvsim_slam_real_bringup'))
    # Reuse standard port/geometry/actuation arguments and all their hardware guards.
    arguments = [action for action in base_launch().generate_launch_description().entities
                 if isinstance(action, DeclareLaunchArgument)]
    arguments += [DeclareLaunchArgument(name, default_value=default) for name, default in {
        'test_params_file': str(share/'config/two_structure_test.yaml'),
        'slot_width': '',  # Empty retains the YAML depth; optional launch override in metres.
        'manual_survey': 'false',
        'require_camera': 'true',
        'start_stack': 'true',
        'start_g6_driver': 'false',
        'image_topic': '/parking_vision/image_raw',
        'calibration_file': '',
        'detector_params': '',
    }.items()]
    return LaunchDescription(arguments+[OpaqueFunction(function=nodes)])
