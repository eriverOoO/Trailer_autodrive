"""Stationary CPU perception capture. No serial sender, guard, or simulator."""
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from trailer_parking_pkg.replay import Platform


def generate_launch_description():
    def setup(context):
        value = lambda name: LaunchConfiguration(name).perform(context)
        paths = {name: Path(value(name)).expanduser().resolve()
                 for name in ('weights', 'calibration', 'profile')}
        for name, path in paths.items():
            if not path.is_file():
                raise RuntimeError(f'Missing {name}: {path}')
        profile = json.loads(paths['profile'].read_text(encoding='utf-8'))
        platform = Platform(profile)
        vision = dict(weights=str(paths['weights']), calibration=str(paths['calibration']),
                      device='cpu', use_sim_time=False,
                      geometry_override=ParameterValue(json.dumps(vars(platform.g)), value_type=str))
        actions = [Node(package='trailer_parking_pkg', executable='parallel_camera_yolo',
                        parameters=[{'capture_output': str(Path(value('output')).expanduser().resolve()),
                                     'sensor_timeout': float(value('sensor_timeout')),
                                     'use_sim_time': False}], output='screen')]
        for camera in ('front', 'rear'):
            actions.append(Node(package='trailer_parking_pkg', executable='parking_vision',
                name=f'parking_capture_{camera}', parameters=[vision, {'camera': camera}],
                remappings=[('image', value(f'{camera}_image')),
                            ('camera_info', value(f'{camera}_info')),
                            ('observation', f'/parking/{camera}/observation')], output='screen'))
        return actions

    arguments = [DeclareLaunchArgument(name) for name in ('weights', 'calibration', 'profile', 'output')]
    arguments.append(DeclareLaunchArgument('sensor_timeout', default_value='2.0'))
    for camera, topic in [('front', '/camera'), ('rear', '/rear_camera')]:
        arguments.extend([DeclareLaunchArgument(f'{camera}_image', default_value=f'{topic}/image_raw'),
                          DeclareLaunchArgument(f'{camera}_info', default_value=f'{topic}/camera_info')])
    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
