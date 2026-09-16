"""Front/rear yolov8_node -> parking_node -> simulation sender or Arduino serial sender.

Simulation:  ros2 launch decision_making_pkg parking.launch.py weights:=/abs/best.pt profile:=/abs/profile.json
Real car:    ros2 launch decision_making_pkg parking.launch.py sim:=false weights:=... profile:=... port:=/dev/ttyACM0
Teaching:    add record_only:=true (motor command stays 0, boxes on /parking_bboxes)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    arg = LaunchConfiguration
    sim = IfCondition(arg('sim'))
    use_sim_time = PythonExpression(["'", arg('sim'), "' == 'true'"])

    yolo = [Node(package='camera_perception_pkg', executable='yolov8_node', name=f'yolov8_{camera}',
                 output='screen',
                 parameters=[{'model': arg('weights'), 'device': arg('device'), 'threshold': 0.5,
                              # Gazebo cameras publish best-effort (2); real drivers usually reliable (1).
                              'image_reliability': PythonExpression(["2 if '", arg('sim'), "' == 'true' else 1"]),
                              'use_sim_time': use_sim_time}],
                 remappings=[('camera/image_raw', arg(f'{camera}_image')),
                             ('detections', f'detections_{camera}'),
                             ('enable', f'yolov8_{camera}/enable')])
            for camera in ('front', 'rear')]

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', description='true: Gazebo, false: Arduino'),
        DeclareLaunchArgument('weights', description='Absolute path to parking YOLO weights (best.pt)'),
        DeclareLaunchArgument('profile', default_value='', description='Absolute path to tuned stage JSON'),
        DeclareLaunchArgument('record_only', default_value='false'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('front_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('rear_image', default_value='/rear_camera/image_raw'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),

        # All models are local; waiting on the online model database delays vehicle cameras.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '', condition=sim),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('simulation_pkg'), 'launch', 'trailer_sim.launch.py')),
            launch_arguments={'gui': arg('gui')}.items(), condition=sim),
        Node(package='simulation_pkg', executable='sim_simulation_sender_node', output='screen',
             parameters=[{'use_sim_time': True}], condition=sim),
        Node(package='serial_communication_pkg', executable='serial_sender_node', output='screen',
             parameters=[{'port': arg('port'), 'baud': PythonExpression([arg('baud')])}],
             condition=UnlessCondition(arg('sim'))),
        *yolo,
        Node(package='decision_making_pkg', executable='parking_node', output='screen',
             parameters=[{'profile': arg('profile'), 'record_only': PythonExpression(["'", arg('record_only'), "' == 'true'"]),
                          'use_sim_time': use_sim_time}]),
    ])
