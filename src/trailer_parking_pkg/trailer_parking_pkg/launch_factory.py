"""One mutually exclusive command source per launch; front/rear cameras only."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def parking_launch(use_lidar):
    def setup(context):
        value = lambda key: LaunchConfiguration(key).perform(context)
        weights = str(Path(value('weights')).expanduser().resolve())
        if not Path(weights).is_file():
            raise RuntimeError('Supply weights:=/absolute/path/to/parking/best.pt')
        start = int(value('start_index'))
        if start not in range(1, 5):
            raise RuntimeError('start_index must be 1..4 for calibrated visual initialization')
        sim = Path(get_package_share_directory('simulation_pkg'))
        # Package-local loader is installed as Python source, including symlink install.
        import importlib.util
        loader = importlib.util.find_spec('simulation_pkg.lib.load_towing_system_node').origin
        vision_params = dict(weights=weights, device=value('device'), start_index=start,
                             model_sdf=str(sim/'models/prius_hybrid/model.sdf'), loader_source=loader,
                             calibration=value('calibration'), use_sim_time=True)
        plugin_lib = str(Path(get_package_prefix('simulation_trailer_plugins'))/'lib')
        gazebo = Path(get_package_share_directory('gazebo_ros'))/'launch/gazebo.launch.py'
        actions = [
            SetEnvironmentVariable('GAZEBO_MODEL_PATH', [str(sim/'models'), ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]),
            SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', [plugin_lib, ':', EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo)),
                launch_arguments={'world': str(sim/'worlds/track.world'), 'gui': value('gui')}.items()),
            Node(package='simulation_pkg', executable='load_towing_system_node', output='screen',
                 parameters=[{'start_index': start, 'parking_lidar': use_lidar, 'use_sim_time': True}]),
            Node(package='trailer_parking_pkg', executable='parking_command_guard', output='screen'),
            Node(package='trailer_parking_pkg',
                 executable='parallel_lidar_yolo' if use_lidar else 'parallel_camera_yolo',
                 parameters=[{'use_sim_time': True, 'parking_speed': float(value('speed')),
                              'sensor_timeout': float(value('sensor_timeout'))}], output='screen')]
        for camera, topic in [('front', 'camera'), ('rear', 'rear_camera')]:
            actions.append(Node(package='trailer_parking_pkg', executable='parking_vision',
                name=f'parking_{camera}_vision', parameters=[vision_params, {'camera': camera}],
                remappings=[('image', value(f'{camera}_image')),
                            ('camera_info', value(f'{camera}_info')),
                            ('observation', f'/parking/{camera}/observation')], output='screen'))
        return actions

    return LaunchDescription([
        DeclareLaunchArgument('weights', description='Existing parking-space/car-front/car-back YOLO segmentation weights'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('start_index', default_value='1'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('speed', default_value='0.30'),
        DeclareLaunchArgument('sensor_timeout', default_value='0.8'),
        DeclareLaunchArgument('calibration', default_value=''),
        DeclareLaunchArgument('front_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('front_info', default_value='/camera/camera_info'),
        DeclareLaunchArgument('rear_image', default_value='/rear_camera/image_raw'),
        DeclareLaunchArgument('rear_info', default_value='/rear_camera/camera_info'),
        OpaqueFunction(function=setup)])
