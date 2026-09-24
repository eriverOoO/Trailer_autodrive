"""One mutually exclusive command source per launch; front/rear cameras only."""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from .core import Geometry


def parking_launch(use_lidar):
    def setup(context):
        value = lambda key: LaunchConfiguration(key).perform(context)
        weights = str(Path(value('weights')).expanduser().resolve())
        if not Path(weights).is_file():
            raise RuntimeError('Supply weights:=/absolute/path/to/parking/best.pt')
        start = int(value('start_index'))
        if start not in range(1, 5):
            raise RuntimeError('start_index must be 1..4 for calibrated visual initialization')
        speed, parking_pwm, steer_steps = (float(value('speed')),
            int(value('parking_pwm')), int(value('steer_steps')))
        steering_sign = int(value('steering_sign'))
        if (speed <= 0 or not 1 <= parking_pwm <= 255 or
                not 1 <= steer_steps <= 7 or steering_sign not in (-1, 1)):
            raise RuntimeError('speed>0, parking_pwm=1..255, steer_steps=1..7 and steering_sign=±1 required')
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
            Node(package='simulation_pkg', executable='sim_simulation_sender_node', output='screen',
                 parameters=[{'max_speed': speed * 255.0 / parking_pwm,
                              'max_steer': 0.60}]),
            Node(package='trailer_parking_pkg',
                 executable='parallel_lidar_yolo' if use_lidar else 'parallel_camera_yolo',
                 parameters=[{'use_sim_time': True, 'parking_speed': speed,
                              'parking_pwm': parking_pwm,
                              'steer_steps': steer_steps,
                              'steering_sign': steering_sign,
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
        DeclareLaunchArgument('parking_pwm', default_value='90'),
        DeclareLaunchArgument('steer_steps', default_value='7'),
        DeclareLaunchArgument('steering_sign', default_value='-1'),
        DeclareLaunchArgument('sensor_timeout', default_value='0.8'),
        DeclareLaunchArgument('calibration', default_value=''),
        DeclareLaunchArgument('front_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('front_info', default_value='/camera/camera_info'),
        DeclareLaunchArgument('rear_image', default_value='/rear_camera/image_raw'),
        DeclareLaunchArgument('rear_info', default_value='/rear_camera/camera_info'),
        OpaqueFunction(function=setup)])


def hardware_parking_launch(use_lidar):
    """Real stroller: cameras/LiDAR -> planner -> MotionCommand -> Arduino."""
    def measured_scan_offset(value):
        try:
            offset = [float(item) for item in json.loads(value)]
        except (TypeError, ValueError) as exc:
            raise RuntimeError('Scan offset must be [x,y,yaw]') from exc
        if len(offset) != 3 or not all(math.isfinite(item) for item in offset):
            raise RuntimeError('Scan offset must contain three finite numbers')
        return offset

    def setup(context):
        value = lambda key: LaunchConfiguration(key).perform(context)
        weights = str(Path(value('weights')).expanduser().resolve())
        calibration = str(Path(value('calibration')).expanduser().resolve())
        if not Path(weights).is_file():
            raise RuntimeError('Supply weights:=/absolute/path/to/parking/best.pt')
        if not Path(calibration).is_file():
            raise RuntimeError('Real hardware requires calibration:=/absolute/path/to/measured.json')
        speed, parking_pwm, steer_steps = (float(value('speed')),
            int(value('parking_pwm')), int(value('steer_steps')))
        steering_sign = int(value('steering_sign'))
        if (speed <= 0 or not 1 <= parking_pwm <= 255 or
                not 1 <= steer_steps <= 7 or steering_sign not in (-1, 1)):
            raise RuntimeError('speed>0, parking_pwm=1..255, steer_steps=1..7 and steering_sign=±1 required')
        wheelbase = float(value('wheelbase'))
        trailer_wheelbase = float(value('trailer_wheelbase'))
        hitch_to_front_axle = float(value('hitch_to_trailer_front_axle'))
        body_length = float(value('body_length'))
        rear_overhang = float(value('rear_overhang'))
        if (not all(math.isfinite(item) and item > 0 for item in
                    (trailer_wheelbase, hitch_to_front_axle, body_length, rear_overhang)) or
                not rear_overhang < body_length or
                body_length - rear_overhang < wheelbase):
            raise RuntimeError('Invalid real body/axle dimensions')
        geometry = Geometry(
            wheelbase=wheelbase,
            hitch_offset=float(value('hitch_offset')),
            trailer_axle=hitch_to_front_axle + trailer_wheelbase,
            front=body_length - rear_overhang,
            rear=rear_overhang,
            width=float(value('body_width')),
            max_steer=float(value('max_steer_rad')),
            max_beta=math.radians(float(value('max_beta_deg'))))
        vision_params = dict(weights=weights, device=value('device'),
                             calibration=calibration, use_sim_time=False,
                             geometry_override=ParameterValue(
                                 json.dumps(vars(geometry)), value_type=str))
        controller_remaps = []
        controller_params = {'use_sim_time': False, 'parking_speed': speed,
                             'parking_pwm': parking_pwm,
                             'steer_steps': steer_steps,
                             'steering_sign': steering_sign,
                             'sensor_timeout': float(value('sensor_timeout')),
                             'collision_margin': float(value('collision_margin'))}
        if use_lidar:
            controller_remaps = [('/parking/tractor/scan', value('tractor_scan')),
                                 ('/parking/trailer/scan', value('trailer_scan'))]
            controller_params.update(
                tractor_scan_offset=measured_scan_offset(value('tractor_scan_offset')),
                trailer_scan_offset=measured_scan_offset(value('trailer_scan_offset')))
        actions = [
            Node(package='trailer_parking_pkg', executable='parking_command_guard', output='screen'),
            Node(package='serial_communication_pkg', executable='serial_sender_node', output='screen',
                 parameters=[{'port': value('port'), 'baud': int(value('baud')),
                              'startup_delay': float(value('serial_startup_delay'))}]),
            Node(package='trailer_parking_pkg',
                 executable='parallel_lidar_yolo' if use_lidar else 'parallel_camera_yolo',
                 parameters=[controller_params],
                 remappings=controller_remaps, output='screen')]
        for camera in ('front', 'rear'):
            actions.append(Node(package='trailer_parking_pkg', executable='parking_vision',
                name=f'parking_{camera}_vision', parameters=[vision_params, {'camera': camera}],
                remappings=[('image', value(f'{camera}_image')),
                            ('camera_info', value(f'{camera}_info')),
                            ('observation', f'/parking/{camera}/observation')], output='screen'))
        return actions

    arguments = [
        DeclareLaunchArgument('weights', description='YOLO segmentation weights'),
        DeclareLaunchArgument('calibration', description='Measured real-stroller camera calibration JSON'),
        DeclareLaunchArgument('wheelbase', default_value='0.52'),
        DeclareLaunchArgument('trailer_wheelbase', default_value='0.52'),
        DeclareLaunchArgument('hitch_offset', default_value='0.33'),
        DeclareLaunchArgument('hitch_to_trailer_front_axle', default_value='0.30'),
        DeclareLaunchArgument('body_length', default_value='0.99'),
        DeclareLaunchArgument('body_width', default_value='0.55'),
        DeclareLaunchArgument('rear_overhang', default_value='0.235'),
        DeclareLaunchArgument('max_steer_rad', default_value='0.60'),
        DeclareLaunchArgument('max_beta_deg', default_value='20.0'),
        DeclareLaunchArgument('collision_margin', default_value='0.03'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('speed', default_value='0.30'),
        DeclareLaunchArgument('parking_pwm', default_value='90'),
        DeclareLaunchArgument('steer_steps', default_value='7'),
        DeclareLaunchArgument('steering_sign', default_value='-1'),
        DeclareLaunchArgument('sensor_timeout', default_value='0.8'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('serial_startup_delay', default_value='4.5'),
        DeclareLaunchArgument('front_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('front_info', default_value='/camera/camera_info'),
        DeclareLaunchArgument('rear_image', default_value='/rear_camera/image_raw'),
        DeclareLaunchArgument('rear_info', default_value='/rear_camera/camera_info')]
    if use_lidar:
        arguments.extend([
            DeclareLaunchArgument('tractor_scan', default_value='/scan'),
            DeclareLaunchArgument('trailer_scan', default_value='/rear_scan'),
            DeclareLaunchArgument('tractor_scan_offset',
                description='Measured [x,y,yaw] from tractor rear axle'),
            DeclareLaunchArgument('trailer_scan_offset',
                description='Measured [x,y,yaw] from trailer rear axle')])
    arguments.append(OpaqueFunction(function=setup))
    return LaunchDescription(arguments)
