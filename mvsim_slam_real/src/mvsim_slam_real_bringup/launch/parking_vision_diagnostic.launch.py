"""Use the existing camera publisher or an external image stream for diagnostics."""
import os
from pathlib import Path
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from mvsim_slam_real_bringup.vision_defaults import ride_on_assets


def nodes(context):
    """Start only the diagnostic and, if requested, the camera publisher."""
    camera_path = LaunchConfiguration('camera_path').perform(context)
    image_topic = LaunchConfiguration('image_topic')
    params_file = LaunchConfiguration('detector_params').perform(context)
    backend = LaunchConfiguration('detector_backend').perform(context)
    if backend not in ('heuristic', 'ride_on_onnx'):
        raise RuntimeError('detector_backend must be heuristic or ride_on_onnx')
    reliability = LaunchConfiguration('image_reliability', default='auto').perform(context)
    if reliability not in ('auto', 'reliable', 'best_effort'):
        raise RuntimeError('image_reliability must be auto, reliable or best_effort')
    reliable_image = (bool(camera_path) or backend == 'ride_on_onnx') if reliability == 'auto' else reliability == 'reliable'
    model = LaunchConfiguration('detector_model').perform(context)
    python = LaunchConfiguration('vision_python').perform(context)
    if backend == 'ride_on_onnx':
        default_model, default_python = ride_on_assets()
        model, python = model or default_model, python or default_python
        if not model or not Path(model).expanduser().is_file():
            raise RuntimeError('Ride-on ONNX model missing; run tools/prepare_ride_on_runtime.sh '
                               'or set detector_model')
        if not python or not os.access(Path(python).expanduser(), os.X_OK):
            raise RuntimeError('Vision Python runtime missing; run tools/prepare_ride_on_runtime.sh '
                               'or set vision_python')
    if python:
        python = str(Path(python).expanduser().absolute())
        # Preserve a virtualenv's symlink path: resolving it would use system Python.
        if not os.access(python, os.X_OK):
            raise RuntimeError(f'vision_python is not executable: {python}')
    parameters = [params_file] if params_file else []
    parameters.append({'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'),
                                                          value_type=str),
                       'reliable_image': reliable_image,
                       'detector_backend': backend, 'detector_model': model,
                       'detector_confidence': ParameterValue(LaunchConfiguration('detector_confidence'), value_type=float)})
    actions = [Node(
        package='camera_perception_pkg', executable='parking_object_diagnostic',
        namespace='parking_vision', parameters=parameters,
        prefix=[shlex.quote(python)] if python else None,
        remappings=[('image_raw', image_topic), ('scan', LaunchConfiguration('scan_topic'))],
        output='screen',
    )]
    if camera_path:
        actions.append(Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            namespace='parking_vision', output='screen', remappings=[('image_raw', image_topic)],
            parameters=[{
                'data_source': 'camera',
                'camera_path': ParameterValue(camera_path, value_type=str),
                'frame_id': 'parking_left_camera_optical_frame',
                'logger': False,
            }],
        ))
    return actions


def generate_launch_description():
    """Declare camera, scan and calibration inputs."""
    return LaunchDescription([
        DeclareLaunchArgument('camera_path', default_value=''),
        DeclareLaunchArgument('image_topic', default_value='/parking_vision/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('detector_params', default_value=''),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('detector_backend', default_value='heuristic'),
        DeclareLaunchArgument('detector_model', default_value=''),
        DeclareLaunchArgument('detector_confidence', default_value='0.65'),
        DeclareLaunchArgument('vision_python', default_value=''),
        DeclareLaunchArgument('image_reliability', default_value='auto'),
        OpaqueFunction(function=nodes),
    ])
