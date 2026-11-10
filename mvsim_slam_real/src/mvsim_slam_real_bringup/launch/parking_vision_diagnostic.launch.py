"""Use the existing camera publisher or an external image stream for diagnostics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    """Start only the diagnostic and, if requested, the camera publisher."""
    camera_path = LaunchConfiguration('camera_path').perform(context)
    image_topic = LaunchConfiguration('image_topic')
    params_file = LaunchConfiguration('detector_params').perform(context)
    parameters = [params_file] if params_file else []
    parameters.append({'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'),
                                                          value_type=str)})
    actions = [Node(
        package='camera_perception_pkg', executable='parking_object_diagnostic',
        namespace='parking_vision', parameters=parameters,
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
        OpaqueFunction(function=nodes),
    ])
