"""Preserved vision perception only. Original motor/planning/sender nodes excluded."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def nodes(context):
    camera=LaunchConfiguration('camera_path').perform(context);model=LaunchConfiguration('model').perform(context)
    if not camera or not model:raise RuntimeError('Explicit camera_path and YOLO model file required')
    return [Node(package='camera_perception_pkg',executable='image_publisher_node',namespace='front',parameters=[{'data_source':'camera','camera_path':camera,'logger':False}],output='screen'),Node(package='camera_perception_pkg',executable='yolov8_node',namespace='front',parameters=[{'model':model,'device':LaunchConfiguration('device').perform(context)}],output='screen'),Node(package='debug_pkg',executable='yolov8_visualizer_node',namespace='front',output='screen'),Node(package='camera_perception_pkg',executable='parking_space_detector_node',parameters=[{'sub_detection_topic':'/front/detections'}],output='screen')]

def generate_launch_description():return LaunchDescription([DeclareLaunchArgument('camera_path',default_value=''),DeclareLaunchArgument('model',default_value=''),DeclareLaunchArgument('device',default_value='cpu'),OpaqueFunction(function=nodes)])
