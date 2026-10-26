from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

FRONT_CAMERA_NUM = 2
REAR_CAMERA_NUM = 4
FRONT_CAMERA_PATH = '/dev/video2'
REAR_CAMERA_PATH = '/dev/video4'

def make_camera_pipeline(
        namespace,
        cam_num,
        camera_path,
        model_path,
        frame_id,
        show_parking_start_overlay=False,
        show_end_line_lost_overlay=False):
    return [
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            namespace=namespace,
            name='image_publisher_node',
            parameters=[{
                'cam_num': cam_num,
                'camera_path': camera_path,
                'logger': False,
                'frame_id': frame_id,
            }],
            output='screen'
        ),
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            namespace=namespace,
            name='yolov8_node',
            parameters=[{
                'model': model_path,
                'imgsz': 640,
            }],
            output='screen'
        ),
        Node(
            package='debug_pkg',
            executable='yolov8_visualizer_node',
            namespace=namespace,
            name='yolov8_visualizer_node',
            parameters=[{
                'show_parking_start_overlay': show_parking_start_overlay,
                'parking_start_status_topic': '/parking_start_status',
                'show_end_line_lost_overlay': show_end_line_lost_overlay,
                'end_line_lost_status_topic': '/debug_end_line_lost',
            }],
            output='screen'
        ),
    ]

def generate_launch_description():
    pkg_launch = get_package_share_directory('launch_pkg')
    rviz_config_path = os.path.join(pkg_launch, 'rviz', 'default.rviz')
    workspace_root = os.path.dirname(os.path.dirname(get_package_prefix('launch_pkg')))
    front_model_path = os.path.join(workspace_root, 'best_front.engine')
    rear_model_path = os.path.join(workspace_root, 'best_rear.engine')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch rviz2 automatically with the default debug image view.'
        ),
        *make_camera_pipeline(
            'front',
            FRONT_CAMERA_NUM,
            FRONT_CAMERA_PATH,
            front_model_path,
            'front_camera_frame',
            True,
            False,
        ),
        *make_camera_pipeline(
            'rear',
            REAR_CAMERA_NUM,
            REAR_CAMERA_PATH,
            rear_model_path,
            'rear_camera_frame',
            False,
            True,
        ),
        Node(
            package='camera_perception_pkg',
            executable='parking_space_info_extractor_node',
            name='parking_space_info_extractor_node',
            parameters=[{
                'sub_detection_topic': '/rear/detections',
                'parking_space_class_name': 'parking_space',
                'parking_space_info_topic': 'yolov8_parking_space_info',
            }],
            output='screen'
        ),
        Node(
            package='camera_perception_pkg',
            executable='parking_space_detector_node',
            name='parking_space_detector_node',
            parameters=[{
                'sub_detection_topic': '/front/detections',
                'parking_space_class_name': 'parking_space',
                'pub_topic': 'yolov8_parking_space_detection_info',
                'normalize_bbox_to_base_resolution': True,
                'source_width': 1280.0,
                'source_height': 720.0,
            }],
            output='screen'
        ),
        # LiDAR publisher
        Node(
            package='lidar_perception_pkg',
            executable='lidar_publisher_node',
            name='lidar_publisher_node',
            output='screen'
        ),

        # LiDAR processor
        Node(
            package='lidar_perception_pkg',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            output='screen'
        ),
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            parameters=[{
                'sub_parking_space_bbox_topic': '/yolov8_parking_space_detection_info',
                'sub_parking_space_lane_topic': '/yolov8_parking_space_info',
            }],
            output='screen'
        ),
        Node(
            package='decision_making_pkg',
            executable='parking_space_path_planner_node',
            name='parking_space_path_planner_node',
            output='screen'
        ),
        Node(
            package='serial_communication_pkg',
            executable='serial_sender_node',
            name='serial_sender_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen'
        ),
    ])
