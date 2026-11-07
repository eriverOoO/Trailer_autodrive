"""LiDAR-only SLAM, dual IMUs and automatic stationary relative yaw display."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup=Path(get_package_share_directory('mvsim_slam_real_bringup'))
    viz=Path(get_package_share_directory('tractor_trailer_rviz_2d'))
    geometry=LaunchConfiguration('geometry_file')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port',default_value='/dev/lidar_g6'),
        DeclareLaunchArgument('tractor_imu_port',default_value='/dev/imu_tractor'),
        DeclareLaunchArgument('trailer_imu_port',default_value='/dev/imu_trailer'),
        DeclareLaunchArgument('geometry_file',default_value=str(bringup/'config/vehicle_geometry.yaml')),
        DeclareLaunchArgument('rviz',default_value='true'),
        LogInfo(msg='Keep both vehicles stationary and straight until READY. No vehicle control is started.'),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/g6_slam.launch.py')),
            launch_arguments={'lidar_port':LaunchConfiguration('lidar_port'),
                              'frame_mode':'vehicle','use_imu':'false',
                              'publish_mount_tf':'false','enable_rviz':'false'}.items())]),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/dual_imu.launch.py')),
            launch_arguments={'tractor_imu_port':LaunchConfiguration('tractor_imu_port'),
                              'trailer_imu_port':LaunchConfiguration('trailer_imu_port'),
                              'params_file':str(bringup/'config/imu.yaml'),
                              'geometry_file':geometry,'publish_sensor_tf':'true'}.items())]),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(viz/'launch/visualization.launch.py')),
            launch_arguments={'config_file':str(viz/'config/relative_yaw.yaml'),
                              'rviz_config':str(viz/'rviz/relative_yaw.rviz'),
                              'geometry_file':geometry,'rviz':LaunchConfiguration('rviz')}.items())]),
    ])
