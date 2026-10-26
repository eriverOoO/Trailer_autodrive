"""Default software development launch: never opens a serial/camera device."""
from pathlib import Path
import os,datetime,yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,SetEnvironmentVariable,LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def enabled(context,name):return LaunchConfiguration(name).perform(context).lower() in ['true','1','yes']
def launch_nodes(context):
    share=Path(get_package_share_directory('mvsim_slam_real_bringup'));c=share/'config';dev=enabled(context,'development_mode');act=enabled(context,'enable_actuation')
    geometry=LaunchConfiguration('geometry_file').perform(context);g=yaml.safe_load(Path(geometry).read_text())
    controller=yaml.safe_load((c/'controller.yaml').read_text())['control_adapter']['ros__parameters']
    if act and dev:raise RuntimeError('enable_actuation requires development_mode=false')
    if act and not (controller['motor_scale_confirmed'] and controller['polarity_confirmed'] and g.get('geometry_confirmed',False) and g['initial_alignment_confirmed'] and g['trailer_bias_confirmed']):raise RuntimeError('Actuation requires measured geometry, confirmed motor scale/polarity, initial alignment and bias in YAML')
    ports={key:LaunchConfiguration(key).perform(context) for key in ['lidar_port','tractor_imu_port','trailer_imu_port','arduino_port']}
    imu_ports=[ports[key] for key,flag in [('tractor_imu_port','enable_tractor_imu'),('trailer_imu_port','enable_trailer_imu')] if enabled(context,flag)]
    driver=LaunchConfiguration('lidar_driver').perform(context)
    if driver not in ['external','rplidar']:raise RuntimeError('lidar_driver must be external or rplidar; sensor manufacturer is undecided')
    if not dev:
        required=imu_ports+([ports['lidar_port']] if driver=='rplidar' and enabled(context,'enable_lidar') else [])+([ports['arduino_port']] if act else [])
        if any(not port for port in required):raise RuntimeError('Explicit hardware ports required outside development mode')
        if len(required)!=len(set(required)):raise RuntimeError('Each hardware device needs a distinct port')
    if enabled(context,'enable_vision') and dev:raise RuntimeError('Vision device launch is excluded from hardware-free development mode; use recorded inputs separately')
    fusion=enabled(context,'enable_imu_fusion')
    if fusion and not enabled(context,'enable_tractor_imu'):raise RuntimeError('Optional Cartographer IMU fusion requires tractor IMU and gravity-inclusive soa1 format')
    output=Path(LaunchConfiguration('output_directory').perform(context));output.mkdir(parents=True,exist_ok=True)
    nodes=[LogInfo(msg=f'Development mode={dev}; actuation={act}; absent sensors remain unavailable'),SetEnvironmentVariable('ROS_LOG_DIR',str(output/'ros')),SetEnvironmentVariable('PYTHONNOUSERSITE','1'),Node(package='real_vehicle_bridge',executable='sensor_tf',parameters=[{'geometry_file':geometry}],output='screen'),Node(package='real_vehicle_bridge',executable='state_estimator',parameters=[{'geometry_file':geometry}],output='screen'),Node(package='real_vehicle_bridge',executable='control_adapter',parameters=[str(c/'controller.yaml'),{'geometry_file':geometry,'enable_actuation':act}],output='screen')]
    if enabled(context,'enable_lidar'):
        nodes.append(Node(package='real_vehicle_bridge',executable='lidar_gateway',parameters=[str(c/'lidar.yaml')],output='screen'))
        if driver=='rplidar':nodes.append(Node(package='real_vehicle_bridge',executable='rplidar_source',parameters=[str(c/'lidar.yaml'),{'port':ports['lidar_port'],'enable_device':not dev}],output='screen'))
    for name,flag in [('tractor','enable_tractor_imu'),('trailer','enable_trailer_imu')]:
        if enabled(context,flag):nodes.append(Node(package='ebimu_driver',executable='ebimu_node',name=name+'_imu',parameters=[str(c/'imu.yaml'),{'port':ports[name+'_imu_port'],'enable_device':not dev}],output='screen'))
    if enabled(context,'enable_slam'):
        lua='reference_imu_optional.lua' if fusion else 'reference_lidar_only.lua'
        nodes += [Node(package='cartographer_ros',executable='cartographer_node',name='cartographer_node',parameters=[str(c/'slam.yaml')],arguments=['-configuration_directory',str(c),'-configuration_basename',lua],remappings=[('scan','/scan'),('imu','/tractor/imu/data')],output='screen'),Node(package='cartographer_ros',executable='cartographer_occupancy_grid_node',name='cartographer_occupancy_grid_node',parameters=[str(c/'slam.yaml')],arguments=['-resolution','0.05','-publish_period_sec','1.0'],output='screen')]
    nodes.append(Node(package='real_parking',executable='auto_parking',parameters=[str(c/'parking.yaml'),{'world_file':geometry,'blocked_debug_directory':str(output)}],output='screen'))
    if act:nodes.append(Node(package='serial_communication_pkg',executable='serial_sender_node',parameters=[{'port':ports['arduino_port'],'sub_topic':'/topic_control_signal'}],output='screen'))
    if enabled(context,'enable_rviz'):nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/real_parking.rviz')],output='screen'))
    if enabled(context,'enable_vision'):
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/vision.launch.py')),launch_arguments={'camera_path':LaunchConfiguration('camera_path'),'model':LaunchConfiguration('vision_model')}.items()))
    return nodes

def generate_launch_description():
    share=Path(get_package_share_directory('mvsim_slam_real_bringup'))
    defaults={'development_mode':'true','enable_lidar':'true','lidar_driver':'external','enable_tractor_imu':'true','enable_trailer_imu':'true','enable_slam':'true','enable_vision':'false','enable_rviz':'false','enable_actuation':'false','enable_imu_fusion':'false','lidar_port':'','tractor_imu_port':'','trailer_imu_port':'','arduino_port':'','camera_path':'','vision_model':'','geometry_file':str(share/'config/vehicle_geometry.yaml'),'output_directory':str(Path.home()/'mvsim_slam_real/log'/('run_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')))}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=launch_nodes)])
