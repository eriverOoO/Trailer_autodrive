from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    def value(name):return LaunchConfiguration(name).perform(context)
    def boolean(name):
        v=value(name).lower()
        if v not in ('true','false'):raise ValueError(name+' must be true/false')
        return v=='true'
    selected={r:value(r+'_port') for r in ('tractor','trailer') if boolean('start_'+r)}
    if selected and not boolean('ports_confirmed'):
        raise ValueError('physically confirm role/IMU/LiDAR mapping before ports_confirmed:=true')
    ports=[os.path.realpath(p) for p in selected.values()]
    if any(not p for p in selected.values()) or len(set(ports))!=len(ports):
        raise ValueError('explicit distinct physical IMU ports required')
    lidar=value('lidar_port')
    if lidar and os.path.realpath(lidar) in ports:raise ValueError('IMU port equals LiDAR port')
    base=value('output_directory')
    if selected and not base:raise ValueError('new output_directory required')
    out=[]
    for role,port in selected.items():
        other=value(('trailer' if role=='tractor' else 'tractor')+'_port')
        out.append(Node(package='tractor_trailer_rviz_2d',executable='tt2d_serial_bridge',
                        name='vehicle_viz_2d_'+role+'_bridge',output='screen',parameters=[{
                            'role':role,'port':port,'other_imu_port':other,'lidar_port':lidar,
                            'ports_confirmed':True,'baud':int(value(role+'_baud')),
                            'orientation':value(role+'_orientation'),
                            'gyro':boolean(role+'_gyro'),'accel':boolean(role+'_accel'),
                            'timestamp':boolean(role+'_timestamp'),
                            'output_directory':str(Path(base)/role)}]))
    return out


def generate_launch_description():
    defaults={'start_tractor':'false','start_trailer':'false','tractor_port':'','trailer_port':'',
              'lidar_port':'','ports_confirmed':'false','output_directory':''}
    for role in ('tractor','trailer'):
        defaults.update({role+'_baud':'115200',role+'_orientation':'quaternion',
                         role+'_gyro':'true',role+'_accel':'true',role+'_timestamp':'true'})
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=start)])
