from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    def value(name):return LaunchConfiguration(name).perform(context)
    out=[Node(package='tractor_trailer_rviz_2d',executable='tt2d_visualizer',output='screen',
              parameters=[{'config_file':value('config_file'),'tractor_topic':value('tractor_topic'),
                           'geometry_file':value('geometry_file'),
                           'trailer_topic':value('trailer_topic'),'slam_frame':value('slam_frame'),
                           'output_directory':value('output_directory'),
                           'visualization_mode':value('visualization_mode'),'use_sim_time':False}])]
    if value('rviz').lower()=='true':
        out.append(Node(package='rviz2',executable='rviz2',name='tractor_trailer_rviz_2d',
                        arguments=['-d',value('rviz_config')],parameters=[{'use_sim_time':False}],output='screen'))
    return out


def generate_launch_description():
    share=Path(get_package_share_directory('tractor_trailer_rviz_2d'))
    defaults={'config_file':str(share/'config/live.yaml'),'geometry_file':'','tractor_topic':'','trailer_topic':'',
              'slam_frame':'','output_directory':'','visualization_mode':'',
              'rviz':'true','rviz_config':str(share/'rviz/live.rviz')}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=start)])
