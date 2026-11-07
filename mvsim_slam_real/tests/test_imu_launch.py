from unittest.mock import patch

import pytest
import yaml
from launch import LaunchContext
from launch_ros.actions import Node
from mvsim_slam_real_bringup.g6_launch import imu_mount_tf, nodes


def test_imu_mount_requires_measurements_and_correct_parent(tmp_path):
    path = tmp_path / 'mount.yaml'
    mount = {'measured': False, 'parent_frame': 'laser_frame',
             'child_frame': 'tractor_imu_link', 'xyz': [None]*3, 'rpy': [None]*3}
    path.write_text(yaml.safe_dump(mount))
    with pytest.raises(RuntimeError, match='measured IMU extrinsics'):
        imu_mount_tf(path, 'laser_frame')
    mount.update(measured=True, xyz=[0., 0., .1], rpy=[0., 0., 0.])
    path.write_text(yaml.safe_dump(mount))
    assert imu_mount_tf(path, 'laser_frame') is not None
    with pytest.raises(RuntimeError, match='base_footprint'):
        imu_mount_tf(path, 'base_footprint')
    mount['xyz'][0] = float('nan')
    path.write_text(yaml.safe_dump(mount))
    with pytest.raises(RuntimeError, match='finite'):
        imu_mount_tf(path, 'laser_frame')


@pytest.mark.parametrize('mode,lua', [('sensor', 'g6_sensor_imu.lua'),
                                    ('vehicle', 'g6_imu_optional.lua')])
def test_fusion_selects_tracking_config_for_external_sensors(mode, lua):
    context = LaunchContext()
    context.launch_configurations.update({
        'frame_mode': mode, 'use_imu': 'true', 'publish_imu_tf': 'false',
        'start_imu': 'false', 'publish_mount_tf': 'false',
        'start_driver': 'false', 'scan_topic': '/scan', 'enable_rviz': 'false',
    })
    with patch('mvsim_slam_real_bringup.g6_launch.Node', wraps=Node) as factory:
        nodes(context, slam=True)
    cartographer = next(c.kwargs for c in factory.call_args_list
                        if c.kwargs.get('name') == 'cartographer_node')
    assert lua in cartographer['arguments']
    assert ('imu', '/tractor/imu/data') in cartographer['remappings']
