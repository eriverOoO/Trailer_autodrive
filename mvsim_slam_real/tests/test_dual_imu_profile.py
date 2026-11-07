"""Matched wire format and one physical mount rotation for the reversed IMU."""
import importlib.util
import math
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest
import yaml
from launch import LaunchContext
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/mvsim_slam_real_bringup/config'


def launch_module():
    path = ROOT / 'src/mvsim_slam_real_bringup/launch/dual_imu.launch.py'
    spec = importlib.util.spec_from_file_location('dual_imu', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_matching_profiles_and_single_mount_rotation():
    profiles = yaml.safe_load((CONFIG / 'imu.yaml').read_text())
    a = profiles['tractor_imu']['ros__parameters'].copy()
    b = profiles['trailer_imu']['ros__parameters'].copy()
    for key in ('port', 'frame_id', 'output_topic'):
        assert a.pop(key) != b.pop(key)
    assert a == b
    assert a['mounting_rpy'] == [0., 0., 0.]
    assert a['orientation_format'] == 'quaternion'
    assert a['include_acceleration'] and a['include_timestamp']
    assert a['output_period'] == .01 and a['baud'] == 115200
    single = yaml.safe_load((CONFIG / 'ebimu_slam.yaml').read_text())['tractor_imu']['ros__parameters']
    for key in a:
        if key != 'enable_device':
            assert single[key] == a[key]
    g = yaml.safe_load((CONFIG / 'vehicle_geometry.yaml').read_text())
    r = Rotation.from_euler('xyz', g['trailer_imu_offset'][3:])
    np.testing.assert_allclose(r.apply([1., 2., 3.]), [-1., -2., 3.], atol=1e-12)
    assert g['trailer_imu_offset'][5] - g['tractor_imu_offset'][5] == pytest.approx(math.pi)


def test_launch_rejects_aliases_for_same_device_before_open():
    module = launch_module()
    context = LaunchContext()
    context.launch_configurations.update(tractor_imu_port='/dev/a', trailer_imu_port='/dev/b')
    with patch.object(module.os.path, 'realpath', return_value='/dev/ttyUSB1'), \
            patch.object(module, 'check_port') as check:
        with pytest.raises(RuntimeError, match='distinct physical'):
            module.nodes(context)
        check.assert_not_called()


def test_launch_only_sensing_and_mount_nodes():
    module = launch_module()
    context = LaunchContext()
    context.launch_configurations.update(tractor_imu_port='/dev/a', trailer_imu_port='/dev/b',
                                        params_file=str(CONFIG/'imu.yaml'),
                                        geometry_file=str(CONFIG/'vehicle_geometry.yaml'),
                                        publish_sensor_tf='true')
    with patch.object(module, 'check_port'), patch.object(module, 'Node') as factory, \
            patch.object(module, 'critical', side_effect=lambda node: [node]):
        module.nodes(context)
        calls = [call.kwargs for call in factory.call_args_list]
    assert [c['executable'] for c in calls] == ['ebimu_node', 'ebimu_node', 'sensor_tf']
    assert [c['name'] for c in calls[:2]] == ['tractor_imu', 'trailer_imu']
