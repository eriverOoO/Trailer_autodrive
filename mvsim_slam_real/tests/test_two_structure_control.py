"""ROS node entry/command guards with synthetic state, never physical actuation."""

from pathlib import Path
import json
import sys
import time
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('rclpy')
import rclpy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/real_parking'))
from real_parking.two_structure_test import TwoStructureParkingTest
from real_parking.two_structure_gap import PairObservation
from real_vehicle_bridge.geometry import VehicleGeometry
from tractor_trailer_model import Geometry
from tractor_parking_perception import Target


@pytest.fixture
def node(tmp_path, monkeypatch):
    geometry = Path(__file__).resolve().parents[1]/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'
    monkeypatch.setattr(Geometry, 'from_world', classmethod(lambda cls, name: VehicleGeometry(name).model()))
    rclpy.init(args=['--ros-args', '-p', f'world_file:={geometry}',
                    '-p', f'blocked_debug_directory:={tmp_path}',
                    '-p', 'auto_execute:=true', '-p', 'slot_width:=1.10'])
    instance = TwoStructureParkingTest()
    try:
        yield instance
    finally:
        instance.planner_pool.shutdown(wait=True, cancel_futures=True)
        instance.destroy_node()
        rclpy.shutdown()


def test_idle_cannot_publish_nonzero_and_auto_search_needs_camera(node):
    node.cmd = Mock()
    node.send(.14, .1)
    assert node.cmd.publish.call_args.args[0].linear.x == 0.0
    node.streams_ready = lambda: True
    node.stationary = lambda: True
    response = node.start_search(Trigger.Request(), Trigger.Response())
    assert not response.success and not node.test_motion_enabled
    node.camera_ready = lambda: True
    response = node.start_search(Trigger.Request(), Trigger.Response())
    assert response.success and node.operation == 'automatic_search'
    assert node.stage == 'WAIT_FOR_LOCALIZATION'
    node.send(.14, 0.)
    assert node.cmd.publish.call_args.args[0].linear.x == .14
    node.stop_test(Trigger.Request(), Trigger.Response())
    node.send(.14, .1)
    assert node.cmd.publish.call_args.args[0].linear.x == 0.0


def test_manual_handoff_requires_stop_and_enters_existing_settle_state(node):
    node.operation = 'manual'
    node.current = np.array([0., 0., 0., 0.])
    node.road_pose = node.current[:3]
    node.observation_hits = 3
    node.streams_ready = node.camera_ready = lambda: True
    node.stationary = lambda: False
    node.detect_gap = lambda *args: PairObservation(target=Target(np.array([3., 2.]), 0., 3.3, 1.10))
    assert not node.start_parking(Trigger.Request(), Trigger.Response()).success
    node.stationary = lambda: True
    assert node.start_parking(Trigger.Request(), Trigger.Response()).success
    assert node.stage == 'STOP_AND_STABILIZE'
    assert node.test_motion_enabled and node.observation_hits == 0


def test_uncalibrated_camera_cannot_confirm_and_sequential_evidence_expires(node):
    node.on_associations(String(data=json.dumps({'calibrated': False, 'valid': False})))
    assert not node.camera_ready()
    for parent, child in [('odom', 'laser_frame'), ('local_map', 'odom')]:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.rotation.w = 1.0
        node.association_buffer.set_transform_static(tf, 'SYNTHETIC_ONLY')
    now = node.get_clock().now().nanoseconds
    for i, x in enumerate([1., 5.]):
        node.on_associations(String(data=json.dumps({
            'calibrated': True, 'valid': True, 'image_stamp_ns': now+i,
            'scan_stamp_ns': now+i, 'scan_frame': 'laser_frame',
            'objects': [{'label': 'white body', 'scan_xy': [[x, 1.], [x+.05, 1.], [x+.10, 1.]]}],
        })))
    assert node.camera_ready()
    assert len(node.evidence_in_map('local_map')) == 2
    points = node.camera_evidence[0][1]
    node.camera_evidence[0] = (time.monotonic()-181., points)
    assert len(node.evidence_in_map('local_map')) == 1


def test_lidar_only_bypass_cannot_start_automatic_motion(node):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('require_camera', value=False)])
    node.streams_ready = node.stationary = lambda: True
    assert not node.start_search(Trigger.Request(), Trigger.Response()).success
