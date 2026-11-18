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
from nav_msgs.msg import OccupancyGrid
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


def test_stop_uses_fresh_stationary_poses_and_requires_new_detection_maps(node):
    node.streams_ready = lambda: True
    node.est.pose_stamp = 1.
    node.detection_msg = OccupancyGrid()
    node.detection_msg.header.stamp.sec = 7
    state = np.zeros(4)
    now = time.monotonic()
    node.begin_stabilizing(state, now-1., 'test stop')
    node.stop_and_stabilize(state, now-.95)
    assert node.stage == 'STOP_AND_STABILIZE' and not node.stationary_samples
    for index, offset in enumerate((.9, .67, .45, .23, 0.)):
        node.est.pose_stamp = 1.1 + index*.1
        node.stop_and_stabilize(state, now-offset)
    assert node.stage == 'DETECT_TARGET'
    assert np.array_equal(node.staging_reference, state)
    target = Target(np.array([3., 2.]), 0., 3.3, 1.10)
    assert not node.stable_observation(target) and node.observation_hits == 0
    for stamp in (8, 9, 10):
        node.detection_msg.header.stamp.sec = stamp
        assert node.stable_observation(target) == (stamp == 10)


@pytest.mark.parametrize('has_state', [True, False])
def test_stop_times_out_when_stationarity_cannot_be_confirmed(node, has_state):
    node.streams_ready = lambda: False
    state = np.zeros(4)
    now = time.monotonic()
    node.begin_stabilizing(state, now-11., 'test stop')
    node.camera_ready = lambda: True
    node.test_motion_enabled = True
    node.est.update = lambda **kwargs: state if has_state else None
    node.tick_checked()
    assert node.stage == 'FAILED' and node.v == 0.


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


@pytest.fixture
def lidar_only_node(tmp_path, monkeypatch):
    geometry = Path(__file__).resolve().parents[1]/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'
    monkeypatch.setattr(Geometry, 'from_world', classmethod(lambda cls, name: VehicleGeometry(name).model()))
    rclpy.init(args=['--ros-args', '-p', f'world_file:={geometry}',
                    '-p', f'blocked_debug_directory:={tmp_path}', '-p', 'slot_width:=1.10',
                    '-p', 'require_camera:=false', '-p', 'allow_lidar_only_motion:=true'])
    instance = TwoStructureParkingTest()
    try:
        yield instance
    finally:
        instance.planner_pool.shutdown(wait=True, cancel_futures=True)
        instance.destroy_node()
        rclpy.shutdown()


def test_lidar_only_test_stage_starts_without_camera_and_is_launch_only(lidar_only_node):
    from rclpy.parameter import Parameter
    node = lidar_only_node
    node.streams_ready = node.stationary = lambda: True
    assert not node.camera_valid and node.lidar_only_motion()
    response = node.start_search(Trigger.Request(), Trigger.Response())
    assert response.success and node.test_motion_enabled and 'LiDAR-only' in response.message
    assert not node.set_parameters([Parameter('allow_lidar_only_motion', value=False)])[0].successful


def test_lidar_only_manual_handoff_without_camera(lidar_only_node):
    node = lidar_only_node
    node.operation = 'manual'
    node.current = np.array([0., 0., 0., 0.])
    node.road_pose = node.current[:3]
    node.observation_hits = 3
    node.streams_ready = node.stationary = lambda: True
    node.detect_gap = lambda *args: PairObservation(target=Target(np.array([3., 2.]), 0., 3.3, 1.10))
    assert node.start_parking(Trigger.Request(), Trigger.Response()).success


def test_allow_flag_alone_keeps_camera_required(node):
    # Default require_camera:=true: the LiDAR-only flag cannot be switched on at runtime either.
    from rclpy.parameter import Parameter
    assert not node.lidar_only_motion()
    assert not node.set_parameters([Parameter('allow_lidar_only_motion', value=True)])[0].successful
    node.streams_ready = node.stationary = lambda: True
    assert not node.start_search(Trigger.Request(), Trigger.Response()).success


@pytest.mark.parametrize('stage', ['PLAN', 'REPLAN', 'READY', 'FAILED', 'EMERGENCY_STOP', 'PARKED'])
def test_recognition_continues_without_motion_or_replanning(node, stage):
    node.stage = stage
    node.failure = 'no bay-contained goal has a safe reverse entry'
    node.road_pose = np.zeros(3)
    target = Target(np.array([3., 2.]), 0., 3.3, 1.10)
    node.target = target
    node.cmd = Mock()
    node.detect_gap = Mock(return_value=PairObservation(target=target, lidar_target=target))
    node.select_goal = Mock(side_effect=AssertionError('recognition must not select parking goals'))
    node.launch_plan = Mock(side_effect=AssertionError('recognition must not replan'))
    node.grid = Mock(side_effect=AssertionError('recognition must not use the walled planning grid'))
    for _ in range(2):
        node.refresh_recognition()
    assert node.detect_gap.call_count == 2
    assert node.stage == stage and node.target is target
    assert node.failure == 'no bay-contained goal has a safe reverse entry'
    node.cmd.publish.assert_not_called()
    node.select_goal.assert_not_called()
    node.launch_plan.assert_not_called()


def test_recognition_refresh_does_not_duplicate_forward_search_or_manual_detection(node):
    node.road_pose = np.zeros(3)
    node.detect_gap = Mock()
    for stage in ('MANUAL_SURVEY', 'WAIT_FOR_START_COMMAND', 'SEARCH_FORWARD',
                  'FIRST_OBSTACLE_SEEN', 'GAP_SEEN', 'SECOND_OBSTACLE_SEEN', 'DETECT_TARGET'):
        node.stage = stage
        node.refresh_recognition()
    node.detect_gap.assert_not_called()


def test_recognition_error_does_not_clear_the_confirmed_target_or_failure(node):
    node.stage, node.failure = 'FAILED', 'parking path blocked by wall'
    node.road_pose = np.zeros(3)
    target = Target(np.array([3., 2.]), 0., 3.3, 1.10)
    node.target = target
    node.detect_gap = Mock(side_effect=RuntimeError('detection unavailable'))
    node.publish_observation = Mock()
    node.refresh_recognition()
    observation = node.publish_observation.call_args.args[0]
    assert observation.target is None and 'detection unavailable' in observation.reason
    assert node.target is target and node.stage == 'FAILED'
    assert node.failure == 'parking path blocked by wall'
