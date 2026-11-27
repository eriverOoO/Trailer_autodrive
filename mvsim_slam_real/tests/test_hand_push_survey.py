"""Conditional survey consumes real SLAM; never fabricates an IMU/control state."""
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
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/real_parking'))
from real_parking.hand_push_survey import HandPushSurvey
from real_parking.nodes import grid_from_message
from tractor_parking_perception import Target


@pytest.fixture
def node():
    filename = Path(__file__).resolve().parents[1]/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'
    rclpy.init(args=['--ros-args', '-p', f'geometry_file:={filename}'])
    instance = HandPushSurvey()
    try:
        yield instance
    finally:
        instance.close(); instance.destroy_node(); rclpy.shutdown()


def measured_map():
    m = OccupancyGrid()
    m.header.frame_id = 'map'; m.header.stamp.sec = 1
    m.info.width = 360; m.info.height = 260; m.info.resolution = .05
    m.info.origin.position.x = m.info.origin.position.y = -5.
    m.info.origin.orientation.w = 1.
    data = np.zeros((260,360), dtype=np.int8)
    data[120:136,120:140] = 100
    data[120:136,206:226] = 100
    m.data = data.ravel().tolist()
    return m


def test_survey_has_no_motion_publisher_and_rejects_auto_search(node):
    topics = {topic for topic, _ in node.get_publisher_names_and_types_by_node(node.get_name(), '/')}
    assert not topics.intersection({'/parking/request_cmd_vel', '/topic_control_signal', '/parking/state', '/parking/planned_tractor_path'})
    assert not node.reject_motion(Trigger.Request(), Trigger.Response()).success
    assert not node.request_plan(Trigger.Request(), Trigger.Response()).success


def test_stationary_result_can_be_published_as_json(node):
    now = time.monotonic()
    for age in (1., .8, .6, .4, .2, 0.):
        node.samples.append((now-age,np.zeros(3)))
    stationary = node.stationary()
    assert type(stationary) is bool and stationary
    assert 'true' in json.dumps(dict(stationary=stationary))


def test_stop_threshold_needs_three_distinct_stable_maps(node):
    node.map = measured_map(); node.road = np.zeros(3)
    node.observe(); node.observe()
    assert len(node.targets) == 1
    assert node.stop_distance(np.array([10.,0.,0.])) is None
    for second in (2,3):
        node.map.header.stamp.sec = second
        node.observe()
    assert len(node.targets) == 3
    assert node.stop_distance(np.array([7.,0.,0.])) > 0
    assert node.stop_distance(np.array([10.,0.,0.])) < 0
    # Lost/rejected target cannot retain previous confirmation counts.
    node.map.data = [0]*(360*260)
    node.observe()
    assert len(node.targets) == 0


def test_survey_requires_fresh_slam_matches_and_distinct_pose_samples(node):
    node.map = measured_map(); node.scan = LaserScan(); node.scan.header.frame_id = 'laser_frame'
    ns = node.get_clock().now().nanoseconds
    for key in ('scan','map','match'):
        node.receipts[key] = (time.monotonic(),ns)
    tf = TransformStamped(); tf.header.stamp = node.get_clock().now().to_msg()
    tf.transform.rotation.w = 1.
    node.tf = Mock(); node.tf.lookup_transform.return_value = tf
    _, assumed = node.fresh_pose()
    node.fresh_pose()
    assert len(node.samples) == 1 and not node.stationary()
    assert assumed[2] == assumed[3]  # Explicit conditional planning assumption.
    node.receipts['match'] = (time.monotonic(),ns-10**9)
    with pytest.raises(ValueError,match='match'):
        node.fresh_pose()


def test_sensor_loss_after_arming_requests_stop_without_starting_plan(node):
    node.armed = True
    node.fresh_pose = Mock(side_effect=ValueError('scan 끊김'))
    node.pubs = {key:Mock() for key in ('guidance','observation','status')}
    node.tick()
    advice = json.loads(node.pubs['guidance'].publish.call_args.args[0].data)
    assert advice['stop_requested'] and not advice['stationary'] and advice['survey_only']
    assert node.future is None and node.stop_reason is not None


def test_planning_requires_stopped_rig_and_moving_invalidates_preview(node):
    node.fresh_pose = lambda: (np.zeros(3),np.zeros(4))
    node.stationary = lambda: False
    assert not node.request_plan(Trigger.Request(), Trigger.Response()).success
    node.plan_start = np.zeros(4); node.path_pub = Mock()
    node.invalidate_plan()
    assert node.plan_invalid
    assert not node.path_pub.publish.call_args.args[0].poses


def test_reference_planner_returns_whole_rig_reverse_path_on_observed_free_map(node):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('planner_timeout',value=3.)])
    m=measured_map(); m.data=[0]*(360*260)
    target=Target(np.array([0.,0.]),0.,3.6,1.10)
    start=node.footprints.goal(np.array([-.4,0.]),0.)
    start[0]+=1.
    result=node.plan(grid_from_message(m),target,start)
    assert result.states.shape[1]==4 and len(result.states)>1
    assert np.isfinite(result.states).all()
    assert -1 in result.gears
    np.testing.assert_allclose(result.states[0],start,atol=1e-6)
    assert target.contains(node.footprints.world(result.states[-1]))
