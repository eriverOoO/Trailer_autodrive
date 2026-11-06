"""Real rclpy message contracts; skipped on systems without ROS 2."""
import json
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import String
from trailer_parking_pkg.controller_node import ParkingController
from trailer_parking_pkg.core import State, Slot, Scene, trailer_pose


@pytest.fixture
def controller():
    rclpy.init()
    node = ParkingController(False)
    yield node
    node.executor_pool.shutdown(wait=True)
    node.destroy_node()
    rclpy.shutdown()


def inject(node, stamp=10.0):
    s = State(0, 0, 0, 0)
    for name, pose in [('front', (0, 0, 0)), ('rear', trailer_pose(s, node.g))]:
        msg = String()
        msg.data = json.dumps(dict(camera=name, stamp=stamp, valid=True, pose=pose,
                                   geometry=vars(node.g), slots=[], obstacles=[]))
        node.observe(name, msg)


def test_camera_only_does_not_subscribe_scan_or_truth(controller):
    topics = [sub.topic_name for sub in controller.subscriptions]
    assert not any('scan' in t or 'odom' in t or 'articulation' in t or 'model_states' in t for t in topics)
    inject(controller)
    state, scene = controller.inputs(10.1)
    assert scene.free(state, controller.g)
    with pytest.raises(ValueError, match='STALE'):
        controller.inputs(11.0)


def test_inconsistent_rear_pose_stops(controller):
    inject(controller)
    controller.observations['rear']['pose'][0] += 1.0
    with pytest.raises(ValueError, match='HITCH_INCONSISTENT'):
        controller.inputs(10.1)


def test_fused_requires_both_lidars(controller):
    inject(controller)
    controller.use_lidar = True
    with pytest.raises(ValueError, match='WAIT_LIDAR'):
        controller.inputs(10.1)


def test_capture_only_saves_recognized_scene_and_publishes_only_stop(controller, tmp_path, monkeypatch):
    from types import SimpleNamespace
    controller.capture_output = str(tmp_path / 'snapshot.json')
    controller.slot = Slot(0, 0, 0, 11.1, 3.2)
    state, scene = State(0, 0, 0, 0), Scene()
    monkeypatch.setattr(controller, 'inputs', lambda now: (state, scene))
    messages = []
    controller.publisher = SimpleNamespace(publish=messages.append)
    controller.control_tick()
    snapshot = json.loads((tmp_path / 'snapshot.json').read_text())
    assert snapshot['schema'] == 'parking_snapshot_v1'
    assert snapshot['slot']['length'] == 11.1
    assert controller.fault == 'SNAPSHOT_SAVED'
    controller.publish_motion(.3, .4)
    assert messages and all((m.steering, m.left_speed, m.right_speed) == (0, 0, 0) for m in messages)


def test_capture_only_never_approaches_a_visible_slot(controller, tmp_path, monkeypatch):
    from types import SimpleNamespace
    controller.capture_output = str(tmp_path / 'snapshot.json')
    inject(controller)
    controller.observations['front']['slots'] = [dict(x=0, y=0, yaw=0, length=1, width=1)]
    monkeypatch.setattr(controller, 'inputs', lambda now: (State(0, 0, 0, 0), Scene()))
    messages = []
    controller.publisher = SimpleNamespace(publish=messages.append)
    controller.control_tick()
    assert controller.phase == 'CAPTURE_WAIT_VISIBLE_SLOT_STATIONARY'
    assert not (tmp_path / 'snapshot.json').exists()
    assert messages and all(m.left_speed == m.right_speed == 0 for m in messages)
