"""Real rclpy message contracts; skipped on systems without ROS 2."""
import json
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import String
from trailer_parking_pkg.controller_node import ParkingController
from trailer_parking_pkg.core import State, trailer_pose


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
