import copy

import pytest

pytest.importorskip('rclpy')
from interfaces_pkg.msg import Detection, DetectionArray  # noqa: E402
from decision_making_pkg.parking_node import StageSequence, normalize, validate_profile  # noqa: E402

TARGET = {'class': 'car_front', 'box': [0.5, 0.5, 0.3, 0.3], 'tolerance': [0.03, 0.03, 0.02, 0.02]}
PROFILE = {'tuned': True, 'stages': [
    {'name': 'ENTRY', 'steering': 7, 'pwm': -90, 'timeout': 3.0, 'targets': {'rear': TARGET}},
    {'name': 'ALIGN', 'steering': -7, 'pwm': -90, 'timeout': 3.0,
     'targets': {'front': dict(TARGET, **{'class': 'car_back'}), 'rear': TARGET}}]}


def obs(stamp, rear=(0.5, 0.5, 0.3, 0.3), front=(0.5, 0.5, 0.3, 0.3)):
    return {'rear': {'stamp': stamp, 'boxes': [{'class': 'car_front', 'box': list(rear)}]},
            'front': {'stamp': stamp, 'boxes': [{'class': 'car_back', 'box': list(front)}]}}


def test_drives_until_bbox_size_matches_then_advances():
    seq = StageSequence(PROFILE, confirm_frames=3, pause=0.5)
    assert seq.step(obs(0.0, rear=(0.5, 0.5, 0.1, 0.1)), 0.0) == (7, -90, 'ENTRY')
    for i in range(3):
        assert seq.step(obs(0.1*(i+1)), 0.1*(i+1))[:2] == (0, 0)
    assert seq.index == 1
    assert seq.step(obs(0.35), 0.35)[2] == 'STAGE_PAUSE'
    assert seq.step(obs(1.0, rear=(0.5, 0.5, 0.2, 0.2)), 1.0) == (-7, -90, 'ALIGN')


def test_frozen_frames_do_not_confirm():
    seq = StageSequence(PROFILE, confirm_frames=3)
    for t in (0.0, 0.1, 0.2, 0.3):
        seq.step(obs(0.0), t)
    assert seq.index == 0


def test_stale_missing_ambiguous_and_timeout_stop():
    seq = StageSequence(PROFILE, stale_timeout=1.0)
    assert seq.step(obs(0.0), 2.0)[:2] == (0, 0)
    two = obs(0.0)
    two['rear']['boxes'] *= 2
    assert seq.step(two, 0.0)[2].startswith('STOP_AMBIGUOUS')
    assert seq.step({'rear': {'stamp': 0.0, 'boxes': []}}, 0.0)[2].startswith('STOP_MISSING')
    far = (0.5, 0.5, 0.1, 0.1)
    assert seq.step(obs(0.0, rear=far), 0.0)[:2] == (7, -90)
    assert seq.step(obs(3.5, rear=far), 3.5)[2].startswith('STOP_STAGE_TIMEOUT')
    assert seq.step(obs(3.6), 3.6)[:2] == (0, 0)


def test_template_and_bad_commands_rejected():
    with pytest.raises(ValueError):
        validate_profile(dict(PROFILE, tuned=False))
    bad = copy.deepcopy(PROFILE)
    bad['stages'][0]['steering'] = 9
    with pytest.raises(ValueError):
        validate_profile(bad)


def test_normalize_drops_border_boxes_and_merges_duplicates():
    msg = DetectionArray()
    for score, cx, w in ((0.9, 320.0, 100.0), (0.8, 325.0, 110.0), (0.95, 30.0, 70.0)):
        det = Detection(class_name='parking_space', score=score)
        det.bbox.center.position.x, det.bbox.center.position.y = cx, 240.0
        det.bbox.size.x, det.bbox.size.y = w, 60.0
        msg.detections.append(det)
    boxes = normalize(msg, 640, 480, 0.5, 0.5)
    assert len(boxes) == 1 and boxes[0]['score'] == 0.9
