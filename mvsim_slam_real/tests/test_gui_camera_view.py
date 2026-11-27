"""Camera freshness/recognition and map scan conversion without a display."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import two_structure_gui_core as core


def association(**overrides):
    return dict(detector_backend='ride_on_onnx',image_stamp_ns=10**9,
                image_candidates=[{'label':'ride_on_car','confidence':.92}],
                calibrated=False,valid=False,objects=[],**overrides)


def test_live_video_is_not_calibrated_correspondence():
    state=core.camera_status(association(),.05,.04,27.,10**9)
    assert state['live'] and '27.0 FPS' in state['stream']
    assert '1개' in state['detection'] and '92%' in state['detection']
    assert state['association']=='보정 없음 · 라이다 대응 전'


def test_disconnect_or_old_detection_cannot_keep_positive_status():
    stale=core.camera_status(association(),1.,.04,27.,10**9)
    assert not stale['live'] and '만료' in stale['detection']
    live=core.camera_status(association(),.03,2.,27.,10**9)
    assert live['live'] and '끊김' in live['detection'] and '92%' not in live['detection']
    wrong=core.camera_status(association(),.03,.03,27.,2*10**9)
    assert '갱신 대기' in wrong['detection'] and '92%' not in wrong['detection']


def test_valid_correspondence_is_separate_from_no_detection():
    packet=association();packet.update(calibrated=True,valid=True,objects=[{'label':'ride_on_car'}],scan_stamp_ns=980_000_000)
    state=core.camera_status(packet,.03,.03,27.,10**9)
    assert '후보 1개' in state['association'] and '20 ms' in state['association']
    packet['image_candidates']=[];packet['objects']=[]
    state=core.camera_status(packet,.03,.03,27.,10**9)
    assert state['detection']=='차체 미검출' and '대응하는 점 없음' in state['association']


@pytest.mark.parametrize('encoding', ['bgr8','8UC3'])
def test_padded_bgr_rows_and_aspect_ratio(encoding):
    message=SimpleNamespace(encoding=encoding,width=2,height=1,step=8,
                            data=bytes([0,0,255,255,0,0,99,99]))
    result=core.camera_rgb(message,4,4)
    np.testing.assert_array_equal(result[1,0],[255,0,0])
    np.testing.assert_array_equal(result[1,3],[0,0,255])
    np.testing.assert_array_equal(result[0,0],[23,33,43])
    message.data=bytes([0,0,255])
    with pytest.raises(ValueError,match='잘렸'):
        core.camera_rgb(message)


def test_scan_points_use_real_transform_and_reject_invalid_returns():
    scan=SimpleNamespace(ranges=[1.,float('inf'),0.,20.],range_min=.1,range_max=12.,angle_min=0.,angle_increment=.1)
    points=core.scan_points(scan,[2.,3.],[0.,0.,np.sin(np.pi/4),np.cos(np.pi/4)])
    np.testing.assert_allclose(points,[[2.,4.]],atol=1e-8)


def test_manual_banner_requires_stop_and_alarms_until_stationary():
    packet = dict(state='stop', stop_requested=True, stationary=False, reason='공간 확인')
    text, _, alarm = core.manual_guidance_status(packet)
    assert '밀기 중지' in text and alarm
    packet['stationary'] = True
    text, _, alarm = core.manual_guidance_status(packet)
    assert '정지 확인' in text and not alarm
    assert core.manual_guidance_status(packet, receipt_age=2.)[2]
    assert core.manual_guidance_status(packet, stamp_age=2.)[2]
    assert core.manual_guidance_status(packet, stamp_age=-2.)[2]


def test_manual_banner_does_not_tell_operator_to_push_without_active_node():
    assert not core.manual_guidance_status(None)[2]
    assert not core.manual_guidance_status(dict(state='inactive'), receipt_age=2.)[2]
    searching = dict(state='searching', reason='공간 탐색 중')
    assert not core.manual_guidance_status(searching)[2]
    assert core.manual_guidance_status(searching, receipt_age=2.)[2]
