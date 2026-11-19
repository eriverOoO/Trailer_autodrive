"""Check model identity, letterboxing, NMS and optional confidence metadata."""
from types import SimpleNamespace
import sys
from unittest.mock import Mock

import numpy as np
import pytest

from camera_perception_pkg.ride_on_detector import RideOnOnnxDetector, letterbox
from camera_perception_pkg.parking_objects import Candidate, associate_candidates


def test_letterbox_preserves_bgr_to_rgb_and_padding():
    image = np.full((480, 640, 3), [10, 20, 30], dtype=np.uint8)
    tensor, scale, pad = letterbox(image, 416)
    assert tensor.shape == (1, 3, 416, 416) and tensor.dtype == np.float32
    assert scale == .65 and pad == (0, 52)
    np.testing.assert_allclose(tensor[0, :, 100, 100], np.array([30, 20, 10])/255.)
    np.testing.assert_allclose(tensor[0, :, 0, 0], 114/255.)


@pytest.fixture
def fake_runtime(tmp_path, monkeypatch):
    path = tmp_path/'model.onnx'
    path.write_bytes(b'test model placeholder; runtime is mocked')
    session = Mock()
    session.get_inputs.return_value = [SimpleNamespace(shape=[1, 3, 416, 416], type='tensor(float)', name='images')]
    session.get_outputs.return_value = [SimpleNamespace(shape=[1, 5, 3])]
    session.get_modelmeta.return_value = SimpleNamespace(custom_metadata_map={'names': "{0: 'ride_on_car'}", 'task':'detect'})
    session.run.return_value = [np.array([[[208, 209, 30], [208, 209, 30], [100, 100, 10],
                                          [100, 100, 10], [.9, .8, .1]]], dtype=np.float32)]
    runtime = SimpleNamespace(SessionOptions=SimpleNamespace, InferenceSession=Mock(return_value=session))
    monkeypatch.setitem(sys.modules, 'onnxruntime', runtime)
    return path, session


def test_decodes_one_class_suppresses_duplicates_and_unpads(fake_runtime):
    path, _ = fake_runtime
    result = RideOnOnnxDetector(path).detect(np.zeros((480, 640, 3), dtype=np.uint8))
    assert len(result) == 1 and result[0].label == 'ride_on_car'
    assert result[0].confidence == pytest.approx(.9)
    np.testing.assert_allclose(result[0].contour.mean(axis=0), [320, 240], atol=1)


@pytest.mark.parametrize('names,task', [("{0: 'car'}",'detect'), ("{0: 'ride_on_car', 1: 'car'}",'detect'),
                                       ("{0: 'ride_on_car'}",'segment'), ('invalid','detect')])
def test_rejects_wrong_class_or_model_task(fake_runtime, names, task):
    path, session = fake_runtime
    session.get_modelmeta.return_value = SimpleNamespace(custom_metadata_map={'names':names,'task':task})
    with pytest.raises(ValueError):
        RideOnOnnxDetector(path)


def test_backend_failure_does_not_fall_back_to_heuristics(fake_runtime):
    path, session = fake_runtime
    detector = RideOnOnnxDetector(path)
    session.run.side_effect = Exception('inference failure')
    with pytest.raises(RuntimeError, match='inference failed'):
        detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))


def test_association_cannot_bridge_missing_beams_and_preserves_confidence():
    candidate = Candidate('ride_on_car', np.array([[0,0],[100,0],[100,100],[0,100]],dtype=np.int32), .91)
    pixels = np.array([[20,20],[21,20],[22,20],[23,20],[24,20],[25,20]])
    xy = np.array([[2.,i*.01] for i in range(6)])
    result = associate_candidates([candidate], pixels, xy, beam_indices=[0,1,2,4,5,6])
    assert len(result)==1 and len(result[0]['scan_xy'])==3
    assert result[0]['confidence']==.91
    assert associate_candidates([candidate], pixels[:3], xy[:3], beam_indices=[0,2,4])==[]
