"""Check that camera parking selects the learned backend and its isolated Python."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from launch import LaunchContext

W=Path(__file__).resolve().parents[1]


def module(name):
    spec=importlib.util.spec_from_file_location(name,W/'src/mvsim_slam_real_bringup/launch'/f'{name}.launch.py')
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(**kwargs):
    result=LaunchContext()
    result.launch_configurations.update(kwargs)
    return result


def test_vision_prefix_preserves_virtualenv_symlink_path(tmp_path,monkeypatch):
    mod=module('parking_vision_diagnostic')
    model=tmp_path/'ride_on.onnx';model.write_bytes(b'model checked by node')
    interpreter=tmp_path/'runtime_python';interpreter.symlink_to('/usr/bin/python3')
    mod.Node=Mock(side_effect=lambda **kwargs:kwargs)
    actions=mod.nodes(context(camera_path='',image_topic='/replay/image',scan_topic='/replay/scan',
        detector_params='',calibration_file='',detector_backend='ride_on_onnx',
        detector_model=str(model),vision_python=str(interpreter),detector_confidence='.65'))
    diagnostic=actions[0]
    assert diagnostic['prefix']==[str(interpreter)]
    assert diagnostic['parameters'][0]['detector_backend']=='ride_on_onnx'
    assert diagnostic['parameters'][0]['detector_model']==str(model)
    assert diagnostic['parameters'][0]['reliable_image'] is True


def test_missing_learned_model_does_not_start_a_heuristic_node(tmp_path):
    mod=module('parking_vision_diagnostic')
    with pytest.raises(RuntimeError,match='ONNX model missing'):
        mod.nodes(context(camera_path='',image_topic='/replay/image',scan_topic='/replay/scan',
            detector_params='',calibration_file='',detector_backend='ride_on_onnx',
            detector_model=str(tmp_path/'missing.onnx'),vision_python='/usr/bin/python3'))


@pytest.mark.parametrize('require,camera_path,backend,labels', [
    ('true','','ride_on_onnx',['ride_on_car']),
    ('false','/dev/NOT_OPENED','ride_on_onnx',['ride_on_car']),
    ('false','','heuristic',['cone','white body'])])
def test_parking_launch_class_filter_matches_selected_detector(monkeypatch,require,camera_path,backend,labels):
    mod=module('two_structure_parking_test')
    base=SimpleNamespace(enabled=lambda ctx,name:ctx.launch_configurations.get(name,'false')=='true',
                         launch_nodes=Mock(return_value=[]))
    monkeypatch.setattr(mod,'base_launch',lambda:base)
    ctx=context(enable_vision='false',development_mode='false',require_camera=require,
        camera_path=camera_path,detector_backend='auto',detector_confidence='.65',
        test_params_file='/not/read/in/unit/test.yaml',manual_survey='true',allow_lidar_only_motion='false',
        slot_width='',start_stack='true',start_g6_driver='false',image_topic='/replay/image',
        calibration_file='',detector_params='',detector_model='',vision_python='',image_reliability='auto')
    actions=mod.nodes(ctx)
    parameters=base.launch_nodes.call_args.kwargs['parking_parameters'][1]
    assert parameters['camera_object_labels']==labels
    assert parameters['camera_min_confidence']==(.65 if backend=='ride_on_onnx' else 0.)
    assert actions  # Included diagnostic launch, not executed in this test.
