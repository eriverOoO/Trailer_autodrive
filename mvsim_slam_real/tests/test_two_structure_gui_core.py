"""Test-window logic without a display: launch arguments, explanations, map rendering."""

import importlib.util
from pathlib import Path
import py_compile
import sys

import numpy as np
import pytest

W = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('two_structure_gui_core', W/'tools/two_structure_gui_core.py')
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)
sys.path.insert(0, str(W/'src/real_parking'))

BASE = dict(core.DEFAULT_SETTINGS, lidar_port='/dev/lidar_g6', tractor_imu_port='/dev/imu_tractor',
            trailer_imu_port='/dev/imu_trailer')


def args(**kwargs):
    return dict(a.split(':=', 1) for a in core.launch_arguments(dict(BASE, **kwargs)))


def test_modes_map_to_launch_arguments():
    lidar_manual = args(mode='lidar_manual')
    assert lidar_manual['require_camera'] == 'false' and lidar_manual['manual_survey'] == 'true'
    assert lidar_manual['allow_lidar_only_motion'] == 'true' and lidar_manual['enable_actuation'] == 'false'
    lidar_auto = args(mode='lidar_auto')
    assert lidar_auto['require_camera'] == 'false' and lidar_auto['allow_lidar_only_motion'] == 'true'
    camera = args(mode='camera_auto', camera_path='/dev/video0', calibration_file='/c.json')
    assert camera['require_camera'] == 'true' and camera['camera_path'] == '/dev/video0'
    assert camera['detector_backend']=='ride_on_onnx' and camera['detector_confidence']=='0.65'
    assert all(a['development_mode'] == 'false' and a['start_g6_driver'] == 'true'
               for a in (lidar_manual, lidar_auto, camera))


@pytest.mark.parametrize('mode', core.MODES)
def test_compact_selectors_preserve_launch_mode_and_start_service(mode):
    assert core.mode_from_choices(*core.mode_choices(mode)) == mode
    service, label = core.primary_action(mode)
    manual = args(mode=mode, camera_path='/dev/video0', calibration_file='/c.json').get('manual_survey') == 'true'
    assert service == ('start_parking' if manual else 'start_search')
    assert label == ('주차 시작' if manual else '자동 탐색 시작')


def test_preview_and_mismatched_start_cannot_send_commands():
    from types import SimpleNamespace
    sys.path.insert(0, str(W/'tools'))
    import two_structure_test_gui as gui
    calls, responses = [], []
    window = object.__new__(gui.TestWindow)
    window.preview = True
    window.ros = SimpleNamespace(call=lambda *args: calls.append(args))
    window.fields = {'response': SimpleNamespace(config=lambda **kw: responses.append(kw))}
    window.running_settings = {'mode': 'camera_manual'}
    window.service('stop')
    window.service('start_parking')
    assert calls == []
    window.preview = False
    window.service('start_search')
    assert calls == [] and '맞지 않는' in responses[-1]['text']
    # Dispatch uses the launched mode, even if displayed settings were changed externally.
    window.current_settings = lambda: {'mode': 'camera_auto'}
    window.start_action()
    assert calls[-1][0] == 'start_parking'


def test_motors_off_by_default_and_need_arduino_when_enabled():
    assert args(mode='lidar_auto')['enable_actuation'] == 'false'
    # Hand-push modes may hand over to motor parking after [주차 시작].
    assert args(mode='lidar_manual', enable_actuation=True, arduino_port='/dev/ttyACM0')['enable_actuation'] == 'true'
    with pytest.raises(ValueError, match='arduino_port'):
        args(mode='lidar_auto', enable_actuation=True)
    auto = args(mode='lidar_auto', enable_actuation=True, arduino_port='/dev/ttyACM0')
    assert auto['enable_actuation'] == 'true' and auto['arduino_port'] == '/dev/ttyACM0'


@pytest.mark.parametrize('settings', [{'lidar_port': ''}, {'mode': 'camera_auto'}, {'slot_width': 'abc'},
                                      {'slot_width': '9'}, {'mode': 'nope'}])
def test_invalid_settings_rejected(settings):
    with pytest.raises(ValueError):
        core.launch_arguments(dict(BASE, **settings))


def test_shell_command_quotes_arguments():
    command = core.launch_shell('/home/a b/ws', core.launch_arguments(dict(BASE, slot_width='1.2')))
    assert "cd '/home/a b/ws'" in command and 'two_structure_parking_test.launch.py' in command
    assert 'slot_width:=1.200' in command


def test_camera_model_override_and_bad_confidence():
    values=args(mode='camera_manual',camera_path='/dev/video0',calibration_file='/c.json',
                detector_model='/models/custom.onnx',detector_confidence='.7')
    assert values['detector_model']=='/models/custom.onnx' and values['detector_confidence']=='0.7'
    for confidence in ['nan','0','1','invalid']:
        with pytest.raises(ValueError):
            args(mode='camera_manual',camera_path='/dev/video0',calibration_file='/c.json',
                 detector_confidence=confidence)


def test_explanations_cover_real_detection_reasons():
    from dataclasses import replace
    from real_parking.two_structure_gap import PairConfig, observe_two_structure_gap
    from tractor_parking_perception import planning_grid
    data = np.zeros((260, 360), dtype=np.int16)
    data[120:136, 120:140] = 100
    data[120:136, 206:226] = 100
    grid = planning_grid(data, .05, [-5., -5., 0.])
    road = np.zeros(3)
    reasons = [observe_two_structure_gap(grid, road, PairConfig(), require_camera=False).reason,
               observe_two_structure_gap(grid, road, PairConfig()).reason,
               observe_two_structure_gap(grid, road, replace(PairConfig(), min_gap_length=4.)).reason]
    grid.data[125:145, 160:190] = -1
    reasons.append(observe_two_structure_gap(grid, road, require_camera=False).reason)
    grid.data[:] = 0
    reasons.append(observe_two_structure_gap(grid, road).reason)
    for reason in reasons:
        summary, _ = core.explain(reason)
        assert summary != reason, reason  # every one has a Korean explanation
    assert '0' in core.explain(reasons[-1])[0]


def test_unknown_text_passes_through():
    assert core.explain('something new') == ('something new', '')


def test_render_view_keeps_thin_walls_and_y_up():
    data = np.full((100, 200), -1, dtype=np.int16)   # 10 x 5 m map, origin (-5, -5)
    data[0, :] = 100                                  # one-cell wall along the bottom edge (y = -5)
    data[99, 150] = 0                                 # one free cell at the top
    rgb, view = core.render_view(data, .05, [-5., -5., 0.], [0., -2.5], 20., 100)
    assert rgb.shape == (100, 100, 3) and view.mpp == pytest.approx(.2)
    (px, py), = view.pixel([[0., -4.99]])
    assert (rgb[int(py), int(px)] == core.COLORS[core.OCCUPIED]).all()   # wall survives 4x reduction
    assert (rgb[5, 50] == core.COLORS[core.OUTSIDE]).all()               # beyond the map
    assert (rgb[int(view.pixel([[0., -2.]])[0, 1]), 50] == core.COLORS[core.UNKNOWN]).all()
    assert core.ppm(rgb).startswith(b'P6 100 100 255 ')


def test_render_view_handles_origin_yaw():
    data = np.zeros((40, 40), dtype=np.int16)
    data[0:4, 30:34] = 100                            # grid +x end, rotated 90 deg -> world +y
    rgb, view = core.render_view(data, .05, [0., 0., np.pi/2], [0., 0.], 4., 200)
    (px, py), = view.pixel([[-.1, 1.6]])
    assert (rgb[int(py), int(px)] == core.COLORS[core.OCCUPIED]).all()


def test_map_view_pixel_is_centred_and_y_up():
    view = core.MapView([1., 2.], 10., 100)
    assert view.pixel([[1., 2.], [6., 7.]]).tolist() == [[50., 50.], [100., 0.]]


def test_gui_script_compiles():
    py_compile.compile(str(W/'tools/two_structure_test_gui.py'), doraise=True)


@pytest.mark.skipif(not sys.platform.startswith('linux'), reason='prctl is Linux only')
def test_launch_child_is_stopped_when_the_window_process_dies(tmp_path):
    import os
    import subprocess
    import time
    parent = tmp_path/'parent.py'
    parent.write_text(
        'import subprocess, sys\n'
        f'sys.path.insert(0, {str(W/"tools")!r})\n'
        'import two_structure_test_gui as gui\n'
        'child = subprocess.Popen(["sleep", "30"], preexec_fn=gui.child_setup)\n'
        'print(child.pid, flush=True)\n')
    pid = int(subprocess.run([sys.executable, str(parent)], capture_output=True, text=True, check=True).stdout)
    for _ in range(50):
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            break
        time.sleep(.05)
    else:
        os.kill(pid, 9)
        pytest.fail('launch child survived its parent')


def test_placement_guide_has_distances_height_and_clearance():
    height = core.lidar_height(W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml')
    assert height == pytest.approx(.687)
    auto = core.placement_guide('lidar_auto', height)
    assert '0.69 m' in auto and '0.5~1.2 m' in auto and '1~10 m' in auto and '0.18 m 안' in auto
    manual = core.placement_guide('camera_manual', height)
    assert '손으로 밀 때' in manual and '멈추지 않음' in manual and '1 m 이상 떨어지기' in manual
    assert 'LiDAR 스캔 높이보다' in core.placement_guide('lidar_auto', None)
