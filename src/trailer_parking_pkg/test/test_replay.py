"""CPU-only integration of planning commands, calibration and fail-stop replay."""
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/trailer_parking_pkg'))
sys.path.insert(0, str(ROOT / 'src/serial_communication_pkg'))
from trailer_parking_pkg.core import State, Step, Slot, advance, rectangle
from trailer_parking_pkg.replay import (Platform, DT, compile_path, save_json,
    transmit, validate_program)
from serial_communication_pkg.protocol import encode_command


@pytest.fixture
def profile():
    return json.loads((ROOT / 'src/trailer_parking_pkg/config/ride_on_platform.example.json').read_text())


def program(profile, forward_first=False):
    platform = Platform(profile)
    slot = Slot(0, 0, 0, 3, 1.2)
    state = State(1.5, 0, 0, 0)
    snapshot = dict(schema='parking_snapshot_v1', geometry=asdict(platform.g),
                    start=asdict(state), slot=asdict(slot),
                    scene=dict(obstacles=[], bounds=[-10, 10, -10, 10], margin=.1))
    path = [Step(state, 0, 0)]
    if forward_first:
        for _ in range(5):
            state = advance(state, .05, 0, platform.g)
            path.append(Step(state, 1, 0))
    target = slot.goal(platform.g)
    while state.x > target.x + .01:
        state = advance(state, -.05, 0, platform.g)
        path.append(Step(state, -1, 0))
    return compile_path(path, snapshot, profile)


def test_complete_reverse_program_uses_measured_reverse_pair(profile):
    result = program(profile)
    assert result['prediction']['model_goal_reached']
    moving = [f for f in result['frames'] if f['left_pwm']]
    assert moving and all((f['left_pwm'], f['right_pwm']) == (-65, -65) for f in moving)
    assert result['frames'][-1]['steering'] == result['frames'][-1]['left_pwm'] == 0


def test_direction_cusp_stops_and_missing_stop_is_rejected(profile):
    result = program(profile, forward_first=True)
    moving = [f for f in result['frames'] if f['left_pwm']]
    assert moving[0]['left_pwm'] > 0 and moving[-1]['left_pwm'] < 0
    seen_forward = False
    for frame in result['frames']:
        if frame['left_pwm'] > 0:
            seen_forward = True
        if seen_forward and frame['left_pwm'] == 0:
            frame['ticks'] = 1
            break
    with pytest.raises(ValueError, match='settling'):
        validate_program(result)


def test_nonlinear_steering_and_no_simulator_geometry_defaults(profile):
    profile['steering_radians']['-1'] = .12
    assert Platform(profile).steering(.13) == -1
    del profile['geometry']['wheelbase']
    with pytest.raises(ValueError, match='explicitly'):
        Platform(profile)


def test_example_rejected_before_hardware_io(profile):
    result = program(profile)
    with pytest.raises(ValueError, match='Example calibration'):
        transmit(result, None, None, None, None)


def test_geometry_mismatch_and_collision_are_rejected(profile):
    result = program(profile)
    result['snapshot']['geometry'] = dict(result['snapshot']['geometry'], wheelbase=.6)
    with pytest.raises(ValueError, match='differs'):
        validate_program(result)
    result = program(profile)
    result['snapshot']['scene']['obstacles'] = [rectangle(1.5, 0, 0, .1, .1, .2)]
    with pytest.raises(ValueError, match='footprint'):
        validate_program(result)


@pytest.mark.parametrize('field,value', [('ticks', 0), ('ticks', 999999),
    ('steering', 1.5), ('steering', 8), ('left_pwm', -256), ('right_pwm', 60)])
def test_malformed_command_file_rejected(profile, field, value):
    result = program(profile)
    moving = next(f for f in result['frames'] if f['left_pwm'])
    moving[field] = value
    with pytest.raises(ValueError):
        validate_program(result)


class Clock:
    now = 0.0
    def monotonic(self): return self.now
    def sleep(self, duration): self.now += duration


class Port:
    def __init__(self, clock, delay=0):
        self.clock, self.delay, self.writes = clock, delay, []
    def write(self, packet):
        self.writes.append((self.clock.now, packet))
        self.clock.now += self.delay
        return len(packet)
    def flush(self): pass


def test_replay_20hz_and_terminal_stop_with_fake_io(profile):
    profile['calibration_status'] = 'measured'  # fake I/O only, no real calibration claim
    result = program(profile)
    clock = Clock()
    port = Port(clock)
    transmit(result, port, encode_command, clock.monotonic, clock.sleep)
    times = [t for t, _ in port.writes]
    assert all(b-a == pytest.approx(DT) for a, b in zip(times, times[1:]))
    assert port.writes[-1][1] == b's0l0r0\n'
    assert clock.now == pytest.approx(result['prediction']['duration_s'])


def test_overrun_stops_without_burst_catchup(profile):
    profile['calibration_status'] = 'measured'
    clock = Clock()
    port = Port(clock, delay=.25)
    with pytest.raises(RuntimeError, match='overrun'):
        transmit(program(profile), port, encode_command, clock.monotonic, clock.sleep)
    assert port.writes[-1][1] == b's0l0r0\n'


def test_cli_replay_defaults_to_preview_without_serial_import(profile, tmp_path, monkeypatch, capsys):
    spec = importlib.util.spec_from_file_location('parking_replay_cli', ROOT / 'tools/parking_replay.py')
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    # Any import of serial in default mode would fail.
    monkeypatch.setitem(sys.modules, 'serial', None)
    path = tmp_path / 'commands.json'
    save_json(path, program(profile))
    assert cli.main(['replay', str(path)]) == 0
    assert 'no serial port opened' in capsys.readouterr().out


def test_curved_plan_to_commands_without_scipy_ros_gpu_or_serial(profile, monkeypatch):
    from trailer_parking_pkg.replay import build_program
    for name in ('scipy', 'numpy', 'rclpy', 'serial', 'torch', 'mvsim'):
        monkeypatch.setitem(sys.modules, name, None)
    snapshot = json.loads((ROOT / 'src/trailer_parking_pkg/config/parking_snapshot.example.json').read_text())
    result = build_program(snapshot, profile, timeout=5)
    assert result['prediction']['model_goal_reached']
    assert {1 if f['left_pwm'] > 0 else -1 for f in result['frames'] if f['left_pwm']} == {-1, 1}
    assert any(f['steering'] for f in result['frames'] if f['left_pwm'])
    assert all(p['steer'] in profile['steering_radians'].values() for p in result['planned_path'])


def test_continuous_simulator_steering_cannot_be_silently_rounded(profile):
    p = Platform(profile)
    slot = Slot(0, 0, 0, 3, 1.2)
    target = slot.goal(p.g)
    start = State(target.x-.2, 0, 0, 0)
    snapshot = dict(schema='parking_snapshot_v1', geometry=asdict(p.g),
                    start=asdict(start), slot=asdict(slot),
                    scene=dict(obstacles=[], bounds=[-10, 10, -10, 10], margin=.1))
    with pytest.raises(ValueError, match='Replan with calibrated'):
        compile_path([Step(start, 0, 0), Step(target, 1, .01234)], snapshot, profile)
