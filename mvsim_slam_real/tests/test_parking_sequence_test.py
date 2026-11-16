"""Fixed-PWM timing, no-device preview, and interruption/serial failure stops."""
import builtins
from pathlib import Path
import sys

import pytest

TOOLS = Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0, str(TOOLS))
import parking_sequence_test as sequence


class Clock:
    def __init__(self):
        self.now = 0.
    def monotonic(self):
        return self.now
    def sleep(self, seconds):
        self.now += seconds


def test_fixed_50_retimes_distance_not_pwm_ratio_and_preserves_stops():
    import json
    original = json.loads(sequence.SOURCE.read_text())
    program = sequence.build_sequence()
    summary = sequence.validate_sequence(program)
    assert summary == {'duration_s': 45.45, 'frames': 14}
    assert program['timing_status'] == 'nominal_unmeasured'
    for source, frame in zip(original['frames'], program['frames']):
        assert source['steering'] == frame['steering']
        if not source['left_pwm']:
            assert frame == source
        else:
            direction = 1 if source['left_pwm'] > 0 else -1
            assert frame['left_pwm'] == frame['right_pwm'] == 50*direction
            old_speed = .12 if direction > 0 else .1
            new_speed = .14 if direction > 0 else .08775
            assert abs(source['ticks']*old_speed-frame['ticks']*new_speed) <= new_speed
    # Nominal forward speed increased, despite changing PWM 60 -> 50.
    assert program['frames'][8]['ticks'] < original['frames'][8]['ticks']
    assert program['frames'][2]['ticks'] > original['frames'][2]['ticks']


@pytest.mark.parametrize('speed', [0, -1, float('nan'), float('inf'), True])
def test_invalid_nominal_speed_rejected(speed):
    with pytest.raises(ValueError):
        sequence.build_sequence(forward=speed)


def test_preview_imports_no_serial_and_opens_no_port(monkeypatch, capsys):
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        assert name != 'serial', 'Preview accessed serial'
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded)
    assert sequence.main([]) == 0
    assert 'PREVIEW ONLY' in capsys.readouterr().out


def test_timed_preview_follows_all_frames_without_writer():
    clock, seen = Clock(), []
    program = sequence.build_sequence()
    sequence.run_sequence(program, monotonic=clock.monotonic, sleep=clock.sleep, progress=seen.append)
    assert clock.now == pytest.approx(45.45)
    assert len(seen) == 15 and '14/14' in seen[-2]


@pytest.mark.parametrize('failure', ['interrupt', 'short_write', 'serial_error', 'late'])
def test_failures_stop_without_recentering_or_catching_up(failure):
    clock, commands = Clock(), []
    def write(command):
        commands.append(command)
        if command == b's1l-50r-50\n':
            if failure == 'interrupt':
                raise KeyboardInterrupt
            if failure == 'short_write':
                return 1
            if failure == 'serial_error':
                raise OSError('USB disconnected')
            clock.now += .3
        return len(command)
    exception = KeyboardInterrupt if failure == 'interrupt' else OSError if failure == 'serial_error' else RuntimeError
    with pytest.raises(exception):
        sequence.run_sequence(sequence.build_sequence(), write, monotonic=clock.monotonic,
                              sleep=clock.sleep, progress=lambda _: None)
    assert commands[-1] == b's1l0r0\n'
    assert commands.count(b's1l-50r-50\n') == 1


def test_complete_transmission_matches_all_ticks_and_ends_stopped():
    clock, commands = Clock(), []
    program = sequence.build_sequence()
    def write(command):
        commands.append(command)
        return len(command)
    sequence.run_sequence(program, write, monotonic=clock.monotonic, sleep=clock.sleep, progress=lambda _: None)
    assert len(commands) == sum(frame['ticks'] for frame in program['frames'])+3
    assert commands[-1] == b's0l0r0\n'


@pytest.mark.parametrize('change', ['pwm', 'steering', 'duration', 'nan', 'direction_dwell', 'steering_dwell', 'end'])
def test_edited_sequence_validation_before_first_write(change):
    program = sequence.build_sequence()
    if change == 'pwm':
        program['frames'][2]['left_pwm'] = -65
    elif change == 'steering':
        program['frames'][2]['steering'] = 8
    elif change == 'duration':
        program['frames'][2]['ticks'] = 1201
    elif change == 'nan':
        program['frames'][2]['ticks'] = float('nan')
    elif change == 'direction_dwell':
        program['frames'][7]['ticks'] = 6
    elif change == 'steering_dwell':
        program['frames'][3]['ticks'] = 1
    elif change == 'end':
        program['frames'][-1]['steering'] = 2
    sent = []
    with pytest.raises(ValueError):
        sequence.run_sequence(program, sent.append)
    assert sent == []


def test_execute_needs_explicit_unmeasured_acknowledgment():
    with pytest.raises(SystemExit):
        sequence.main(['--execute', '--port', '/dev/not-opened'])


def test_main_stop_routes_to_owned_sequence_without_ros():
    import two_structure_test_gui as gui
    from types import SimpleNamespace
    stopped = []
    window = object.__new__(gui.TestWindow)
    window.preview = True
    window.process_kind = 'sequence'
    window.terminate = lambda: stopped.append(True)
    window.ros = SimpleNamespace(call=lambda *_: pytest.fail('Sequence stop called ROS'))
    window.stop()
    assert stopped == [True]


@pytest.mark.parametrize('preview,enabled,confirm,external,expected', [
    (True, True, True, False, 'dry'),
    (False, False, True, False, 'dry'),
    (False, True, True, False, 'hardware'),
    (False, True, False, False, 'blocked'),
    (False, True, True, True, 'blocked'),
])
def test_gui_execution_gate_and_other_sender_exclusion(monkeypatch, preview, enabled, confirm, external, expected):
    import two_structure_test_gui as gui
    from tkinter import messagebox
    from types import SimpleNamespace
    commands, errors = [], []
    window = object.__new__(gui.TestWindow)
    window.process, window.sequence_file = None, None
    window.preview = preview
    window.ros = SimpleNamespace(motion_source_running=lambda: external)
    window.current_settings = lambda: {'mode': 'lidar_manual', 'enable_actuation': enabled,
                                        'arduino_port': '/dev/fake-not-opened'}
    window.set_running = lambda _: None
    window.reader = lambda _: None
    window.append = lambda _: None
    window.process_label = SimpleNamespace(configure=lambda **_: None)
    window.fields = {name: SimpleNamespace(configure=lambda **_: None) for name in ('stage', 'recognition', 'failure')}
    dialog = SimpleNamespace(window=None)
    monkeypatch.setattr(messagebox, 'askyesno', lambda *_args, **_kwargs: confirm)
    monkeypatch.setattr(messagebox, 'showerror', lambda *args, **_: errors.append(args))
    monkeypatch.setattr(gui.os.path, 'exists', lambda _: True)
    monkeypatch.setattr(gui.threading, 'Thread', lambda **_: SimpleNamespace(start=lambda: None))
    def popen(args, **kwargs):
        commands.append(args)
        return SimpleNamespace(args=args)
    monkeypatch.setattr(gui.subprocess, 'Popen', popen)
    try:
        window.launch_sequence(sequence.build_sequence(), dialog)
        if expected == 'blocked':
            assert commands == [] and window.sequence_file is None
        else:
            assert len(commands) == 1
            assert ('--execute' in commands[0]) == (expected == 'hardware')
            assert ('--dry-run' in commands[0]) == (expected == 'dry')
            assert window.sequence_file.exists()
            assert window.process_kind == 'sequence'
    finally:
        if window.sequence_file is not None:
            window.sequence_file.unlink(missing_ok=True)
