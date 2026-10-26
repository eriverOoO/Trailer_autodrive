"""Check motion expiry and Ackermann sign conventions without opening devices."""
import importlib.util
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'manual_slam_check', Path(__file__).resolve().parents[1] / 'tools/manual_slam_check.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def drive():
    return module.KeyboardDrive(0.08, 0.06, math.radians(20), 0.58, 0.35)


def test_motion_expires_and_unrelated_keys_do_not_extend_it():
    control = drive()
    assert control.command(0) == (0, 0)
    control.key('q', 10)
    assert control.command(10.1)[0] == 0.08
    control.key('a', 10.3)
    assert control.command(10.36) == (0, 0)
    control.key('q', 10.4)
    assert control.command(10.5)[0] == 0.08


def test_reverse_left_reverses_yaw_rate_and_straight_clears_steering():
    control = drive()
    control.key('q', 1)
    forward = control.command(1)
    control.key('z', 2)
    reverse = control.command(2)
    assert forward[1] > 0 > reverse[1]
    assert reverse[1] == pytest.approx(-0.06 * math.tan(math.radians(20)) / 0.58)
    control.key('s', 3)
    assert control.command(3) == (-0.06, 0)


@pytest.mark.parametrize('key', [' ', 'x'])
def test_stop_is_immediate(key):
    control = drive()
    control.key('e', 1)
    control.key(key, 1.01)
    assert control.command(1.01) == (0, 0)


def test_startup_gate_holds_zero_on_wait_or_stale_ready(monkeypatch):
    from types import SimpleNamespace
    control=drive();control.key('w',10.)
    node=SimpleNamespace(args=SimpleNamespace(require_ready=True),ready=False,
                         ready_wall=10.,drive=control)
    monkeypatch.setattr(module.time,'monotonic',lambda:10.1)
    assert module.SlamCheck.current_command(node)==(0.,0.)
    node.ready=True
    assert module.SlamCheck.current_command(node)==(.08,0.)
    node.ready_wall=9.7
    assert module.SlamCheck.current_command(node)==(0.,0.)
    node.args.require_ready=False
    assert module.SlamCheck.current_command(node)==(.08,0.)
