from types import SimpleNamespace

from simulation_pkg.command_adapter import SendSignal


def test_parking_pwm_matches_metric_planner_command():
    adapter = SendSignal(max_speed=0.30 * 255 / 90, max_steer=0.60,
                         steering_direction=-1)
    steer, left, right = adapter.process(
        SimpleNamespace(steering=-7, left_speed=90, right_speed=90))
    assert steer == 0.60
    assert left == right == 0.30


def test_signed_pwm_is_preserved_for_reverse():
    adapter = SendSignal(max_speed=0.30 * 255 / 90, max_steer=0.60,
                         steering_direction=-1)
    _, left, right = adapter.process(
        SimpleNamespace(steering=-7, left_speed=-90, right_speed=-90))
    assert left == right == -0.30
