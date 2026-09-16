"""Conversion from metric planner commands to the stroller Arduino contract."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StrollerActuation:
    """Limits used by the existing MotionCommand -> Arduino serial pipeline."""
    max_steer_rad: float = 0.60
    max_steer_step: int = 7
    reference_speed_mps: float = 0.30
    reference_pwm: int = 90
    max_pwm: int = 255

    def __post_init__(self):
        if (not math.isfinite(self.max_steer_rad) or self.max_steer_rad <= 0 or
                not math.isfinite(self.reference_speed_mps) or self.reference_speed_mps <= 0 or
                not 1 <= self.max_steer_step <= 7 or
                not 1 <= self.reference_pwm <= self.max_pwm <= 255):
            raise ValueError('Invalid stroller actuation limits')


def motion_values(speed_mps, steering_rad, limits=StrollerActuation()):
    """Return ``(steering, left_pwm, right_pwm)`` for MotionCommand.

    The supplied stroller firmware accepts steering steps in [-7, 7] and
    signed PWM in [-255, 255]. Equal left/right PWM preserves its Ackermann
    steering arrangement; a negative PWM is reverse.
    """
    if not math.isfinite(speed_mps) or not math.isfinite(steering_rad):
        raise ValueError('Non-finite actuation command')
    steer = round(steering_rad / limits.max_steer_rad * limits.max_steer_step)
    steer = max(-limits.max_steer_step, min(limits.max_steer_step, steer))
    pwm = round(speed_mps / limits.reference_speed_mps * limits.reference_pwm)
    pwm = max(-limits.max_pwm, min(limits.max_pwm, pwm))
    if pwm == 0:
        steer = 0
    return int(steer), int(pwm), int(pwm)
