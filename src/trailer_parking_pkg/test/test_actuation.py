import math
from trailer_parking_pkg.actuation import StrollerActuation, motion_values
import pytest


def test_matches_original_stroller_ranges():
    limits = StrollerActuation()
    assert motion_values(0.30, 0.60, limits) == (-7, 90, 90)
    assert motion_values(-0.30, -0.60, limits) == (7, -90, -90)
    assert motion_values(0.0, 0.60, limits) == (0, 0, 0)


def test_clamps_to_firmware_limits():
    assert motion_values(10.0, 10.0) == (-7, 255, 255)
    assert motion_values(-10.0, -10.0) == (7, -255, -255)


@pytest.mark.parametrize('speed,steer', [(math.nan, 0), (0, math.inf)])
def test_rejects_non_finite(speed, steer):
    with pytest.raises(ValueError):
        motion_values(speed, steer)
