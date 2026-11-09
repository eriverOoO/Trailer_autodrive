"""ROS-independent geometry regressions for the real vehicle's U-bay detector."""
from pathlib import Path
import math
import sys
import unittest

import numpy as np

VENDOR = Path(__file__).resolve().parents[1] / 'src/real_parking/real_parking/vendor'
sys.path.insert(0, str(VENDOR))
from tractor_local_parking_gap_detector import observe_gap
from tractor_parking_perception import planning_grid
from tractor_trailer_model import transform


def bay_grid(side, road_heading=0., bay_angle=0., interior=0, *, entrance_depth=2.):
    """Rasterize a measured 3.3 x 1.1 m opening, with no exterior walls."""
    origin = np.array([-2., -6.])
    translation = np.array([3., -1.])
    grid = planning_grid(np.zeros((240, 240), dtype=int), .05,
                         [*transform([origin], translation, road_heading)[0], road_heading])
    jj, ii = np.indices(grid.data.shape)
    road_xy = np.c_[(ii.ravel()+.5)*.05+origin[0],
                    (jj.ravel()+.5)*.05+origin[1]]
    bay_xy = transform(road_xy, [0, 0], -bay_angle)
    u, d = bay_xy[:, 0], bay_xy[:, 1] * (1 if side == 'left' else -1)
    d += 2. - entrance_depth
    def rect(x0, x1, d0, d1):
        return (u >= x0) & (u < x1) & (d >= d0) & (d < d1)
    occupied = (rect(0, 2, 2, 2.15) | rect(5.3, 7.3, 2, 2.15)
                | rect(1.85, 2, 2, 3.1) | rect(5.3, 5.45, 2, 3.1)
                | rect(0, 7.3, 3.1, 3.25))
    grid.data.flat[occupied] = 100
    if interior:
        grid.data.flat[rect(2.4, 4.9, 2.25, 2.95)] = interior
    # Rebuild collision/free-space metadata after rasterization.
    return planning_grid(grid.data, grid.resolution, grid.origin), np.array([*translation, road_heading])


class GapSideTests(unittest.TestCase):
    def test_selected_side_and_world_coordinates(self):
        for side in ('left', 'right'):
            for road_yaw, bay_yaw in ((0., 0.), (.7, .05), (-1.2, -.05)):
                with self.subTest(side=side, road_yaw=road_yaw, bay_yaw=bay_yaw):
                    grid, road = bay_grid(side, road_yaw, bay_yaw)
                    obs = observe_gap(grid, road, 3.36, 1.10, min_length=1.9, parking_side=side)
                    self.assertIsNotNone(obs.target, obs.reason)
                    self.assertTrue(obs.first_obstacle and obs.gap_seen and obs.second_obstacle)
                    expected = transform([[3.65, 2.55 if side == 'left' else -2.55]],
                                         road[:2], road_yaw+bay_yaw)[0]
                    np.testing.assert_allclose(obs.target.center, expected, atol=.12)
                    # The existing coverage-based ranking has a few degrees of
                    # raster bias; exact reflection symmetry is checked below.
                    self.assertAlmostEqual(obs.target.heading, road_yaw+bay_yaw, delta=math.radians(3))
                    self.assertAlmostEqual(obs.target.length, 3.3, delta=.15)
                    self.assertAlmostEqual(obs.target.width, 1.1, delta=.15)
                    other = 'right' if side == 'left' else 'left'
                    self.assertIsNone(observe_gap(grid, road, 3.36, 1.10, parking_side=other).target)

    def test_occupied_and_unknown_interior_still_rejected(self):
        for side in ('left', 'right'):
            for value in (100, -1):
                with self.subTest(side=side, interior=value):
                    grid, road = bay_grid(side, interior=value)
                    self.assertIsNone(observe_gap(grid, road, 3.36, 1.10,
                                                 min_length=1.9, parking_side=side).target)

    def test_real_profile_detects_near_entrance(self):
        grid, road = bay_grid('left', entrance_depth=.98)
        legacy = observe_gap(grid, road, 3.36, 1.10, min_length=1.9, parking_side='left')
        self.assertIsNone(legacy.target)
        near = observe_gap(grid, road, 3.36, 1.10, min_length=1.9,
                           parking_side='left', min_lateral_m=.55)
        self.assertIsNotNone(near.target, near.reason)
        np.testing.assert_allclose(near.target.center-road[:2], [3.65, 1.53], atol=.12)

    def test_mirrored_bays_produce_mirrored_targets(self):
        for angle in (0., .05, -.05):
            right_grid, road = bay_grid('right', bay_angle=angle)
            left_grid, _ = bay_grid('left', bay_angle=-angle)
            right = observe_gap(right_grid, road, 3.36, 1.10, parking_side='right').target
            left = observe_gap(left_grid, road, 3.36, 1.10, parking_side='left').target
            self.assertIsNotNone(right)
            self.assertIsNotNone(left)
            np.testing.assert_allclose(left.center-road[:2],
                                       (right.center-road[:2])*[1, -1], atol=1e-9)
            self.assertAlmostEqual(left.heading, -right.heading)
            self.assertAlmostEqual(left.length, right.length)
            self.assertAlmostEqual(left.width, right.width)

    def test_legacy_function_default_and_invalid_side(self):
        grid, road = bay_grid('right')
        self.assertIsNotNone(observe_gap(grid, road, 3.36, 1.10).target)
        with self.assertRaisesRegex(ValueError, 'parking_side'):
            observe_gap(grid, road, parking_side='Left')


if __name__ == '__main__':
    unittest.main()
