"""Measured pair geometry and camera confirmation without hardware or ROS."""

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/real_parking'))
from real_parking.two_structure_gap import (
    PairConfig, camera_confirms_pair, observe_two_structure_gap,
)
from tractor_parking_perception import planning_grid


def scene(heading=0.0, side='left'):
    data = np.zeros((260, 360), dtype=np.int16)
    # Two 1.0 x 0.8 m structures, 3.3 m apart, with no rear wall at all.
    data[120:136, 120:140] = 100
    data[120:136, 206:226] = 100
    origin = np.array([-5., -5.])
    if side == 'right':
        data = data[::-1].copy()
        origin[1] = -8.
    c, s = np.cos(heading), np.sin(heading)
    rotation = np.array([[c, -s], [s, c]])
    translation = np.array([2., -1.])
    grid = planning_grid(data, .05, [*(rotation@origin+translation), heading])
    return grid, np.r_[translation, heading]


def lidar_observation(grid, road, config=PairConfig()):
    return observe_two_structure_gap(grid, road, config, require_camera=False)


@pytest.mark.parametrize('heading', [0., .7, -1.2])
def test_pair_without_back_wall_and_sequential_camera_evidence(heading):
    grid, road = scene(heading)
    lidar = lidar_observation(grid, road)
    assert lidar.target is not None, lidar.reason
    assert lidar.target.length == pytest.approx(3.3, abs=.08)
    assert lidar.target.width == 1.10
    assert lidar.target.heading == pytest.approx(heading)
    waiting = observe_two_structure_gap(grid, road)
    assert waiting.target is None and waiting.lidar_target is not None
    # Distinct observations may be taken on different frames during the push/drive.
    first, second = lidar.components
    assert observe_two_structure_gap(grid, road, camera_evidence=[first]).target is None
    confirmed = observe_two_structure_gap(grid, road, camera_evidence=[first, second])
    assert confirmed.target is not None and confirmed.camera_confirmed
    assert confirmed.first_obstacle and confirmed.gap_seen and confirmed.second_obstacle


def test_one_or_wrong_side_structure_does_not_define_a_bay():
    grid, road = scene()
    grid.data[120:136, 206:226] = 0
    assert lidar_observation(grid, road).lidar_target is None
    grid, road = scene(side='right')
    assert lidar_observation(grid, road).lidar_target is None


@pytest.mark.parametrize('value', [-1, 100])
def test_unknown_or_even_one_occupied_interior_cell_is_rejected(value):
    grid, road = scene()
    grid.data[129, 174] = value
    result = lidar_observation(grid, road)
    assert result.target is None


def test_configured_depth_moves_center_without_synthesizing_a_back_wall():
    grid, road = scene()
    before = grid.data.copy()
    small = lidar_observation(grid, road, PairConfig(slot_width=.8)).target
    large = lidar_observation(grid, road, PairConfig(slot_width=1.4)).target
    assert small is not None and large is not None
    assert large.center[1]-small.center[1] == pytest.approx(.3)
    assert small.length == large.length
    np.testing.assert_array_equal(grid.data, before)


def test_one_image_region_spanning_both_structures_cannot_confirm_a_pair():
    grid, road = scene()
    first, second = lidar_observation(grid, road).components
    assert not camera_confirms_pair((first, second), [np.vstack([first, second])])
    assert not camera_confirms_pair((first, second), [first, first.copy()])
    assert not camera_confirms_pair((first, second), [first+10., second+10.])
    assert camera_confirms_pair((first, second), [first, second])


def test_gap_size_and_nonfinite_configuration_are_rejected():
    grid, road = scene()
    assert lidar_observation(grid, road, replace(PairConfig(), min_gap_length=4.)).target is None
    for kwargs in ({'slot_width': float('nan')}, {'slot_width': -.1},
                   {'min_gap_length': 7.}, {'cluster_join_m': 0.}):
        with pytest.raises(ValueError):
            PairConfig(**kwargs)
