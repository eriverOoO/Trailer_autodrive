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


STRICT = replace(PairConfig(), max_speck_cells=0, max_unknown_fraction=0.)


def test_strict_config_rejects_even_one_occupied_interior_cell():
    grid, road = scene()
    grid.data[129, 174] = 100
    assert lidar_observation(grid, road, STRICT).target is None


@pytest.mark.parametrize('cells', [[(129, 174)], [(129, 174), (130, 175)], [(129, 174), (135, 200)]])
def test_isolated_specks_and_single_unknown_cell_are_tolerated(cells):
    grid, road = scene()
    for row, col in cells:
        grid.data[row, col] = 100
    grid.data[133, 190] = -1
    result = lidar_observation(grid, road)
    assert result.target is not None, result.reason
    assert 0 < result.free_fraction < 1


def test_larger_occupied_group_is_rejected():
    grid, road = scene()
    grid.data[129, 174:177] = 100  # three connected cells: an object, not noise
    # With the default min_structure_cells=3 this object is itself a structure candidate and
    # breaks the pair; raise it here so the interior check is what rejects the bay.
    result = lidar_observation(grid, road, replace(PairConfig(), min_structure_cells=5))
    assert result.lidar_target is not None and result.target is None
    assert 'occupied_group_cells=3/2' in result.reason
    # With defaults the 3-cell object is a recessed candidate: skipped for pairing, still an obstacle.
    default = lidar_observation(grid, road)
    assert default.lidar_target is not None and default.target is None and not default.rear_wall


def test_unknown_depth_allows_planning_only_after_both_camera_matches():
    grid, road = scene()
    grid.data[125:145, 160:190] = -1  # ~40% of the interior never observed
    result = lidar_observation(grid, road)
    assert result.target is not None and not result.target.depth_observed
    first, second = result.components
    assert observe_two_structure_gap(grid, road, camera_evidence=[first]).target is None
    matched = observe_two_structure_gap(grid, road, camera_evidence=[first, second])
    assert matched.target is not None and not matched.target.depth_observed


@pytest.mark.parametrize('heading', [0., .7])
def test_shallow_target_extends_only_toward_road(heading):
    grid, road = scene(heading)
    config = PairConfig(slot_width=.50)
    nominal = lidar_observation(grid, road, config).target
    goal = observe_two_structure_gap(grid, road, config, require_camera=False,
                                     minimum_goal_width=.91).target
    assert goal.width == pytest.approx(.91)
    normal = np.array([-np.sin(heading), np.cos(heading)])
    np.testing.assert_allclose(goal.center+normal*goal.width/2,
                               nominal.center+normal*nominal.width/2)
    assert goal.length == nominal.length


def test_speck_touching_a_larger_group_outside_the_interior_is_rejected():
    grid, road = scene()
    # Only one cell lies inside the inset interior, but its group continues past the rear edge.
    grid.data[139:150, 174] = 100
    assert lidar_observation(grid, road).target is None


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
                   {'min_gap_length': 7.}, {'cluster_join_m': 0.}, {'max_speck_cells': -1},
                   {'max_unknown_fraction': 1.}, {'max_unknown_fraction': -.1}):
        with pytest.raises(ValueError):
            PairConfig(**kwargs)


def wall_scene(structure_rows=16, structure_cols=20, wall_gap_rows=0, heading=0.0):
    """Structures standing against a long building wall on the left (0.05 m cells)."""
    data = np.zeros((260, 360), dtype=np.int16)
    top = 120+structure_rows+wall_gap_rows  # wall face row
    data[top:top+3, 20:340] = 100  # 16 m wall, 0.15 m thick
    data[120:120+structure_rows, 120:120+structure_cols] = 100
    data[120:120+structure_rows, 206:206+structure_cols] = 100
    c, s = np.cos(heading), np.sin(heading)
    rotation = np.array([[c, -s], [s, c]])
    translation = np.array([2., -1.])
    origin = np.array([-5., -5.])
    grid = planning_grid(data, .05, [*(rotation@origin+translation), heading])
    return grid, np.r_[translation, heading]


@pytest.mark.parametrize('heading', [0., .5])
def test_structures_flush_against_wall_define_a_shallow_bay_on_the_wall(heading):
    grid, road = wall_scene(heading=heading)
    result = lidar_observation(grid, road)
    assert result.target is not None, result.reason
    assert result.rear_wall and result.rear_depth_m == pytest.approx(.8, abs=.08)
    assert result.target.width == 1.10
    # Inner edge on the wall face (row 136 starts at 1.80 m), bay extending toward the road.
    normal = np.array([-np.sin(heading), np.cos(heading)])
    inner = (result.target.center-road[:2])@normal+.55
    assert inner == pytest.approx(1.80, abs=.04)
    assert len(result.walls) == 1


def test_small_30cm_structures_against_wall_are_split_off():
    grid, road = wall_scene(structure_rows=6, structure_cols=6)
    result = lidar_observation(grid, road, replace(PairConfig(), min_gap_length=3.3))
    assert result.target is not None, result.reason
    assert result.structures == 2 and result.rear_wall
    assert result.rear_depth_m == pytest.approx(.3, abs=.08)


def test_wall_beyond_configured_depth_keeps_configured_bay():
    grid, road = wall_scene(wall_gap_rows=14)  # wall face 0.8+0.7 = 1.5 m behind entrance
    result = lidar_observation(grid, road)
    assert result.target is not None, result.reason
    assert not result.rear_wall and result.rear_depth_m is None


def test_no_pair_reason_names_the_rejected_gap():
    grid, road = scene()
    result = lidar_observation(grid, road, replace(PairConfig(), min_gap_length=4.))
    assert result.target is None and 'gap 3.' in result.reason and result.structures == 2
    assert len(result.components) == 2


def test_short_object_inside_bay_is_not_taken_as_rear_wall():
    grid, road = scene()
    grid.data[140:143, 160:170] = 100  # 0.5 m box half way into the bay
    result = lidar_observation(grid, road)
    assert result.target is None and not result.rear_wall
