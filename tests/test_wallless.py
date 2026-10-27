import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/real_parking'))
from real_parking.wallless import Detector, Config


def scene(unknown=False, obstacle=False):
    # .1m cells, x origin -2, y origin -5; no rear wall.
    grid = np.full((80, 180), -1 if unknown else 0, dtype=int)
    grid[30:36, 40:50] = 100  # x=2..3, y=-2..-1.4
    grid[30:36, 90:100] = 100  # x=7..8; gap=4m
    if obstacle:
        grid[29:31, 66:68] = 100
    return grid


def observe(detector, grid, pose=(0., 0., 0.)):
    return detector.update(grid, .1, [-2., -5., 0.], pose, 3.)


def warmed(grid):
    d = Detector()
    for stamp in (0., 1.):
        d.update(grid, .1, [-2., -5., 0.], [0., 0., 0.], stamp)
    return d


def test_wall_free_gap_requires_temporal_confirmation():
    g = scene(); d = Detector()
    assert not d.update(g, .1, [-2., -5., 0.], [0., 0., 0.], 0.)['ready']
    d.update(g, .1, [-2., -5., 0.], [0., 0., 0.], 1.)
    result = observe(d, g)
    assert result['ready'] and len(result['objects']) == 2
    assert result['gaps'][0]['width_source'] == 'configured_prior'


def test_unknown_and_occupied_gap_rejected():
    for grid in (scene(unknown=True), scene(obstacle=True)):
        assert not observe(warmed(grid), grid)['ready']


def test_duplicate_maps_cannot_confirm_stationary():
    d = Detector(); g = scene()
    for _ in range(5):
        result = d.update(g, .1, [-2., -5., 0.], [0., 0., 0.], 1.)
    assert not result['ready']


def test_ego_motion_does_not_make_static_objects_move():
    g = scene(); result = observe(warmed(g), g, (1., 0., 0.))
    assert result['ready']
    assert all(o['stationary'] for o in result['objects'])


def test_displaced_objects_need_new_confirmation():
    g = scene(); d = warmed(g)
    moved = np.roll(g, 6, axis=1)
    assert not observe(d, moved)['ready']


def test_left_side_and_long_wall_rejected():
    g = scene(); assert not observe(warmed(g), g, (0., -4., 0.))['ready']
    g[30:36, 40:100] = 100
    assert not observe(warmed(g), g)['ready']


def test_expired_tracks_do_not_confirm_stationarity():
    d = warmed(scene())
    assert not d.update(scene(), .1, [-2., -5., 0.], [0., 0., 0.], 20.)['ready']


def test_scan_clustering_splits_invalid_and_depth_edges():
    from real_parking.wallless import scan_clusters
    result = scan_clusters([2.]*6+[float('inf')]+[3.]*6+[6.]*6, -.3, .01, .15, 12.)
    assert len(result) == 3
    assert [c['points'] for c in result] == [6, 6, 6]


def test_rotated_grid_origin_and_road_pose():
    g = scene(); d = Detector()
    origin = [5., -2., np.pi/2]
    pose = [0., 0., np.pi/2]  # transformed original ego about the new origin
    # Original grid origin [-2,-5] -> [5,-2] is a rotation about zero.
    for t in (0., 1., 3.):
        result = d.update(g, .1, origin, pose, t)
    assert result['ready']
