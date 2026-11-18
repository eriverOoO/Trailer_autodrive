"""Virtual wall behind the confirmed two-structure bay (planning/execution grids only)."""

from pathlib import Path
import math
import sys
import time
from dataclasses import replace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('rclpy')
W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W/'src/real_parking'))
from real_parking.bootstrap import install_b
from real_parking.two_structure_test import TwoStructureParkingTest, block_behind_bay, open_bay_unknown
from real_parking.two_structure_gap import PairConfig, observe_two_structure_gap
from real_vehicle_bridge.geometry import VehicleGeometry
from tractor_parking_perception import Target, planning_grid
from tractor_trailer_model import Footprints, Geometry, transform

GEOMETRY = W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'
RES = .05


def depth_of(points, target):
    normal = np.array([-math.sin(target.heading), math.cos(target.heading)])
    return (np.asarray(points)-target.center)@normal


def cell_centres(shape, origin):
    rows, cols = np.indices(shape)
    return transform(np.c_[(cols.ravel()+.5)*RES, (rows.ravel()+.5)*RES], origin[:2], origin[2])


@pytest.mark.parametrize('heading,origin_yaw', [(0., 0.), (.6, 0.), (0., -.8), (2.5, 1.1)])
def test_only_cells_deeper_than_inner_edge_are_blocked(heading, origin_yaw):
    data = np.full((120, 120), 0, dtype=np.int16)
    data[5:10, 5:10] = -1
    origin = np.array([-3., -3., origin_yaw])
    target = Target(center=np.array([.3, -.2]), heading=heading, length=3., width=1.1)
    depth = depth_of(cell_centres(data.shape, origin), target).reshape(data.shape)
    # Existing occupied cells on both sides of the inner edge.
    data[np.unravel_index(np.argmin(abs(depth-1.)), data.shape)] = 100
    data[np.unravel_index(np.argmin(abs(depth+1.)), data.shape)] = 70
    original = data.copy()
    out = block_behind_bay(data, RES, origin, target)
    np.testing.assert_array_equal(data, original)  # input not modified
    deep = depth > .55
    assert deep.any() and (~deep).any()
    assert (out[deep & (data < 50)] == 100).all()
    # Bay interior, road side, unknown and occupied cells there are unchanged.
    np.testing.assert_array_equal(out[~deep], data[~deep])
    assert (out[(data >= 50)] == data[data >= 50]).all()
    # Margin moves the wall further away from the bay.
    margin = block_behind_bay(data, RES, origin, target, .3)
    np.testing.assert_array_equal(margin[depth <= .85], data[depth <= .85])
    assert (margin[(depth > .85) & (data < 50)] == 100).all()


@pytest.mark.parametrize('margin', [-.1, float('nan'), float('inf')])
def test_invalid_margin_rejected(margin):
    target = Target(center=np.zeros(2), heading=0., length=3., width=1.1)
    with pytest.raises(ValueError):
        block_behind_bay(np.zeros((4, 4)), RES, [0., 0., 0.], target, margin)


def scene(gap, wall=False):
    """Road at (0,0) heading +x; two 0.99 x 0.55 m structures on the left.

    wall=False: nothing behind. wall=True: a long building wall flush behind them.
    """
    origin = np.array([-4., -3., 0.])
    data = np.zeros((160, 400), dtype=np.int16)
    centres = cell_centres(data.shape, origin).reshape(*data.shape, 2)
    x, y = centres[..., 0], centres[..., 1]
    for start in (2., 2.99+gap):
        data[(x >= start) & (x <= start+.99) & (y >= 1.) & (y <= 1.55)] = 100
    if wall:
        data[(x >= -3.) & (x <= 15.) & (y > 1.55) & (y <= 1.70)] = 100
    return data, origin, np.array([0., 0., 0.])


def rig():
    vehicle = VehicleGeometry(GEOMETRY)
    geometry = vehicle.model()
    return geometry, Footprints(*vehicle.polygons(), geometry)


def detected_target(data, origin, road, footprints, slot_width=1.10):
    config = PairConfig(slot_width=slot_width, min_gap_length=max(2.6, float(footprints.size[0])+.36))
    observation = observe_two_structure_gap(planning_grid(data, RES, origin), road, config,
                                            require_camera=False, minimum_goal_width=footprints.size[1]+.36)
    assert observation.target is not None, observation.reason
    return observation.target


def plan_with_wall(gap, timeout=35., wall=False, slot_width=1.10):
    """Same goal selection/parking_plan call as AutoParking, on the walled grid."""
    mod = install_b()
    geometry, footprints = rig()
    data, origin, road = scene(gap, wall)
    target = detected_target(data, origin, road, footprints, slot_width)
    grid = planning_grid(block_behind_bay(data, RES, origin, target), RES, origin, simulation_clearance=True)
    heading = target.heading
    along = np.array([math.cos(heading), math.sin(heading)])
    normal = np.array([-along[1], along[0]])
    start = np.array([2.99+gap+.99+1., 0., 0., 0.])  # forward search already passed the bay
    errors = []
    for shift in (.4, 0., .2, .6):
        goal = footprints.goal(target.center-shift*along, heading)
        if not target.contains(footprints.world(goal)) or not grid.check(goal, footprints):
            continue
        road_y = float((road[:2]-goal[:2])@normal)
        planner = mod.HybridAStar(geometry, footprints, grid, timeout=timeout)
        try:
            return target, grid, footprints, planner.parking_plan(start, goal, road_y=road_y,
                                                                  staging_distances=(7.,))
        except RuntimeError as exc:
            errors.append(str(exc))
    return target, grid, footprints, None


def max_rig_depth(path, target, footprints):
    return max(float(depth_of(polygon, target).max())
               for state in path.states for polygon in footprints.world(state))


@pytest.mark.planning
def test_shallow_open_bay_has_a_reverse_entry_with_the_virtual_wall():
    target, grid, footprints, path = plan_with_wall(3.6, slot_width=.50)
    assert path is not None
    assert grid.path_free(path.states, footprints)
    assert target.contains(footprints.world(path.states[-1]))
    assert max_rig_depth(path, target, footprints) <= target.width/2+RES


@pytest.mark.planning
@pytest.mark.parametrize('gap', [3.6, 3.3, 3.0])
def test_planned_path_never_passes_behind_the_bay(gap):
    started = time.monotonic()
    target, grid, footprints, path = plan_with_wall(gap)
    if gap >= 3.6:
        assert path is not None, 'expected a reverse entry at 3.6 m'
    if path is not None:
        assert grid.path_free(path.states, footprints)
        assert max_rig_depth(path, target, footprints) <= target.width/2+RES
        print(f'gap={gap} gear_changes={path.gear_changes} '
              f'elapsed={time.monotonic()-started:.1f}s')
    else:
        print(f'gap={gap} no path elapsed={time.monotonic()-started:.1f}s')


@pytest.fixture
def node(tmp_path, monkeypatch):
    import rclpy
    monkeypatch.setattr(Geometry, 'from_world', classmethod(lambda cls, name: VehicleGeometry(name).model()))
    rclpy.init(args=['--ros-args', '-p', f'world_file:={GEOMETRY}',
                    '-p', f'blocked_debug_directory:={tmp_path}', '-p', 'slot_width:=1.10'])
    instance = TwoStructureParkingTest()
    try:
        yield instance
    finally:
        instance.planner_pool.shutdown(wait=True, cancel_futures=True)
        instance.destroy_node()
        rclpy.shutdown()


def occupancy(data, origin, sequence=1):
    from nav_msgs.msg import OccupancyGrid
    m = OccupancyGrid()
    m.header.frame_id = 'local_map'
    m.header.stamp.sec = 7
    m.info.width, m.info.height = data.shape[1], data.shape[0]
    m.info.resolution = RES
    m.info.origin.position.x, m.info.origin.position.y = float(origin[0]), float(origin[1])
    m.info.origin.orientation.z = math.sin(origin[2]/2)
    m.info.origin.orientation.w = math.cos(origin[2]/2)
    m.data = data.ravel().astype(int).tolist()
    return m, sequence


def test_node_grid_walls_only_after_target_and_keeps_search_grid(node):
    from rclpy.parameter import Parameter
    data, origin, road = scene(3.6)
    node.map_snapshot = occupancy(data, origin)
    before = node.grid(False)
    assert (before.data == data).all()
    target = detected_target(data, origin, road, node.footprints or rig()[1])
    node.target = target
    for fill in (False, True):
        walled = node.grid(fill)
        assert walled is not before and walled.map_sequence == 1 and walled.frame_id == 'local_map'
        assert walled.map_stamp_ns == 7*10**9
        deep = depth_of(cell_centres(data.shape, origin), target).reshape(data.shape) > target.width/2
        assert (walled.data[deep] == 100).all() and walled.blocked[deep].all()
        # Clearance was recomputed: cells just inside the inner edge are now close to the wall.
        inside = ~deep & (depth_of(cell_centres(data.shape, origin), target).reshape(data.shape) > .45)
        assert walled.clearance[inside & (data == 0)].max() < before.clearance[inside & (data == 0)].max()
        assert node.grid(fill) is walled  # cached
    node.set_parameters([Parameter('search_stop_on_unknown', value=True)])
    assert (node.search_grid().data == data).all()
    # A different bay rebuilds the wall.
    node.target = Target(center=target.center+[0., .3], heading=target.heading,
                         length=target.length, width=target.width)
    assert node.grid(False) is not walled
    node.set_parameters([Parameter('pair_block_behind_bay', value=False),
                         Parameter('pair_bay_unknown_as_free', value=False)])
    assert (node.grid(False).data == data).all()


def test_node_planning_grid_frees_unknown_cells_inside_the_bay(node):
    from rclpy.parameter import Parameter
    data, origin, road = scene(3.6)
    target = detected_target(data, origin, road, node.footprints or rig()[1])
    data[np.unravel_index(np.argmin(np.linalg.norm(cell_centres(data.shape, origin)-target.center, axis=1)),
                          data.shape)] = -1
    node.map_snapshot = occupancy(data, origin)
    assert (node.grid(False).data < 0).sum() == 1
    node.target = target
    assert (node.grid(False).data < 0).sum() == 0
    node.set_parameters([Parameter('pair_bay_unknown_as_free', value=False)])
    assert (node.grid(False).data < 0).sum() == 1


@pytest.mark.planning
def test_node_plans_from_short_staging_when_seven_metres_is_blocked(node):
    mod = install_b()
    geometry, footprints = rig()
    node.road_pose = road = np.zeros(3)
    node.footprints = footprints
    data, origin, _ = scene(3.6)
    observation = observe_two_structure_gap(planning_grid(data, RES, origin), road,
                                            require_camera=False)
    node.target = target = observation.target
    goal = footprints.goal(target.center-[.4, 0.], target.heading)
    centres = cell_centres(data.shape, origin).reshape(*data.shape, 2)
    data[centres[:, :, 0] >= goal[0]+6.4] = 100
    node.map_snapshot = occupancy(data, origin)
    node.observation_hits = 3
    node.current = np.array([goal[0]+4.6, 0., 0., 0.])
    stop_u, should_stop = node.legacy_search_stop(observation, node.current[0])
    assert should_stop and stop_u < goal[0]+7.1
    node.current[0] = stop_u+.02
    node.staging_reference = node.current.copy()
    node.goal = node.select_goal()
    assert np.allclose(node.goal, goal)
    normal = np.array([0., 1.])
    grid = node.grid(False)
    distances = node.staging_distances(grid, goal, normal)
    assert distances[0] == pytest.approx(node.current[0]-goal[0])
    assert 7. not in distances
    planner = mod.HybridAStar(geometry, footprints, grid, timeout=10.)
    path = planner.parking_plan(node.current, goal, road_y=-goal[1], staging_distances=distances)
    assert grid.path_free(path.states, footprints)
    assert target.contains(footprints.world(path.states[-1]))


def test_configured_gap_floor_allows_a_measured_gap_just_below_three_point_three(node):
    data, origin, road = scene(3.3)
    grid = planning_grid(data, RES, origin)
    config = node.pair_config()
    result = observe_two_structure_gap(grid, road, config, require_camera=False)
    assert result.target is not None and result.target.length < 3.3
    old = observe_two_structure_gap(grid, road, replace(config, min_gap_length=3.3),
                                    require_camera=False)
    assert old.target is None


def test_blocked_forward_sweep_can_stop_before_passing_second_structure(node):
    node.road_pose = road = np.zeros(3)
    _, node.footprints = rig()
    data, origin, _ = scene(3.6)
    observation = observe_two_structure_gap(planning_grid(data, RES, origin), road,
                                            require_camera=False)
    node.observation_hits = 3
    node.v = .14
    node.current = np.array([8., 0., 0., 0.])
    node.map_snapshot = occupancy(data, origin)
    stop_u, should_stop = node.legacy_search_stop(observation, node.current[0])
    assert node.current[0] < stop_u and not should_stop
    centres = cell_centres(data.shape, origin).reshape(*data.shape, 2)
    front = np.vstack(node.footprints.world(node.current))[:, 0].max()
    data[centres[:, :, 0] >= front+.25] = 100
    node.map_snapshot = occupancy(data, origin, sequence=2)
    assert node.legacy_search_stop(observation, node.current[0]) == (stop_u, True)


def test_lateral_goal_candidate_survives_precheck_and_reaches_scheduler(node):
    node.road_pose = np.zeros(3)
    node.current = np.array([10., 0., 0., 0.])
    _, node.footprints = rig()
    node.target = Target(center=np.array([4., 1.6]), heading=0., length=3.6, width=1.5)
    grid = Mock(map_stamp_ns=7, data=np.zeros((10, 10)), origin=np.zeros(3), resolution=RES)
    # The road is accessible and only the road-side parking target has clearance.
    grid.check.side_effect = lambda state, _: state[0] > 7. or state[1] < 1.55
    node.grid = Mock(return_value=grid)
    node.submit_plan_candidates = Mock()
    node.goal = node.select_goal()
    assert node.goal_lateral_shift == pytest.approx(-.1)
    node.launch_plan()
    _, goal, shift, lateral = node.plan_context['attempts'][0]
    assert np.allclose(goal, node.goal)
    assert (shift, lateral) == (.4, -.1)
    node.submit_plan_candidates.assert_called_once()


def test_unobserved_depth_keeps_unknown_blocked_and_virtual_wall_active(node):
    data, origin, road = scene(3.6)
    centres = cell_centres(data.shape, origin).reshape(*data.shape, 2)
    unknown = ((centres[:, :, 0] > 3.2) & (centres[:, :, 0] < 5.8)
               & (centres[:, :, 1] > 1.5) & (centres[:, :, 1] < 2.))
    data[unknown] = -1
    node.target = detected_target(data, origin, road, node.footprints or rig()[1])
    assert not node.target.depth_observed
    node.map_snapshot = occupancy(data, origin)
    for fill in (False, True):
        grid = node.grid(fill)
        assert (grid.data[unknown] == -1).all() and grid.blocked[unknown].all()
        behind = depth_of(centres, node.target) > node.target.width/2
        assert grid.blocked[behind].all()


def test_rear_block_parameters_validated_and_locked_during_motion(node):
    from rclpy.parameter import Parameter
    assert not node.set_parameters([Parameter('pair_rear_block_margin_m', value=-.1)])[0].successful
    assert node.set_parameters([Parameter('pair_rear_block_margin_m', value=.2)])[0].successful
    node.test_motion_enabled = True
    for parameter in (Parameter('pair_rear_block_margin_m', value=.0),
                      Parameter('pair_block_behind_bay', value=False)):
        assert not node.set_parameters([parameter])[0].successful


@pytest.mark.planning
def test_structures_against_building_wall_still_get_a_reverse_entry():
    started = time.monotonic()
    target, grid, footprints, path = plan_with_wall(3.6, wall=True)
    assert path is not None, 'expected a path with structures flush against a wall'
    assert grid.path_free(path.states, footprints)
    assert max_rig_depth(path, target, footprints) <= target.width/2+RES
    print(f'wall gap=3.6 gear_changes={path.gear_changes} elapsed={time.monotonic()-started:.1f}s')


def test_unknown_inside_bay_becomes_free_only_inside_the_rectangle():
    data = np.full((80, 80), -1, dtype=np.int16)
    data[40, 40] = 100
    origin = np.array([-2., -2., .3])
    target = Target(center=np.zeros(2), heading=.7, length=2., width=1.)
    out = open_bay_unknown(data, RES, origin, target)
    centres = cell_centres(data.shape, origin)
    along = centres@[math.cos(.7), math.sin(.7)]
    depth = centres@[-math.sin(.7), math.cos(.7)]
    inside = ((abs(along) <= 1.) & (abs(depth) <= .5)).reshape(data.shape)
    assert (out[inside & (data < 0)] == 0).all() and (out[~inside] == data[~inside]).all()
    assert out[40, 40] == 100 and (data == -1).sum() == data.size-1
