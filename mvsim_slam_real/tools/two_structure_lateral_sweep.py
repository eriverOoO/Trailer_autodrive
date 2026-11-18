"""Offline check: structure lateral distance / rear wall vs detection and planning.

Run on a built workspace (needs optimizer/libnative.so):
  PYTHONNOUSERSITE=1 python3 tools/two_structure_lateral_sweep.py
Start pose = historical 7.1 m staging case. Synthetic map, all free except the modelled structures.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W/'src/real_parking'))
sys.path.insert(0, str(W/'src/real_vehicle_bridge'))
from real_parking.bootstrap import install_b  # noqa: E402
from real_parking.two_structure_test import block_behind_bay, open_bay_unknown  # noqa: E402
from real_parking.two_structure_gap import PairConfig, observe_two_structure_gap  # noqa: E402
from real_vehicle_bridge.geometry import VehicleGeometry  # noqa: E402
from tractor_parking_perception import planning_grid  # noqa: E402
from tractor_trailer_model import Footprints, transform  # noqa: E402

RES = .05
mod = install_b()
vehicle = VehicleGeometry(W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml')
geometry = vehicle.model()
fp = Footprints(*vehicle.polygons(), geometry)
print('rig size (length, width):', np.round(fp.size, 3))


def run(lateral, gap, wall, depth=.55, length=.99):
    origin = np.array([-4., -4., 0.])
    data = np.zeros((200, 440), dtype=np.int16)
    rows, cols = np.indices(data.shape)
    x, y = origin[0]+(cols+.5)*RES, origin[1]+(rows+.5)*RES
    for start in (2., 2.+length+gap):
        data[(x >= start) & (x <= start+length) & (y >= lateral) & (y <= lateral+depth)] = 100
    if wall:
        data[(x >= -3.) & (x <= 17.) & (y > lateral+depth) & (y <= lateral+depth+.15)] = 100
    road = np.zeros(3)
    config = PairConfig(min_gap_length=max(3.3, float(fp.size[0])+.36))
    obs = observe_two_structure_gap(planning_grid(data, RES, origin), road, config, require_camera=False)
    if obs.target is None:
        return f'detect FAIL: {obs.reason}'
    target = obs.target
    grid = planning_grid(block_behind_bay(open_bay_unknown(data, RES, origin, target), RES, origin, target),
                         RES, origin, simulation_clearance=True)
    along = np.array([math.cos(target.heading), math.sin(target.heading)])
    normal = np.array([-along[1], along[0]])
    t0 = time.monotonic()
    for shift in (.4, 0., .2, .6):
        goal = fp.goal(target.center-shift*along, target.heading)
        if not target.contains(fp.world(goal)) or not grid.check(goal, fp):
            continue
        start = np.array([goal[0]+7.1, 0., 0., 0.])  # Preserve the original sweep baseline.
        try:
            path = mod.HybridAStar(geometry, fp, grid, timeout=35.).parking_plan(
                start, goal, road_y=float((road[:2]-goal[:2])@normal), staging_distances=(7.,))
        except RuntimeError:
            continue
        ys = np.concatenate([np.vstack(fp.world(s))[:, 1] for s in path.states])
        return (f'path OK gear_changes={path.gear_changes} right_extent_y={ys.min():.2f} '
                f'(start right side at -{fp.size[1]/2:.3f}) rear_wall={obs.rear_wall} '
                f'bay_depth_used={obs.rear_depth_m} t={time.monotonic()-t0:.1f}s')
    return f'no path (t={time.monotonic()-t0:.1f}s) rear_wall={obs.rear_wall}'


for wall in (False, True):
    for lateral in (.6, .8, 1.0, 1.5, 2.0):
        print(f'wall={wall} structure_face_y={lateral:.2f} gap=3.6:', run(lateral, 3.6, wall), flush=True)
print('small 0.3x0.3 structures against wall, face 1.0, gap 3.6:', run(1.0, 3.6, True, depth=.3, length=.3))
