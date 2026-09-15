"""Reproducible kinematic benchmark, not a Gazebo/vision performance claim."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/trailer_parking_pkg'))
from trailer_parking_pkg.core import (Geometry, State, Scene, Slot, rectangle,
    plan, goal_reached, advance, Tracker)


def run(x, y, timeout, length=11.094831):
    g = Geometry()
    slot = Slot(0, 0, 0, length, 3.2)
    obstacles = tuple(rectangle(p, 0, 0, 2.27524825, 2.27524825, g.width)
                      for p in (-(length/2+2.27524825), length/2+2.27524825))
    scene = Scene(obstacles, (-27, 27, -16, 8))
    start = State(x, y, 0, 0)
    began = time.monotonic()
    path = plan(start, slot, scene, g, timeout=timeout)
    elapsed = time.monotonic()-began
    tracker, q = Tracker(path, g), start
    trajectory = []
    for _ in range(12000):
        if goal_reached(q, slot, g):
            break
        v, d = tracker.command(q, scene)
        q = advance(q, v*0.1, d, g)
        if not scene.free(q, g):
            raise RuntimeError('Closed-loop collision')
        trajectory.append([q.x, q.y, q.yaw, q.beta, v, d])
    result = dict(start=[x, y], slot_length=length, planning_seconds=elapsed, path_points=len(path),
                  reverse_points=sum(p.direction < 0 for p in path),
                  max_beta_deg=max(abs(p.state.beta)*180/math.pi for p in path),
                  planned_goal=goal_reached(path[-1].state, slot, g),
                  tracked_goal=goal_reached(q, slot, g), ticks=len(trajectory))
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=9.0)
    parser.add_argument('--y', type=float, default=-4.0)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--length', type=float, default=11.094831)
    args = parser.parse_args()
    run(args.x, args.y, args.timeout, args.length)
