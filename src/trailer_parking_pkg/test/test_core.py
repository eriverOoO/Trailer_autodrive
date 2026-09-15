"""Run with python -m unittest discover -s src/trailer_parking_pkg/test."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trailer_parking_pkg.core import (Geometry, State, Scene, Slot, advance,
    trailer_pose, rectangle, overlaps, goal_reached, plan, PlanningError,
    gazebo_command, Tracker)


class CoreTest(unittest.TestCase):
    def test_straight_and_reversible(self):
        g = Geometry()
        s = State(0, 0, 0, 0)
        self.assertAlmostEqual(advance(s, -2, 0, g).x, -2)
        self.assertAlmostEqual(trailer_pose(s, g)[0], -4.8)
        q = advance(advance(s, 0.05, 0.3, g), -0.05, 0.3, g)
        for a, b in zip(vars(s).values(), vars(q).values()):
            self.assertAlmostEqual(a, b, places=9)

    def test_hitch_velocity_constraint(self):
        g = Geometry()
        s = State(2, 1, 0.4, 0.2)
        p = trailer_pose(s, g)
        q = trailer_pose(advance(s, 1e-5, 0.3, g), g)
        lateral = -(q[0]-p[0])*math.sin(p[2]) + (q[1]-p[1])*math.cos(p[2])
        self.assertLess(abs(lateral), 1e-10)

    def test_reverse_articulation_and_adapter(self):
        g = Geometry()
        s = State(0, 0, 0, 0.1)
        self.assertGreater(advance(s, -0.2, 0, g).beta, s.beta)
        self.assertEqual(gazebo_command(-0.3, 0.2), (-0.3, -0.2))

    def test_trailer_collision_not_only_tractor(self):
        obstacle = rectangle(-5, 0, 0, 0.1, 0.1, 0.2)
        self.assertFalse(Scene((obstacle,)).free(State(0, 0, 0, 0), Geometry()))
        self.assertFalse(Scene().free(State(0, 0, 0, 0.8), Geometry()))
        self.assertTrue(overlaps(obstacle, obstacle))

    def test_complete_rig_goal(self):
        g = Geometry()
        slot = Slot(0, 0, 0, 11.1, 3.2)
        self.assertTrue(slot.contains(slot.goal(g), g))
        self.assertTrue(goal_reached(slot.goal(g), slot, g))
        self.assertFalse(slot.contains(State(3, 0, 0, 0), g))
        with self.assertRaises(PlanningError):
            plan(State(0, 0, 0, 0), Slot(0, 0, 0, 5, 3), Scene())

    def test_plan_and_closed_loop_straight(self):
        g = Geometry()
        slot = Slot(0, 0, 0, 11.1, 3.2)
        target = slot.goal(g)
        start = State(target.x-2.8, target.y, 0, 0)
        path = plan(start, slot, Scene(), timeout=10)
        self.assertTrue(goal_reached(path[-1].state, slot, g))
        tracker = Tracker(path, g)
        q = start
        for _ in range(300):
            if goal_reached(q, slot, g):
                break
            v, d = tracker.command(q, Scene())
            q = advance(q, v*0.1, d, g)
        self.assertTrue(goal_reached(q, slot, g))


if __name__ == '__main__':
    unittest.main()
