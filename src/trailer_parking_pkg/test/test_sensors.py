import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trailer_parking_pkg.sensors import fresh, scan_obstacles
from trailer_parking_pkg.core import Geometry, State


class SensorTest(unittest.TestCase):
    def test_clock_validity(self):
        self.assertTrue(fresh(2.0, 2.1, 0.8))
        for stamp in (0.0, 3.0, math.nan, math.inf):
            self.assertFalse(fresh(stamp, 2.1, 0.8))

    def test_actual_scan_angles_and_no_return(self):
        g, state = Geometry(), State(0, 0, 0, 0)
        ranges = [math.inf]*16
        ranges[4] = 5.0
        result = scan_obstacles(ranges, -math.pi, math.pi/8, 0.1, 20,
                                (0, 0, 0), state, g, (0, 0, 0))
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(sum(p[0] for p in result[0])/4, 0, places=6)
        self.assertAlmostEqual(sum(p[1] for p in result[0])/4, -5, places=6)
        self.assertEqual(scan_obstacles([math.inf]*16, -math.pi, math.pi/8,
            0.1, 20, (0, 0, 0), state, g, (0, 0, 0)), ())

    def test_bad_scan_is_not_free_space(self):
        for ranges in ([math.nan]*16, [0.0]*16, [-math.inf]*16):
            with self.assertRaises(ValueError):
                scan_obstacles(ranges, 0, 0.1, 0.1, 20, (0, 0, 0), State(0, 0, 0, 0), Geometry(), (0, 0, 0))


if __name__ == '__main__':
    unittest.main()
