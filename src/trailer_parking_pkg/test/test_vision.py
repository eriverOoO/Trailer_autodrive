import math
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trailer_parking_pkg.vision import GroundCamera, GroundOdometry, to_world, merge_slots
from trailer_parking_pkg.calibration import rotation_rpy, simulation_calibration
from trailer_parking_pkg.core import Slot


class VisionTest(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[320, 0, 320], [0, 320, 240], [0, 0, 1]], dtype=float)
        optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        self.r = rotation_rpy(0, 0.5, 0) @ optical
        self.t = np.array([0.8, 0, 2.0])
        self.camera = GroundCamera(self.k, np.zeros(5), self.r, self.t)

    def pixels(self, world, pose=(0, 0, 0)):
        x, y, yaw = pose
        ground = np.asarray(world) - [x, y, 0]
        ground = ground @ rotation_rpy(0, 0, yaw)
        optical = (ground-self.t) @ self.r
        projected = optical @ self.k.T
        return projected[:, :2]/projected[:, 2:3], optical[:, 2] > 0

    def test_metric_roundtrip(self):
        ground = np.array([[5, -1, 0], [5, 1, 0], [9, 1, 0], [9, -1, 0]])
        pixels, _ = self.pixels(ground)
        actual, valid = self.camera.project(pixels)
        self.assertTrue(valid.all())
        np.testing.assert_allclose(actual, ground[:, :2], atol=1e-6)

    def test_horizon_and_calibration_reject(self):
        _, valid = self.camera.project([[320, -10000]])
        self.assertFalse(valid[0])
        with self.assertRaises(ValueError):
            GroundCamera(self.k, [], np.zeros((3, 3)), self.t)

    def test_ground_visual_motion(self):
        rng = np.random.default_rng(24)
        ground = np.c_[rng.uniform(3, 13, 2500), rng.uniform(-8, 8, 2500), np.zeros(2500)]
        def render(pose):
            image = np.zeros((480, 640, 3), np.uint8)
            pixels, valid = self.pixels(ground, pose)
            for (u, v), good in zip(pixels, valid):
                if good and 3 <= u < 637 and 3 <= v < 477:
                    cv2.circle(image, (round(u), round(v)), 2, (255, 255, 255), -1)
            return image
        vo = GroundOdometry(self.camera, (0, 0, 0))
        self.assertIsNone(vo.update(render((0, 0, 0)), 1.0)[0])
        pose, reason = vo.update(render((0.04, 0, 0.002)), 1.2)
        self.assertEqual(reason, 'OK')
        self.assertAlmostEqual(pose[0], 0.04, delta=0.025)
        self.assertAlmostEqual(pose[1], 0, delta=0.025)
        self.assertAlmostEqual(pose[2], 0.002, delta=0.01)
        self.assertIsNone(vo.update(render((0.04, 0, 0.002)), 3.0)[0])

    def test_slot_merge_requires_adjacent_same_row(self):
        a, b = Slot(0, 0, 0, 5.2, 3), Slot(5.3, 0, 0, 5.2, 3)
        self.assertEqual(len(merge_slots([a, b])), 3)
        self.assertEqual(len(merge_slots([a, Slot(5.3, 2, 0, 5.2, 3)])), 2)

    def test_calibration_matches_sdf(self):
        repo = Path(__file__).resolve().parents[3]
        cfg = simulation_calibration(repo/'src/simulation_pkg/models/prius_hybrid/model.sdf',
            repo/'src/simulation_pkg/simulation_pkg/lib/load_towing_system_node.py', 1)
        self.assertAlmostEqual(cfg['rig_length'], 9.3504965)
        self.assertAlmostEqual(cfg['geometry']['width'], 2.197298)
        self.assertAlmostEqual(cfg['initial_state']['x'], 0.9841301698728664)
        for camera in cfg['cameras'].values():
            r = np.array(camera['rotation'])
            np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-8)


if __name__ == '__main__':
    unittest.main()
