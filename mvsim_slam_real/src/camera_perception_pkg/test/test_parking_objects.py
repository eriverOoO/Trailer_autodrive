"""Test image heuristics and scan projection without a ROS installation."""

from types import SimpleNamespace
import unittest

import cv2
import numpy as np

from camera_perception_pkg.parking_objects import (
    ScanProjection, detect_candidates, draw_diagnostic,
)


class ParkingObjectsTest(unittest.TestCase):
    """Exercise detection, image coordinates and calibrated 2D returns."""

    def test_color_and_shape_candidates(self):
        """Retain a tapered cone and bright body while rejecting a flat orange region."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(image, (10, 12), (120, 70), (240, 240, 240), -1)
        cv2.fillPoly(image, [np.array([(155, 170), (112, 310), (195, 310)])], (0, 140, 255))
        cv2.fillPoly(image, [np.array([(136, 232), (173, 232), (175, 239), (134, 239)])],
                     (240, 240, 240))
        cv2.rectangle(image, (220, 180), (310, 255), (0, 140, 255), -1)
        cv2.rectangle(image, (355, 250), (540, 330), (240, 240, 240), -1)
        candidates = detect_candidates(image)
        boxes = {item.label: cv2.boundingRect(item.contour) for item in candidates}
        self.assertEqual(len(candidates), 2)
        self.assertLess(abs(boxes['cone'][1] - 170), 10)
        self.assertEqual(boxes['white body'], (355, 250, 186, 81))
        overlay = draw_diagnostic(image.copy(), candidates, np.array([[155, 270]]), 'test')
        self.assertFalse(np.array_equal(overlay, image))

    def test_scan_frame_to_camera_projection(self):
        """Reject invalid and behind-camera returns and account for lateral offset."""
        projection = ScanProjection({
            'image_size': [640, 480],
            'camera_frame': 'camera_optical',
            'scan_frame': 'laser',
            'camera_matrix': [[400, 0, 320], [0, 400, 240], [0, 0, 1]],
            'distortion_coefficients': [0, 0, 0, 0, 0],
            'lidar_to_camera_rotation': [[0, -1, 0], [0, 0, -1], [1, 0, 0]],
            'lidar_to_camera_translation_m': [0.5, 0, 0],
        })
        scan = SimpleNamespace(ranges=[2., float('inf'), 0.02, 2., 2.],
                               angle_min=0., angle_increment=np.pi / 4,
                               range_min=0.1, range_max=15.)
        np.testing.assert_array_equal(projection.pixels(scan), [[420, 240]])
        scan.ranges = [float('inf')]
        self.assertEqual(projection.pixels(scan).shape, (0, 2))


if __name__ == '__main__':
    unittest.main()
