"""Test image heuristics and scan projection without a ROS installation."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from camera_perception_pkg.parking_objects import (
    ScanProjection, detect_candidates, draw_diagnostic,
)

try:
    from cv_bridge import CvBridge
    import rclpy
except ImportError:
    rclpy = None
else:
    from camera_perception_pkg.parking_object_diagnostic_node import ParkingObjectDiagnostic


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


@unittest.skipIf(rclpy is None, 'Requires ROS 2 and cv_bridge')
class CameraMessageTest(unittest.TestCase):
    """Pass real legacy and standard image messages through the diagnostic callback."""

    def test_existing_publisher_image_encodings(self):
        """Preserve BGR pixels and headers for 8UC3, bgr8 and rgb8 messages."""
        rclpy.init(args=[])
        node = None
        try:
            node = ParkingObjectDiagnostic()
            node.overlay = Mock()
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.rectangle(image, (250, 200), (370, 300), (240, 240, 240), -1)
            image[450, 600] = (10, 20, 30)
            bridge = CvBridge()
            for encoding in ('8UC3', 'bgr8', 'rgb8'):
                with self.subTest(encoding=encoding):
                    source = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if encoding == 'rgb8' else image
                    message = bridge.cv2_to_imgmsg(source, encoding=encoding)
                    message.header.frame_id = 'existing_camera_frame'
                    message.header.stamp.sec = 123
                    node.overlay.reset_mock()
                    node.on_image(message)
                    node.overlay.publish.assert_called_once()
                    output = node.overlay.publish.call_args.args[0]
                    self.assertEqual(output.encoding, 'bgr8')
                    self.assertEqual(output.header, message.header)
                    actual = bridge.imgmsg_to_cv2(output, desired_encoding='bgr8')
                    np.testing.assert_array_equal(actual[450, 600], image[450, 600])
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
