"""Publish an image-only or calibrated camera/scan diagnostic overlay."""

from collections import deque
import json
import math
from pathlib import Path

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .parking_objects import ScanProjection, associate_candidates, detect_candidates, draw_diagnostic


def stamp_ns(message):
    """Return the sensor header timestamp in nanoseconds."""
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ParkingObjectDiagnostic(Node):
    """Inspect detections and scan overlap without publishing drive commands."""

    def __init__(self):
        super().__init__('parking_object_diagnostic')
        calibration_file = self.declare_parameter('calibration_file', '').value
        self.projection = None
        if calibration_file:
            with Path(calibration_file).expanduser().open(encoding='utf-8') as file:
                self.projection = ScanProjection(json.load(file))
        self.detector_options = {
            name: self.declare_parameter(name, default).value for name, default in (
                ('roi_top', 0.25), ('cone_hue_min', 3), ('cone_hue_max', 28),
                ('white_saturation_max', 50), ('white_value_min', 195),
            )
        }
        options = self.detector_options
        if not (0 <= options['roi_top'] < 1
                and 0 <= options['cone_hue_min'] <= options['cone_hue_max'] <= 179
                and 0 <= options['white_saturation_max'] <= 255
                and 0 <= options['white_value_min'] <= 255):
            raise ValueError('Invalid ROI or HSV bounds')
        max_delta = self.declare_parameter('max_stamp_delta_sec', 0.10).value
        if not math.isfinite(max_delta) or max_delta <= 0:
            raise ValueError('max_stamp_delta_sec must be finite and positive')
        self.max_delta_ns = int(max_delta * 1_000_000_000)
        self.bridge = CvBridge()
        self.recent_scans = deque(maxlen=8)
        self.overlay = self.create_publisher(Image, 'objects_overlay', 1)
        self.associations = self.create_publisher(String, 'associations', 5)
        self.selected_scan = None
        if self.projection is not None:
            self.create_subscription(LaserScan, 'scan', self.recent_scans.append,
                                     qos_profile_sensor_data)
        self.create_subscription(Image, 'image_raw', self.on_image, qos_profile_sensor_data)

    def scan_pixels(self, image_message, image):
        """Use a nearby stamped scan only when its frames and image size match."""
        calibration = self.projection
        self.selected_scan = None
        if calibration is None:
            return None, 'Camera only: no calibration file'
        if (image.shape[1::-1] != calibration.image_size
                or image_message.header.frame_id != calibration.camera_frame):
            return None, 'Calibration does not match camera frame or resolution'
        image_stamp = stamp_ns(image_message)
        scans = [scan for scan in self.recent_scans if stamp_ns(scan) != 0]
        if image_stamp == 0 or not scans:
            return None, 'Waiting for stamped image and scan'
        scan = min(scans, key=lambda item: abs(stamp_ns(item) - image_stamp))
        delta = abs(stamp_ns(scan) - image_stamp)
        if delta > self.max_delta_ns:
            return None, 'No scan close to image timestamp'
        if scan.header.frame_id != calibration.scan_frame:
            return None, 'Calibration does not match scan frame'
        self.selected_scan = scan
        return calibration.pixels(scan), f'Projected scan | stamp difference {delta / 1e6:.0f} ms'

    def on_image(self, message):
        """Run detection on every frame, even without a usable scan."""
        try:
            # The existing OpenCV publisher uses generic 8UC3 for its BGR frames.
            encoding = 'passthrough' if message.encoding == '8UC3' else 'bgr8'
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding=encoding).copy()
            candidates = detect_candidates(image, **self.detector_options)
            pixels, status = self.scan_pixels(message, image)
            scan = self.selected_scan
            matched = []
            if scan is not None:
                paired_pixels, scan_xy = self.projection.projected_returns(scan)
                matched = associate_candidates(candidates, paired_pixels, scan_xy)
            association = String()
            association.data = json.dumps({
                'calibrated': self.projection is not None, 'valid': scan is not None,
                'image_stamp_ns': stamp_ns(message),
                'scan_stamp_ns': stamp_ns(scan) if scan is not None else None,
                'scan_frame': scan.header.frame_id if scan is not None else None,
                'objects': matched, 'reason': status,
            }, allow_nan=False)
            self.associations.publish(association)
            overlay = draw_diagnostic(image, candidates, pixels, status)
            output = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}', throttle_duration_sec=5)
            return
        output.header = message.header
        self.overlay.publish(output)


def main(args=None):
    """Run the diagnostic until shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = ParkingObjectDiagnostic()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
