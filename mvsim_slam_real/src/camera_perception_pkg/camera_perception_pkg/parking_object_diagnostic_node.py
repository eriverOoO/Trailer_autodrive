"""Publish an image-only or calibrated camera/scan diagnostic overlay."""

from collections import deque
import json
import math
from pathlib import Path
import signal
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .parking_objects import ScanProjection, associate_candidates, detect_candidates, draw_diagnostic


def stamp_ns(message):
    """Return the sensor header timestamp in nanoseconds."""
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ParkingObjectDiagnostic(Node):
    """Inspect detections and scan overlap without publishing drive commands."""

    def __init__(self, parameter_overrides=None):
        super().__init__('parking_object_diagnostic', parameter_overrides=parameter_overrides)
        self.detector_backend = self.declare_parameter('detector_backend', 'heuristic').value
        model_path = self.declare_parameter('detector_model', '').value
        confidence = self.declare_parameter('detector_confidence', .65).value
        self.max_image_age_sec = self.declare_parameter('max_image_age_sec', .5).value
        if not math.isfinite(self.max_image_age_sec) or self.max_image_age_sec <= 0:
            raise ValueError('max_image_age_sec must be finite and positive')
        self.scan_wait_sec = self.declare_parameter('scan_wait_sec', .15).value
        if not math.isfinite(self.scan_wait_sec) or not 0 <= self.scan_wait_sec < self.max_image_age_sec:
            raise ValueError('scan_wait_sec must be nonnegative and below max_image_age_sec')
        if not math.isfinite(confidence) or not 0 < confidence < 1:
            raise ValueError('detector_confidence must be between zero and one')
        self.detector = None
        if self.detector_backend == 'ride_on_onnx':
            from .ride_on_detector import RideOnOnnxDetector
            self.detector = RideOnOnnxDetector(model_path, confidence)
        elif self.detector_backend != 'heuristic':
            raise ValueError('detector_backend must be heuristic or ride_on_onnx')
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
        self.pending_images = deque(maxlen=12)
        self.overlay = self.create_publisher(Image, 'objects_overlay', 1)
        self.associations = self.create_publisher(String, 'associations', 5)
        self.selected_scan = None
        self.scan_callback_group = MutuallyExclusiveCallbackGroup()
        if self.projection is not None:
            self.create_subscription(LaserScan, 'scan', self.recent_scans.append,
                                     qos_profile_sensor_data, callback_group=self.scan_callback_group)
        # Large raw images can lose fragments with best-effort delivery. Match
        # the built-in reliable camera publisher when launched together.
        reliable_image = self.declare_parameter('reliable_image', False).value
        image_qos = QoSProfile(depth=1) if reliable_image else qos_profile_sensor_data
        self.create_subscription(Image, 'image_raw', self.on_image, image_qos)
        self.create_timer(.02, self.process_pending_image)
        self.get_logger().info(f'Vision detector: {self.detector_backend}; confidence={confidence:.2f}; '
                               f'calibration={"loaded" if self.projection else "missing"}')

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
        scans = [scan for scan in list(self.recent_scans) if stamp_ns(scan) != 0]
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
        # A whole scan arrives after its first beam's header time. Briefly retain
        # camera frames so matching does not repeatedly compare them to old scans.
        if self.projection is None:
            self.process_image(message)
            return
        self.pending_images.append((time.monotonic(), message))
        self.process_pending_image()

    def process_pending_image(self):
        if not self.pending_images:
            return
        now = time.monotonic()
        scans = list(self.recent_scans)
        newest_scan = max((stamp_ns(scan) for scan in scans
                           if scan.header.frame_id == self.projection.scan_frame), default=0)
        ready = [i for i, (received, message) in enumerate(self.pending_images)
                 if now-received >= self.scan_wait_sec or newest_scan >= stamp_ns(message)]
        if not ready:
            return
        message = None
        for _ in range(ready[-1]+1):
            _, message = self.pending_images.popleft()
        self.process_image(message)

    def process_image(self, message):
        """Run detection on every frame, even without a usable scan."""
        try:
            # The existing OpenCV publisher uses generic 8UC3 for its BGR frames.
            encoding = 'passthrough' if message.encoding == '8UC3' else 'bgr8'
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding=encoding).copy()
        except CvBridgeError as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}', throttle_duration_sec=5)
            return
        candidates, pixels, scan, matched = [], None, None, []
        try:
            if not self.image_fresh(message):
                raise ValueError('Image timestamp stale/future/missing')
            candidates = (self.detector.detect(image) if self.detector is not None else
                          detect_candidates(image, **self.detector_options))
            if not self.image_fresh(message):
                raise ValueError('Image became stale during inference')
            pixels, status = self.scan_pixels(message, image)
            scan = self.selected_scan
            if scan is not None:
                paired_pixels, scan_xy, indices = self.projection.projected_returns(scan, return_indices=True)
                matched = associate_candidates(candidates, paired_pixels, scan_xy, beam_indices=indices)
        except (ValueError, RuntimeError) as exc:
            candidates, pixels, scan, matched = [], None, None, []
            status = str(exc)
            self.get_logger().warning(status, throttle_duration_sec=5)
        association = String()
        association.data = json.dumps({
            'calibrated': self.projection is not None, 'valid': scan is not None,
            'detector_backend': self.detector_backend,
            'image_stamp_ns': stamp_ns(message),
            'scan_stamp_ns': stamp_ns(scan) if scan is not None else None,
            'scan_frame': scan.header.frame_id if scan is not None else None,
            'image_candidates': [{'label': item.label, 'confidence': item.confidence,
                                  'box_xywh': list(cv2.boundingRect(item.contour))}
                                 for item in candidates],
            'objects': matched, 'reason': status,
        }, allow_nan=False)
        self.associations.publish(association)
        overlay = draw_diagnostic(image, candidates, pixels, status)
        output = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        output.header = message.header
        self.overlay.publish(output)

    def image_fresh(self, message):
        stamp = stamp_ns(message)
        return stamp > 0 and -.03 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= self.max_image_age_sec


def main(args=None):
    """Run the diagnostic until shutdown."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = ParkingObjectDiagnostic()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch may forward SIGINT after the terminal already signalled us.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
