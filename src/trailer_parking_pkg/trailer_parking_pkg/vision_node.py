"""YOLO segmentation + metric ground VO for one on-board camera."""
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from .calibration import simulation_calibration
from .core import Geometry
from .vision import GroundCamera, GroundOdometry, slot_from_mask, obstacle_from_mask, merge_slots


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


class ParkingVision(Node):
    def __init__(self):
        super().__init__('parking_vision')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.camera_name = param('camera', 'front')
        weights = Path(param('weights', ''))
        if not weights.is_file():
            raise ValueError('weights must be an existing parking YOLO segmentation .pt file')
        self.device = param('device', 'cpu')
        self.threshold = param('confidence', 0.65)
        calibration_path = param('calibration', '')
        if calibration_path:
            self.calibration = json.loads(Path(calibration_path).read_text())
        else:
            self.calibration = simulation_calibration(param('model_sdf', ''), param('loader_source', ''), param('start_index', 1))
        geometry_override = param('geometry_override', '')
        if geometry_override:
            # The real vehicle geometry comes from the hardware launch arguments.
            self.calibration['geometry'] = vars(Geometry(**json.loads(geometry_override)))
        if self.camera_name not in self.calibration['cameras']:
            raise ValueError('Missing camera extrinsic calibration')
        self.model = YOLO(str(weights))
        required = {'parking_space', 'car_front', 'car_back'}
        if self.model.task != 'segment' or not required.issubset(set(self.model.names.values())):
            raise ValueError('Expected a segment model with parking_space/car_front/car_back classes')
        self.bridge, self.info, self.image, self.vo = CvBridge(), None, None, None
        self.last_processed = None
        self.last_report = ''
        self.create_subscription(Image, 'image', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera_info', self.info_cb, qos_profile_sensor_data)
        self.publisher = self.create_publisher(String, 'observation', 5)
        self.create_timer(0.10, self.process)

    def image_cb(self, msg):
        self.image = msg

    def info_cb(self, msg):
        self.info = msg

    def emit(self, data):
        msg = String()
        msg.data = json.dumps(data, allow_nan=False)
        self.publisher.publish(msg)
        status = data.get('reason', '')
        if status != self.last_report:
            self.get_logger().info(f'{self.camera_name}: {status}')
            self.last_report = status

    def process(self):
        if self.image is None or self.info is None:
            return
        msg, info = self.image, self.info
        stamp = seconds(msg.header.stamp)
        if stamp == self.last_processed:
            return
        self.last_processed = stamp
        data = {'stamp': stamp, 'camera': self.camera_name, 'valid': False,
                'reason': 'PROCESSING', 'slots': [], 'obstacles': []}
        try:
            if msg.width != info.width or msg.height != info.height:
                raise ValueError('Image/CameraInfo dimensions disagree')
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.vo is None:
                config = self.calibration['cameras'][self.camera_name]
                self.vo = GroundOdometry(GroundCamera(info.k, info.d, config['rotation'], config['translation']), config['initial_pose'])
            prediction = self.model.predict(image, conf=self.threshold, device=self.device, verbose=False)[0]
            masks = [] if prediction.masks is None else prediction.masks.xy
            named = [(self.model.names[int(box.cls.item())], np.asarray(poly))
                     for box, poly in zip(prediction.boxes, masks)]
            pose, reason = self.vo.update(image, stamp, [p for name, p in named if name != 'parking_space'])
            data['reason'] = reason
            if pose is None:
                self.emit(data)
                return
            slots, obstacles = [], []
            for name, poly in named:
                if name == 'parking_space':
                    # An image-border-clipped mask cannot establish full slot size.
                    if (poly[:, 0].min() <= 2 or poly[:, 1].min() <= 2 or
                            poly[:, 0].max() >= msg.width-3 or poly[:, 1].max() >= msg.height-3):
                        continue
                    slot = slot_from_mask(poly, self.vo.camera, pose)
                    if slot:
                        slots.append(slot)
                elif name in ('car_front', 'car_back'):
                    obstacle = obstacle_from_mask(poly, self.vo.camera, pose)
                    if obstacle:
                        obstacles.append(obstacle)
            data.update(valid=True, pose=list(pose), geometry=self.calibration['geometry'],
                        slots=[vars(s) for s in merge_slots(slots)], obstacles=obstacles)
            self.emit(data)
        except Exception as exc:
            data['reason'] = f'VISION_ERROR: {exc}'
            self.emit(data)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ParkingVision()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
