"""Trailer parking from front/rear YOLO bbox size and position.

detections_front / detections_rear (yolov8_node) -> taught stages -> topic_control_signal.
Each stage drives with a fixed Arduino command (steering -7 left..+7 right, signed PWM)
until every target bbox matches its taught normalized [cx, cy, w, h], then stops,
confirms the match on several fresh frames and moves on to the next stage.
Metric pose is never inferred from boxes.
"""
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from interfaces_pkg.msg import DetectionArray, MotionCommand

CAMERAS = ('front', 'rear')
TIMER = 0.1


def validate_profile(profile):
    stages = profile.get('stages', [])
    if not profile.get('tuned') or not stages:
        raise ValueError('Profile must set "tuned": true and contain measured stages')
    for stage in stages:
        if (type(stage['steering']) is not int or abs(stage['steering']) > 7 or
                type(stage['pwm']) is not int or abs(stage['pwm']) > 255 or
                not math.isfinite(stage['timeout']) or not 0 < stage['timeout'] <= 30):
            raise ValueError(f"{stage.get('name')}: steering -7..7, pwm -255..255, timeout 0..30 s")
        targets = stage['targets']
        if not targets or not set(targets) <= set(CAMERAS):
            raise ValueError(f"{stage['name']}: targets must use front and/or rear")
        for target in targets.values():
            box, tolerance = target['box'], target['tolerance']
            if (len(box) != 4 or len(tolerance) != 4 or
                    not all(math.isfinite(x) and 0 <= x <= 1 for x in box) or
                    not all(math.isfinite(x) and 0 < x <= 1 for x in tolerance) or
                    min(box[2:]) <= 0):
                raise ValueError(f"{stage['name']}: box/tolerance are normalized cx, cy, w, h")
    return stages


def iou(a, b):
    ax1, ay1, ax2, ay2 = a[0]-a[2]/2, a[1]-a[3]/2, a[0]+a[2]/2, a[1]+a[3]/2
    bx1, by1, bx2, by2 = b[0]-b[2]/2, b[1]-b[3]/2, b[0]+b[2]/2, b[1]+b[3]/2
    inter = max(0.0, min(ax2, bx2)-max(ax1, bx1)) * max(0.0, min(ay2, by2)-max(ay1, by1))
    union = a[2]*a[3] + b[2]*b[3] - inter
    return inter/union if union > 0 else 0.0


def normalize(msg, width, height, confidence, merge_iou):
    """DetectionArray -> border-free normalized boxes, overlapping same-class duplicates merged."""
    boxes = []
    for det in sorted(msg.detections, key=lambda d: -d.score):
        cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
        w, h = det.bbox.size.x, det.bbox.size.y
        # A box clipped by the image border does not show its true size.
        if det.score < confidence or cx-w/2 <= 2 or cy-h/2 <= 2 or cx+w/2 >= width-2 or cy+h/2 >= height-2:
            continue
        box = [cx/width, cy/height, w/width, h/height]
        if any(b['class'] == det.class_name and iou(b['box'], box) > merge_iou for b in boxes):
            continue
        boxes.append({'class': det.class_name, 'score': round(det.score, 3),
                      'box': [round(v, 4) for v in box]})
    return boxes


class StageSequence:
    def __init__(self, profile, confirm_frames=5, stale_timeout=1.0, max_skew=0.5, pause=0.5):
        self.stages = validate_profile(profile)
        self.confirm_frames, self.stale_timeout = confirm_frames, stale_timeout
        self.max_skew, self.pause = max_skew, pause
        self.index, self.started, self.hits = 0, None, 0
        self.last_stamps, self.fault, self.ready_at = None, None, 0.0

    def step(self, observations, now):
        """Return (steering, pwm, status)."""
        if self.fault:
            return 0, 0, self.fault
        if self.index == len(self.stages):
            return 0, 0, 'PARKING_COMPLETE'
        stage = self.stages[self.index]
        if self.started is not None and now-self.started > stage['timeout']:
            self.fault = f"STOP_STAGE_TIMEOUT: {stage['name']}"
            return 0, 0, self.fault
        stamps, matched = [], True
        for camera, target in stage['targets'].items():
            obs = observations.get(camera)
            if not obs or not 0 <= now-obs['stamp'] <= self.stale_timeout:
                self.hits = 0
                return 0, 0, f'WAIT_FRESH_{camera.upper()}'
            stamps.append(obs['stamp'])
            boxes = [b['box'] for b in obs['boxes'] if b['class'] == target['class']]
            # Never silently switch to another object of the same class.
            if len(boxes) != 1:
                self.hits = 0
                return 0, 0, f"STOP_{'MISSING' if not boxes else 'AMBIGUOUS'}_{camera.upper()}_{target['class']}"
            matched &= all(abs(a-b) <= t for a, b, t in zip(boxes[0], target['box'], target['tolerance']))
        if max(stamps)-min(stamps) > self.max_skew:
            self.hits = 0
            return 0, 0, 'STOP_CAMERA_TIME_SKEW'
        if now < self.ready_at:
            return 0, 0, 'STAGE_PAUSE'
        if self.started is None:
            self.started = now
        if not matched:
            self.hits = 0
            return stage['steering'], stage['pwm'], stage['name']
        # Count each frame set once so a frozen camera cannot confirm a target.
        if self.last_stamps is None or all(a > b for a, b in zip(stamps, self.last_stamps)):
            self.last_stamps = stamps
            self.hits += 1
        if self.hits >= self.confirm_frames:
            self.index += 1
            self.started, self.hits, self.last_stamps = None, 0, None
            self.ready_at = now+self.pause
        return 0, 0, f"CONFIRM_{stage['name']}"


class ParkingNode(Node):
    def __init__(self):
        super().__init__('parking_node')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.record_only = param('record_only', False)
        profile = param('profile', '')
        self.width, self.height = param('image_width', 640), param('image_height', 480)
        self.confidence, self.merge_iou = param('confidence', 0.5), param('merge_iou', 0.5)
        self.sequence = None if self.record_only else StageSequence(
            json.loads(Path(profile).read_text(encoding='utf-8')),
            param('confirm_frames', 5), param('stale_timeout', 1.0),
            param('max_skew', 0.5), param('stage_pause', 0.5))

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.observations = {}
        for camera in CAMERAS:
            self.create_subscription(DetectionArray, param(f'{camera}_detection_topic', f'detections_{camera}'),
                                     lambda msg, name=camera: self.detection_callback(name, msg), qos)
        self.publisher = self.create_publisher(MotionCommand, param('pub_topic', 'topic_control_signal'), qos)
        self.status_pub = self.create_publisher(String, 'parking_status', 5)
        self.bbox_pub = self.create_publisher(String, 'parking_bboxes', 5)
        self.last_status = None
        self.create_timer(TIMER, self.timer_callback)

    def detection_callback(self, camera, msg):
        self.observations[camera] = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
            'boxes': normalize(msg, self.width, self.height, self.confidence, self.merge_iou)}

    def timer_callback(self):
        steering, pwm = 0, 0
        if self.record_only:
            status = 'RECORD_ONLY'
        else:
            try:
                steering, pwm, status = self.sequence.step(self.observations, self.get_clock().now().nanoseconds/1e9)
            except Exception as exc:
                self.sequence.fault = status = f'STOP_ERROR: {exc}'
        self.publish(steering, pwm)
        self.bbox_pub.publish(String(data=json.dumps(self.observations)))
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def publish(self, steering, pwm):
        self.publisher.publish(MotionCommand(steering=steering, left_speed=pwm, right_speed=pwm))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ParkingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish(0, 0)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
