"""BBox-triggered parallel parking state machine for the articulated stroller.

Existing front/rear YOLO nodes publish DetectionArray messages.  This single
parking node selects the nearest requested object, advances explicit parking
states from normalized bbox conditions, and publishes the existing Arduino
MotionCommand contract.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from interfaces_pkg.msg import DetectionArray, MotionCommand


CAMERAS = ('front', 'rear')
STOP_STATES = ('STOP_BEFORE_REVERSE', 'STOP_BEFORE_FORWARD')
FINAL_STATES = ('PARKED',)


def iou(a, b):
    """Return IoU for normalized [cx, cy, width, height] boxes."""
    ax1, ay1, ax2, ay2 = a[0]-a[2]/2, a[1]-a[3]/2, a[0]+a[2]/2, a[1]+a[3]/2
    bx1, by1, bx2, by2 = b[0]-b[2]/2, b[1]-b[3]/2, b[0]+b[2]/2, b[1]+b[3]/2
    overlap = max(0.0, min(ax2, bx2)-max(ax1, bx1))
    overlap *= max(0.0, min(ay2, by2)-max(ay1, by1))
    union = a[2]*a[3] + b[2]*b[3] - overlap
    return overlap/union if union > 0 else 0.0


def normalize(msg, width, height, confidence, merge_iou):
    """Convert DetectionArray to usable normalized boxes."""
    boxes = []
    for detection in sorted(msg.detections, key=lambda item: -item.score):
        cx = detection.bbox.center.position.x
        cy = detection.bbox.center.position.y
        w, h = detection.bbox.size.x, detection.bbox.size.y
        values = (cx, cy, w, h, detection.score)
        if not all(math.isfinite(value) for value in values):
            continue
        # A border-clipped detection cannot provide a reliable size trigger.
        if (detection.score < confidence or w <= 0 or h <= 0 or
                cx-w/2 <= 2 or cy-h/2 <= 2 or
                cx+w/2 >= width-2 or cy+h/2 >= height-2):
            continue
        box = [cx/width, cy/height, w/width, h/height]
        if any(item['class'] == detection.class_name and
               iou(item['box'], box) > merge_iou for item in boxes):
            continue
        boxes.append({
            'class': detection.class_name,
            'score': round(float(detection.score), 3),
            'box': [round(value, 4) for value in box],
        })
    return boxes


def nearest_box(observation, class_name):
    """Choose the largest same-class box, representing the nearest object."""
    candidates = [
        item for item in observation.get('boxes', ())
        if item['class'] == class_name
    ]
    return max(candidates, key=lambda item: item['box'][2]*item['box'][3],
               default=None)


class ParkingStateMachine:
    """Explicit maneuver states adapted from the original parking mission."""

    ORDER = (
        'WAIT_TARGET',
        'APPROACH',
        'FORWARD_SIDE_SETUP',
        'STOP_BEFORE_REVERSE',
        'REVERSE_ENTRY',
        'REVERSE_COUNTER_STEER',
        'REVERSE_STRAIGHT',
        'STOP_BEFORE_FORWARD',
        'FORWARD_ALIGN',
        'PARKED',
    )

    def __init__(self, config):
        self.config = config
        self._validate()
        self.reset()

    def _validate(self):
        c = self.config
        if c['parking_side'] not in (-1, 1):
            raise ValueError('parking_side must be -1 (left) or 1 (right)')
        if not 1 <= c['steer'] <= 7:
            raise ValueError('steer must be 1..7')
        for name in ('approach_pwm', 'setup_pwm', 'reverse_pwm', 'fine_pwm'):
            if not 1 <= c[name] <= 255:
                raise ValueError(f'{name} must be 1..255')
        for name in ('approach_front_width', 'setup_front_cx',
                     'reverse_entry_rear_width', 'counter_rear_width',
                     'reverse_stop_rear_width', 'final_front_width',
                     'final_rear_width', 'final_front_cx', 'final_rear_cx'):
            if not 0 < c[name] < 1:
                raise ValueError(f'{name} must be normalized to 0..1')
        for name in ('width_tolerance', 'center_tolerance'):
            if not 0 < c[name] < 1:
                raise ValueError(f'{name} must be normalized to 0..1')
        if not c['front_class'] or not c['rear_class']:
            raise ValueError('front_class and rear_class cannot be empty')
        if not (c['reverse_entry_rear_width'] <=
                c['counter_rear_width'] <= c['reverse_stop_rear_width']):
            raise ValueError('rear width triggers must increase in maneuver order')
        if (c['confirm_frames'] < 1 or c['stale_timeout'] <= 0 or
                c['state_timeout'] <= 0 or c['gear_pause'] < 0):
            raise ValueError('invalid timing parameter')

    def reset(self, now=0.0):
        self.index = 0
        self.state_started = now
        self.hits = 0
        self.last_stamps = None
        self.fault = None

    @property
    def state(self):
        return self.ORDER[self.index]

    def _transition(self, now):
        if self.index < len(self.ORDER)-1:
            self.index += 1
        self.state_started = now
        self.hits = 0
        self.last_stamps = None

    def _target(self, observations, camera, class_name, now):
        observation = observations.get(camera)
        if observation is None:
            return None, None, f'WAIT_{camera.upper()}_DETECTION'
        age = now-observation['stamp']
        if not 0 <= age <= self.config['stale_timeout']:
            return None, None, f'STOP_{camera.upper()}_STALE'
        target = nearest_box(observation, class_name)
        if target is None:
            return None, None, f'WAIT_{camera.upper()}_{class_name}'
        return target['box'], observation['stamp'], None

    def _condition(self, observations, now):
        c, state = self.config, self.state
        front_class, rear_class = c['front_class'], c['rear_class']
        if state in ('WAIT_TARGET', 'APPROACH', 'FORWARD_SIDE_SETUP'):
            front, stamp, error = self._target(
                observations, 'front', front_class, now)
            if error:
                return False, (), error
            if state == 'WAIT_TARGET':
                return True, (stamp,), None
            if state == 'APPROACH':
                return front[2] >= c['approach_front_width'], (stamp,), None
            reached = (front[0] <= c['setup_front_cx']
                       if c['parking_side'] == 1
                       else front[0] >= 1.0-c['setup_front_cx'])
            return reached, (stamp,), None
        if state in ('REVERSE_ENTRY', 'REVERSE_COUNTER_STEER',
                     'REVERSE_STRAIGHT'):
            rear, stamp, error = self._target(
                observations, 'rear', rear_class, now)
            if error:
                return False, (), error
            threshold = {
                'REVERSE_ENTRY': c['reverse_entry_rear_width'],
                'REVERSE_COUNTER_STEER': c['counter_rear_width'],
                'REVERSE_STRAIGHT': c['reverse_stop_rear_width'],
            }[state]
            return rear[2] >= threshold, (stamp,), None
        if state == 'FORWARD_ALIGN':
            front, front_stamp, error = self._target(
                observations, 'front', front_class, now)
            if error:
                return False, (), error
            rear, rear_stamp, error = self._target(
                observations, 'rear', rear_class, now)
            if error:
                return False, (), error
            centered = (
                abs(front[0]-c['final_front_cx']) <= c['center_tolerance'] and
                abs(rear[0]-c['final_rear_cx']) <= c['center_tolerance'])
            spaced = (
                abs(front[2]-c['final_front_width']) <= c['width_tolerance'] and
                abs(rear[2]-c['final_rear_width']) <= c['width_tolerance'])
            return centered and spaced, (front_stamp, rear_stamp), None
        return False, (), None

    def _command(self):
        c, side, state = self.config, self.config['parking_side'], self.state
        commands = {
            'WAIT_TARGET': (0, 0),
            'APPROACH': (0, c['approach_pwm']),
            'FORWARD_SIDE_SETUP': (side*c['steer'], c['setup_pwm']),
            'STOP_BEFORE_REVERSE': (0, 0),
            # Reversing with opposite steering first creates trailer angle.
            'REVERSE_ENTRY': (-side*c['steer'], -c['reverse_pwm']),
            'REVERSE_COUNTER_STEER': (side*c['steer'], -c['reverse_pwm']),
            'REVERSE_STRAIGHT': (0, -c['fine_pwm']),
            'STOP_BEFORE_FORWARD': (0, 0),
            'FORWARD_ALIGN': (0, c['fine_pwm']),
            'PARKED': (0, 0),
        }
        return commands[state]

    def step(self, observations, now):
        """Return (steering, pwm, status) for the current sensor snapshot."""
        if self.fault:
            return 0, 0, self.fault
        state = self.state
        if state in FINAL_STATES:
            return 0, 0, state
        if state in STOP_STATES:
            if now-self.state_started >= self.config['gear_pause']:
                self._transition(now)
            return 0, 0, state
        if state != 'WAIT_TARGET' and (
                now-self.state_started > self.config['state_timeout']):
            self.fault = f'STOP_TIMEOUT_{state}'
            return 0, 0, self.fault

        reached, stamps, error = self._condition(observations, now)
        if error:
            self.hits = 0
            return 0, 0, error
        if not reached:
            self.hits = 0
            steering, pwm = self._command()
            return steering, pwm, state

        # A frozen frame cannot satisfy a transition repeatedly.
        if self.last_stamps is None or all(
                current > previous
                for current, previous in zip(stamps, self.last_stamps)):
            self.last_stamps = stamps
            self.hits += 1
        if self.hits >= self.config['confirm_frames']:
            completed = state
            self._transition(now)
            return 0, 0, f'{completed}_COMPLETE'
        return 0, 0, f'CONFIRM_{state}'


class ParkingNode(Node):
    def __init__(self):
        super().__init__('parking_node')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.record_only = bool(param('record_only', False))
        self.enabled = bool(param('enabled_at_start', False))
        self.width = int(param('image_width', 640))
        self.height = int(param('image_height', 480))
        self.confidence = float(param('confidence', 0.5))
        self.merge_iou = float(param('merge_iou', 0.5))
        timer_period = float(param('timer', 0.1))
        if (self.width <= 0 or self.height <= 0 or timer_period <= 0 or
                not 0 <= self.confidence <= 1 or
                not 0 <= self.merge_iou <= 1):
            raise ValueError('invalid image, timer, confidence, or IoU parameter')
        config = {
            # The parking space is on the vehicle's left by default.
            'parking_side': int(param('parking_side', -1)),
            'steer': int(param('steer', 6)),
            'approach_pwm': int(param('approach_pwm', 70)),
            'setup_pwm': int(param('setup_pwm', 70)),
            'reverse_pwm': int(param('reverse_pwm', 90)),
            'fine_pwm': int(param('fine_pwm', 60)),
            'front_class': str(param('front_class', 'car_back')),
            'rear_class': str(param('rear_class', 'car_front')),
            # Initial estimates only; replace from /parking_bboxes recordings.
            'approach_front_width': float(param('approach_front_width', 0.18)),
            'setup_front_cx': float(param('setup_front_cx', 0.30)),
            'reverse_entry_rear_width': float(param(
                'reverse_entry_rear_width', 0.16)),
            'counter_rear_width': float(param('counter_rear_width', 0.24)),
            'reverse_stop_rear_width': float(param(
                'reverse_stop_rear_width', 0.32)),
            'final_front_width': float(param('final_front_width', 0.24)),
            'final_rear_width': float(param('final_rear_width', 0.24)),
            'final_front_cx': float(param('final_front_cx', 0.50)),
            'final_rear_cx': float(param('final_rear_cx', 0.50)),
            'width_tolerance': float(param('width_tolerance', 0.04)),
            'center_tolerance': float(param('center_tolerance', 0.08)),
            'confirm_frames': int(param('confirm_frames', 5)),
            'stale_timeout': float(param('stale_timeout', 1.0)),
            'state_timeout': float(param('state_timeout', 12.0)),
            'gear_pause': float(param('gear_pause', 0.7)),
        }
        self.machine = ParkingStateMachine(config)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.observations = {}
        for camera in CAMERAS:
            topic = str(param(
                f'{camera}_detection_topic', f'detections_{camera}'))
            self.create_subscription(
                DetectionArray, topic,
                lambda msg, name=camera: self.detection_callback(name, msg),
                qos)
        self.create_subscription(
            Bool, str(param('enable_topic', 'parking_enable')),
            self.enable_callback, qos)
        self.publisher = self.create_publisher(
            MotionCommand, str(param('pub_topic', 'topic_control_signal')), qos)
        self.status_pub = self.create_publisher(
            String, 'parking_status', 5)
        self.bbox_pub = self.create_publisher(
            String, 'parking_bboxes', 5)
        self.last_status = None
        self.create_timer(timer_period, self.timer_callback)

    def enable_callback(self, msg):
        if msg.data and not self.enabled:
            self.machine.reset(self.get_clock().now().nanoseconds/1e9)
        if not msg.data:
            self.machine.reset()
        self.enabled = bool(msg.data)

    def detection_callback(self, camera, msg):
        self.observations[camera] = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
            'boxes': normalize(
                msg, self.width, self.height,
                self.confidence, self.merge_iou),
        }

    def timer_callback(self):
        steering, pwm = 0, 0
        if self.record_only:
            status = 'RECORD_ONLY'
        elif not self.enabled:
            status = 'PARKING_DISABLED'
        else:
            try:
                now = self.get_clock().now().nanoseconds/1e9
                steering, pwm, status = self.machine.step(
                    self.observations, now)
            except Exception as exc:
                self.machine.fault = status = f'STOP_ERROR: {exc}'
        self.publish_motion(steering, pwm)
        self.bbox_pub.publish(String(data=json.dumps(self.observations)))
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def publish_motion(self, steering, pwm):
        self.publisher.publish(MotionCommand(
            steering=int(steering),
            left_speed=int(pwm),
            right_speed=int(pwm)))


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
            # Humble's SIGINT handler may invalidate the context before this
            # finally block runs.  Only publish when the publisher is usable.
            if rclpy.ok():
                node.publish_motion(0, 0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
