#!/usr/bin/env python3
"""Keyboard Ackermann teleoperation and live SLAM diagnostics (no device startup)."""
import argparse
from collections import deque
import curses
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class KeyboardDrive:
    """Motion expires without repeated movement keys; steering alone cannot renew it."""
    def __init__(self, speed, reverse_speed, steer, wheelbase, timeout):
        self.speed, self.reverse_speed = speed, reverse_speed
        self.steer, self.wheelbase, self.timeout = steer, wheelbase, timeout
        self.velocity = self.angle = 0.0
        self.last_motion = -math.inf

    def key(self, key, now):
        motions = {'w': (1, 0), 'e': (1, -1), 'q': (1, 1),
                   's': (-1, 0), 'c': (-1, -1), 'z': (-1, 1)}
        if key in motions:
            direction, steering = motions[key]
            self.velocity = self.speed if direction > 0 else -self.reverse_speed
            self.angle = steering * self.steer
            self.last_motion = now
        elif key in (' ', 'x'):
            self.velocity = 0.0
            self.last_motion = -math.inf

    def command(self, now):
        velocity = self.velocity if now - self.last_motion < self.timeout else 0.0
        return velocity, velocity * math.tan(self.angle) / self.wheelbase


class SlamCheck(Node):
    def __init__(self, args):
        super().__init__('manual_slam_check')
        self.args = args
        self.drive = KeyboardDrive(args.speed, args.reverse_speed, math.radians(args.steer_deg),
                                   args.wheelbase, args.key_timeout)
        self.publisher = self.create_publisher(Twist, args.command_topic, 1)
        self.ready = False
        self.motor_status = 'waiting for bridge status'
        self.motor_status_wall = -math.inf
        if args.motor_pwm:
            self.create_subscription(String, '/parking/manual_motor_status', self.on_motor_status, 1)
        self.ready_wall = -math.inf
        if args.require_ready:
            self.create_subscription(Bool, '/vehicle_viz_2d/ready', self.on_ready, 1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.receipts = {name: deque(maxlen=200) for name in ('scan', 'map', 'match')}
        self.counts = dict.fromkeys(self.receipts, 0)
        self.scan = None
        self.map_info = None
        self.pose = None
        self.tf_error = 'waiting for scan'
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
        # Volatile works with both volatile and transient-local map publishers.
        self.create_subscription(OccupancyGrid, args.map_topic, self.on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PointCloud2, '/scan_matched_points2',
                                 lambda msg: self.received('match'), qos_profile_sensor_data)
        self.create_timer(0.05, self.publish_command)
        self.create_timer(0.2, self.lookup)
        self.samples = deque(maxlen=36000)
        self.create_timer(1.0, self.record)

    def received(self, name):
        self.receipts[name].append(time.monotonic())
        self.counts[name] += 1

    def on_scan(self, msg):
        self.received('scan')
        self.scan = msg

    def on_map(self, msg):
        self.received('map')
        self.map_info = {'width': msg.info.width, 'height': msg.info.height,
                         'resolution_m': msg.info.resolution,
                         'known_cells': sum(v >= 0 for v in msg.data),
                         'occupied_cells': sum(v >= 50 for v in msg.data)}

    def lookup(self):
        self.pose = None
        if self.scan is None:
            return
        try:
            # Use acquisition time: a cached latest transform can hide SLAM stalls.
            tf = self.buffer.lookup_transform(self.args.map_frame, self.args.pose_frame,
                                             Time.from_msg(self.scan.header.stamp))
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            self.pose = {'x_m': p.x, 'y_m': p.y, 'yaw_deg': math.degrees(yaw)}
            self.tf_error = ''
        except TransformException as error:
            self.tf_error = str(error).split('\n')[0]

    def on_ready(self, msg):
        self.ready = msg.data
        self.ready_wall = time.monotonic()

    def on_motor_status(self, msg):
        try:
            value = json.loads(msg.data)
            self.motor_status = f"PWM={value['pwm']} steering={value['steering']} {value['reason']}"
            self.motor_status_wall = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.motor_status = 'invalid bridge status'

    def current_command(self):
        now = time.monotonic()
        if self.args.require_ready and (not self.ready or now-self.ready_wall > 0.3):
            return 0., 0.
        return self.drive.command(now)

    def publish_command(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.current_command()
        self.publisher.publish(msg)

    def snapshot(self):
        now = time.monotonic()
        topics = {}
        for name, times in self.receipts.items():
            span = times[-1] - times[0] if len(times) > 1 else 0
            topics[name] = {'count': self.counts[name],
                            'age_s': now - times[-1] if times else None,
                            'hz': (len(times)-1)/span if span else 0.0}
        age = None
        if self.scan is not None:
            stamp = self.scan.header.stamp
            age = (self.get_clock().now().nanoseconds - stamp.sec*10**9 - stamp.nanosec)/1e9
        fresh = lambda name, limit: topics[name]['age_s'] is not None and topics[name]['age_s'] < limit
        pipeline = (fresh('scan', 0.5) and fresh('map', 3.0) and self.pose is not None
                    and age is not None and -0.03 <= age < 0.5
                    and self.map_info['known_cells'] > 0)
        v, w = self.current_command()
        return {'wall_time': time.time(), 'pipeline_receiving': bool(pipeline),
                'topics': topics, 'scan_stamp_age_s': age, 'pose': self.pose,
                'tf_error': self.tf_error, 'map': self.map_info,
                'command': {'speed_m_s': v, 'yaw_rate_rad_s': w}}

    def record(self):
        self.samples.append(self.snapshot())

    def screen(self, screen):
        screen.nodelay(True)
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        while rclpy.ok():
            key = screen.getch()
            if key in (27, ord('Q')):
                break
            if 0 <= key < 256:
                self.drive.key(chr(key), time.monotonic())
            rclpy.spin_once(self, timeout_sec=0.02)
            status = self.snapshot()
            lines = [
                'Manual SLAM check | Q / Esc: quit | Space / x: stop',
                'q w e: forward left/straight/right | z s c: reverse left/straight/right',
                f'Repeat movement key to drive; timeout={self.args.key_timeout:.2f}s',
                f'Twist output: {self.args.command_topic}',
                (f'Motor bridge: fixed PWM +/-{self.args.motor_pwm}; Twist speed is nominal, not measured'
                 if self.args.motor_pwm else 'Motor connection: depends on external controller'),
                (f"Motor: {self.motor_status if time.monotonic()-self.motor_status_wall < .5 else 'BRIDGE STATUS UNAVAILABLE'}"
                 if self.args.motor_pwm else ''),
                f'Startup READY required: {self.args.require_ready} | latest signal: {self.ready}',
                f"SLAM pipeline: {'RECEIVING (accuracy unverified)' if status['pipeline_receiving'] else 'WAIT / STALE'}",
                f"Command: v={status['command']['speed_m_s']:+.3f} m/s, yaw rate={status['command']['yaw_rate_rad_s']:+.3f} rad/s",
            ]
            for name, item in status['topics'].items():
                age = 'none' if item['age_s'] is None else f"{item['age_s']:.2f}s"
                lines.append(f"{name:5}: {item['hz']:6.1f} Hz | age={age} | count={item['count']}")
            lines += [f"Scan timestamp age: {status['scan_stamp_age_s']}",
                      f"Pose ({self.args.pose_frame}): {status['pose']}",
                      f"Map: {status['map']}", f"TF: {status['tf_error'] or 'OK at scan timestamp'}",
                      'Use RViz: inspect wall overlap, jumps, and return-to-start alignment.']
            screen.erase()
            height, width = screen.getmaxyx()
            for row, line in enumerate(lines[:max(0, height-1)]):
                try:
                    screen.addnstr(row, 0, line, max(0, width-1))
                except curses.error:
                    pass
            screen.refresh()

    def stop(self):
        self.drive.key(' ', time.monotonic())
        for _ in range(5):
            if not rclpy.ok():
                break
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.02)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--command-topic', default='/parking/manual_cmd_vel',
                        help='Default is an isolated preview; adapter input is /parking/request_cmd_vel')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--require-ready', action='store_true',
                        help='Hold zero until /vehicle_viz_2d/ready is true and fresh (<0.3s)')
    parser.add_argument('--motor-pwm', type=int, default=0,
                        help='Display fixed PWM used by a separately started manual_motor_bridge')
    parser.add_argument('--pose-frame', default='laser_frame')
    parser.add_argument('--speed', type=float, default=0.08)
    parser.add_argument('--reverse-speed', type=float, default=0.06)
    parser.add_argument('--steer-deg', type=float, default=20.0)
    parser.add_argument('--wheelbase', type=float, default=0.55, help='Measured wheelbase in metres')
    parser.add_argument('--key-timeout', type=float, default=0.35)
    parser.add_argument('--output', type=Path, default=Path('log/manual_slam_check.json'))
    args, ros_args = parser.parse_known_args()
    for name in ('speed', 'reverse_speed', 'wheelbase', 'key_timeout'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if not math.isfinite(args.steer_deg) or not 0 < args.steer_deg < 80:
        parser.error('--steer-deg must be between 0 and 80')
    return args, ros_args


def main():
    args, ros_args = arguments()
    rclpy.init(args=ros_args)
    node = SlamCheck(args)
    try:
        curses.wrapper(node.screen)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
            node.record()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps({
                'command_topic': args.command_topic,
                'accuracy_verified': False,
                'note': 'Topic/TF availability only; samples retain the latest 10 hours.',
                'samples': list(node.samples)}, indent=2, allow_nan=False) + '\n')
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
