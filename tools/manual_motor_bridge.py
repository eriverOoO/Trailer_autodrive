#!/usr/bin/env python3
"""Manual PWM commissioning using ros2_parking's unchanged Arduino protocol.

Twist magnitude is NOT a calibrated speed: any nonzero speed selects PWM 50.
Requires fresh READY, scan and keyboard input. Never enables autonomous control.
"""
import argparse
import json
import math
import signal
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from interfaces_pkg.msg import MotionCommand
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from serial_communication_pkg.lib.protocol_convert_func_lib import convert_serial_message


def motor_command(v, w, pwm, wheelbase=0.55, max_steer=0.45):
    if not math.isfinite(v) or not math.isfinite(w):
        raise ValueError('nonfinite command')
    if abs(v) < 1e-6:
        if abs(w) > 1e-6:
            raise ValueError('cannot rotate in place')
        return None, 0
    angle = math.atan(wheelbase * w / v)
    steering = max(-7, min(7, round(-angle / max_steer * 7)))
    return steering, pwm if v > 0 else -pwm


class ManualMotorBridge(Node):
    def __init__(self, args):
        super().__init__('manual_motor_bridge')
        self.args = args
        self.command = (0., 0.)
        self.pwm_input = getattr(args, 'input_mode', 'twist') == 'pwm'
        self.input_topic = '/parking/manual_pwm' if self.pwm_input else '/parking/manual_cmd_vel'
        self.requested_steering = self.requested_pwm = 0
        self.command_time = self.ready_time = self.scan_time = -math.inf
        self.ready = False
        self.scan = None
        self.estop = getattr(args, 'start_stopped', False)
        self.steering = 0
        self.previous = None
        # Request an advisory exclusive lock for cooperating serial clients.
        self.serial = serial.Serial(args.port, 115200, timeout=0.1,
                                    write_timeout=0.1, exclusive=True)
        time.sleep(2.)  # Arduino may reset on serial open.
        self.stop()
        if self.pwm_input:
            self.create_subscription(MotionCommand, self.input_topic, self.receive_pwm, 1)
        else:
            self.create_subscription(Twist, self.input_topic, self.receive, 1)
        self.create_subscription(Bool, '/vehicle_viz_2d/ready', self.on_ready, 1)
        self.create_subscription(Bool, '/parking/emergency_stop', self.on_estop, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/parking/manual_motor_status', 1)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'MANUAL PWM ONLY: +/-{args.pwm}; input {self.input_topic}; {args.port}')

    def receive_pwm(self, msg):
        if (msg.left_speed != msg.right_speed or abs(msg.left_speed) > self.args.pwm
                or abs(msg.steering) > 7):
            self.estop = True
            return
        self.requested_steering, self.requested_pwm = msg.steering, msg.left_speed
        self.command = (float(msg.left_speed), 0.)  # Direction only for clearance checks.
        self.command_time = time.monotonic()

    def receive(self, msg):
        vals = [msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z]
        if (not all(math.isfinite(v) for v in vals)
                or any(abs(v) > 1e-8 for v in [vals[1], vals[2], vals[3], vals[4]])):
            self.estop = True
            return
        self.command = vals[0], vals[5]
        self.command_time = time.monotonic()

    def on_ready(self, msg):
        self.ready, self.ready_time = msg.data, time.monotonic()

    def on_estop(self, msg):
        self.estop |= msg.data

    def on_scan(self, msg):
        self.scan, self.scan_time = msg, time.monotonic()

    def blocked(self):
        now = time.monotonic()
        if self.estop:
            return 'emergency stop (restart required)'
        if not self.ready or now - self.ready_time > .3:
            return 'READY unavailable'
        if now - self.command_time > .25:
            return 'command timeout'
        if self.count_publishers(self.input_topic) != 1:
            return 'requires exactly one manual publisher'
        if (self.count_publishers('/topic_control_signal')
                or self.count_publishers('/parking/request_cmd_vel')
                or self.count_publishers('/parking/manual_cmd_vel' if self.pwm_input else '/parking/manual_pwm')):
            return 'another control source is active'
        if self.scan is None or now - self.scan_time > .5:
            return 'scan unavailable'
        stamp = self.scan.header.stamp
        age = (self.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e9
        if not -.03 <= age < .5:
            return 'scan timestamp stale'
        v, _ = self.command
        if abs(v) < 1e-6:
            return ''
        # Forward LiDAR-to-nose is measured as 0.895m. Rear threshold includes trailer.
        heading, clearance = (0., 1.2) if v > 0 else (math.pi, 1.5)
        rays = []
        for i, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment - heading
            if abs(math.atan2(math.sin(angle), math.cos(angle))) <= math.radians(20):
                if math.isfinite(distance) and self.scan.range_min <= distance <= self.scan.range_max:
                    rays.append(distance)
        if len(rays) < 5 or min(rays) < clearance:
            return 'directional clearance insufficient or unknown'
        return ''

    def send(self, steering, pwm):
        self.steering = steering
        packet = convert_serial_message(steering, pwm, pwm).encode('ascii')
        if self.serial.write(packet) != len(packet):
            raise OSError('short serial write')

    def stop(self):
        for _ in range(5):
            self.send(self.steering, 0)
            time.sleep(.05)

    def tick(self):
        reason = self.blocked()
        try:
            steering, pwm = ((self.requested_steering, self.requested_pwm) if self.pwm_input
                             else motor_command(*self.command, self.args.pwm))
        except ValueError as error:
            reason = str(error)
            steering, pwm = None, 0
        if reason:
            pwm = 0
            steering = None
        self.send(self.steering if steering is None else steering, pwm)
        self.status.publish(String(data=json.dumps({'pwm': pwm, 'steering': self.steering,
                                                    'reason': reason})))
        state = (pwm, self.steering, reason)
        if state != self.previous:
            self.get_logger().info(f'PWM={pwm} steering={self.steering} reason={reason or "manual command"}')
            self.previous = state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--input-mode', choices=('twist', 'pwm'), default='twist')
    parser.add_argument('--start-stopped', action='store_true', help='Latch emergency stop on startup')
    parser.add_argument('--pwm', type=int, default=50, choices=range(1, 71), metavar='1..70')
    args = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = ManualMotorBridge(args)
        signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop()
            finally:
                node.serial.close()
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
