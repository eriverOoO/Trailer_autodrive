#!/usr/bin/env python3
"""Hardware-only diagnostic publisher. No settings, polling, or command TX."""
import argparse
import json
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from imu_common import serial_arguments, make_parser, open_serial


class Receiver(Node):
    def __init__(self, args):
        super().__init__('ebimu_diagnostic_receiver')
        self.args = args
        self.parser = make_parser(args)
        self.ser = open_serial(args)
        self.pub = self.create_publisher(Imu, args.topic, qos_profile_sensor_data)
        self.diag = self.create_publisher(DiagnosticArray, args.namespace+'/diagnostics', 10)
        args.output.mkdir(parents=True, exist_ok=False)
        self.raw = (args.output/'rx.bin').open('xb')
        self.log = (args.output/'samples.jsonl').open('x')
        self.started = time.monotonic()
        self.last = None
        self.last_stamp = 0
        self.rx_bytes = self.messages = self.sensor_gaps = self.sensor_resets = 0
        self.last_sensor_ms = None
        self.error = ''
        self.create_timer(.002, self.poll)
        self.create_timer(.2, self.health)
        self.get_logger().info('REAL UART ONLY; TX=0; '+args.port+' -> '+args.topic)

    def poll(self):
        if self.error:
            return
        try:
            data = self.ser.read(min(8192, self.ser.in_waiting))
            if not data:
                return
            self.raw.write(data)
            self.rx_bytes += len(data)
            samples = self.parser.feed(data)
            receipt = time.monotonic()
            for sample in samples:
                stamp = self.get_clock().now().nanoseconds
                if stamp <= self.last_stamp:
                    continue
                m = Imu()
                m.header.frame_id = self.args.frame
                m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
                m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = sample['q']
                # Zero covariance: unknown, per sensor_msgs/Imu convention.
                if sample['gyro'] is None:
                    m.angular_velocity_covariance[0] = -1.
                else:
                    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = sample['gyro']
                if sample['accel'] is None:
                    m.linear_acceleration_covariance[0] = -1.
                else:
                    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = sample['accel']
                t = sample['sensor_ms']
                if t is not None and self.last_sensor_ms is not None:
                    dt = t-self.last_sensor_ms
                    if dt <= 0:
                        self.sensor_resets += 1
                    elif dt > self.args.expected_period_ms*1.5:
                        self.sensor_gaps += max(1, round(dt/self.args.expected_period_ms)-1)
                self.last_sensor_ms = t
                self.pub.publish(m)
                self.messages += 1
                self.last_stamp, self.last = stamp, receipt
                self.log.write(json.dumps(dict(host_ns=stamp, receipt_monotonic=receipt, **sample))+'\n')
        except (OSError, ValueError) as exc:
            self.error = repr(exc)
            self.ser.close()
            self.get_logger().error('UART stopped: '+self.error)

    def health(self):
        age = None if self.last is None else time.monotonic()-self.last
        state = 'NO DATA' if age is None else ('STALE' if age > .5 else 'LIVE')
        d = DiagnosticStatus()
        d.name = self.args.namespace+'/real_uart'
        d.hardware_id = self.args.port
        d.level = DiagnosticStatus.OK if state == 'LIVE' else DiagnosticStatus.ERROR
        d.message = state+(' '+self.error if self.error else '')
        values = dict(rx_bytes=self.rx_bytes, tx_bytes=0, messages=self.messages,
                      valid_age_s=age, parse_rejected=self.parser.rejected,
                      estimated_sensor_missing=self.sensor_gaps, sensor_resets=self.sensor_resets,
                      coordinate_reference='manufacturer native; ROS ENU alignment unverified',
                      stamp_basis='host parse/publication time; not synchronized sensor time')
        d.values = [KeyValue(key=k, value=str(v)) for k,v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [d]
        self.diag.publish(msg)
        self.raw.flush()
        self.log.flush()

    def destroy_node(self):
        self.ser.close()
        self.raw.close()
        self.log.close()
        (self.args.output/'summary.json').write_text(json.dumps(dict(
            rx_bytes=self.rx_bytes, tx_bytes=0, messages=self.messages,
            accepted=self.parser.accepted, rejected=self.parser.rejected,
            estimated_sensor_missing=self.sensor_gaps, sensor_resets=self.sensor_resets,
            duration_s=time.monotonic()-self.started, error=self.error), indent=2))
        return super().destroy_node()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    serial_arguments(p)
    p.add_argument('--namespace', default='/ebimu_diag/tractor')
    p.add_argument('--frame', default='ebimu_diag_tractor_sensor')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--expected-period-ms', type=float, default=10.)
    p.add_argument('--seconds', type=float, default=0., help='0: until Ctrl+C')
    args = p.parse_args()
    if not args.namespace.startswith('/ebimu_diag/') or not args.frame.startswith('ebimu_diag_'):
        p.error('diagnostic namespace/frame required')
    if args.output.exists():
        p.error('output must be a new directory')
    args.topic = args.namespace+'/imu/data'
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stop = False
    def request_stop(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node = Receiver(args)
        start = time.monotonic()
        while rclpy.ok() and not stop and (args.seconds <= 0 or time.monotonic()-start < args.seconds):
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
