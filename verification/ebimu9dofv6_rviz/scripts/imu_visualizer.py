#!/usr/bin/env python3
"""RViz markers with independent NO DATA/STALE watchdog. No TF or control topics."""
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from imu_common import quaternion_euler, rotate


class Visualizer(Node):
    def __init__(self, args):
        super().__init__('ebimu_diagnostic_visualizer')
        self.args = args
        self.pub = self.create_publisher(MarkerArray, args.namespace+'/markers', 1)
        self.sub = self.create_subscription(Imu, args.namespace+'/imu/data', self.receive, qos_profile_sensor_data)
        self.last = self.q = self.stamp = None
        self.rpy = (0., 0., 0.)
        self.times = deque(maxlen=501)
        self.count = self.invalid = 0
        self.previous_stamp = None
        self.started = time.monotonic()
        self.last_state = None
        args.output.mkdir(parents=True, exist_ok=False)
        self.file = (args.output/'orientation.csv').open('x')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['receipt_monotonic_s','host_stamp_ns','roll_deg','pitch_deg','yaw_deg',
                           'quaternion_norm','gx_rad_s','gy_rad_s','gz_rad_s','ax_m_s2','ay_m_s2','az_m_s2','host_stamp_age_ms'])
        self.events = (args.output/'status.jsonl').open('x')
        self.create_timer(.05, self.display)

    def receive(self, m):
        q = (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)
        norm = math.sqrt(sum(v*v for v in q))
        stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if (m.header.frame_id != self.args.sensor_frame or m.orientation_covariance[0] == -1
                or not math.isfinite(norm) or abs(norm-1.) > .01
                or age > self.args.stale or age < -.05
                or (self.previous_stamp is not None and stamp <= self.previous_stamp)):
            self.invalid += 1
            return
        self.previous_stamp = stamp
        self.last = time.monotonic()
        self.stamp = stamp
        self.times.append(self.last)
        self.q = tuple(v/norm for v in q)
        self.rpy = tuple(map(math.degrees, quaternion_euler(self.q)))
        self.count += 1
        g = (m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z) if m.angular_velocity_covariance[0] != -1 else ('','','')
        a = (m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z) if m.linear_acceleration_covariance[0] != -1 else ('','','')
        self.csv.writerow([self.last,stamp,*self.rpy,norm,*g,*a,age*1000])

    def marker(self, ident, kind):
        m = Marker()
        m.header.frame_id = self.args.fixed_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = self.args.namespace+'/REAL_HARDWARE'
        m.id, m.type, m.action = ident, kind, Marker.ADD
        m.pose.orientation.w = 1.
        m.color.a = 1.
        m.lifetime.sec = 1
        return m

    def display(self):
        now = time.monotonic()
        age = None if self.last is None else max(now-self.last, (self.get_clock().now().nanoseconds-self.stamp)/1e9)
        state = 'NO DATA' if age is None else ('STALE' if age > self.args.stale else 'LIVE')
        if state != self.last_state:
            event = dict(elapsed_s=now-self.started, status=state, data_age_s=age,
                         accepted=self.count, invalid=self.invalid)
            self.events.write(json.dumps(event)+'\n')
            self.events.flush()
            self.get_logger().info(json.dumps(event))
            self.last_state = state
        active = [t for t in self.times if now-t <= 2.]
        rate = (len(active)-1)/(active[-1]-active[0]) if len(active)>1 else 0.
        age_text = '--' if age is None else '%.1f' % (age*1000)
        values = ['Roll : --', 'Pitch: --', 'Yaw  : --'] if self.q is None else [
            'Roll : %.2f deg' % self.rpy[0], 'Pitch: %.2f deg' % self.rpy[1], 'Yaw  : %.2f deg' % self.rpy[2]]
        lines = ['EBIMU-9DOFV6 REAL UART', self.args.label, *values,
                 'Data Rate: %.1f Hz' % rate, 'Data Age: %s ms' % age_text,
                 'Status:'+state, 'Native axes; ENU unverified',
                 'LAST POSE - NOT LIVE' if state == 'STALE' else '']
        markers = []
        for i, line in enumerate(lines):
            text = self.marker(10+i, Marker.TEXT_VIEW_FACING)
            text.pose.position.z = .72-i*.048
            text.scale.z = .028
            text.color.r, text.color.g, text.color.b = (0.2,1.,0.3) if state == 'LIVE' else (1.,.2,.1)
            if line:
                # This host's Ogre font renders ASCII spaces with excessive width.
                # Visible separators keep every status/value within the viewport.
                text.text = line.replace(' ', '_')
            else:
                # Ogre 1.12 may fail allocating a zero-vertex whitespace text buffer.
                text.action = Marker.DELETE
            markers.append(text)
        if self.q is not None:
            cube = self.marker(0, Marker.CUBE)
            cube.scale.x, cube.scale.y, cube.scale.z = .18,.12,.045
            cube.pose.orientation.x,cube.pose.orientation.y,cube.pose.orientation.z,cube.pose.orientation.w = self.q
            cube.color.r,cube.color.g,cube.color.b = (.7,.7,.7) if state == 'STALE' else (.9,.65,.12)
            markers.append(cube)
            for i, (v, color) in enumerate([((.25,0,0),(1.,0.,0.)),((0,.25,0),(0.,1.,0.)),((0,0,.25),(0.,0.,1.))]):
                arrow = self.marker(i+1, Marker.ARROW)
                tip = rotate(self.q, v)
                arrow.points = [Point(x=0.,y=0.,z=0.),Point(x=tip[0],y=tip[1],z=tip[2])]
                arrow.scale.x,arrow.scale.y,arrow.scale.z = .008,.018,.025
                arrow.color.r,arrow.color.g,arrow.color.b = color
                arrow.color.a = 1. if state == 'LIVE' else .3
                markers.append(arrow)
        self.pub.publish(MarkerArray(markers=markers))
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        self.events.close()
        (self.args.output/'summary.json').write_text(json.dumps(dict(received=self.count, invalid=self.invalid,
            last_status=self.last_state, duration_s=time.monotonic()-self.started), indent=2))
        return super().destroy_node()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--namespace', default='/ebimu_diag/tractor')
    p.add_argument('--sensor-frame', default='ebimu_diag_tractor_sensor')
    p.add_argument('--fixed-frame', default='ebimu_diag_reference')
    p.add_argument('--label', default='Tractor IMU (one physical sensor)')
    p.add_argument('--stale', type=float, default=.5)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=0.)
    args = p.parse_args()
    if (not args.namespace.startswith('/ebimu_diag/') or not args.fixed_frame.startswith('ebimu_diag_')
            or args.stale <= 0 or args.output.exists()):
        p.error('diagnostic namespace/frame, positive timeout, new output directory required')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stop = False
    def request_stop(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node = Visualizer(args)
        started = time.monotonic()
        while rclpy.ok() and not stop and (args.seconds <= 0 or time.monotonic()-started < args.seconds):
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
