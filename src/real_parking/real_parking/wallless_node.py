"""Read-only field diagnostic: current SLAM pose, scan support and wallless gaps."""
import json
import math
import time
from dataclasses import fields
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from .wallless import Config, Detector, scan_clusters


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class WalllessValidation(Node):
    def __init__(self):
        super().__init__('wallless_validation')
        for name, default in [('map_topic', '/map'), ('scan_topic', '/scan'),
                              ('fixed_frame', 'map'), ('ego_frame', 'base_footprint'),
                              ('output_file', ''), ('max_age', 1.5), ('max_map_age', 5.0)]:
            self.declare_parameter(name, default)
        defaults = Config()
        for f in fields(Config):
            self.declare_parameter(f.name, getattr(defaults, f.name))
        self.detector = Detector(Config(**{f.name: self.get_parameter(f.name).value for f in fields(Config)}))
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.scan = None; self.scan_wall = 0.; self.grid = None; self.grid_wall = 0.
        self.previous = None; self.result = {}; self.error = 'waiting_for_map'
        self.create_subscription(LaserScan, self.p('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.p('map_topic'), self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/wallless/reference_label', self.on_reference, 10)
        self.pub = self.create_publisher(String, '/wallless/status', 10)
        self.markers = self.create_publisher(MarkerArray, '/wallless/markers', 10)
        self.file = None
        if self.p('output_file'):
            p = Path(self.p('output_file')).expanduser(); p.parent.mkdir(parents=True, exist_ok=True)
            self.file = p.open('a', buffering=1)
        self.create_timer(.5, self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    def on_scan(self, msg):
        self.scan = msg; self.scan_wall = time.monotonic()

    def pose(self, stamp=None):
        t = self.buffer.lookup_transform(self.p('fixed_frame'), self.p('ego_frame'), stamp or Time())
        return [t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)], t

    def write(self, event):
        if self.file:
            self.file.write(json.dumps(event, allow_nan=False)+'\n')

    def on_reference(self, msg):
        # Operator stops at a physically marked point with the same heading.
        try:
            p, t = self.pose()
            self.write(dict(event='reference', label=msg.data, pose=p,
                            stamp=Time.from_msg(t.header.stamp).nanoseconds/1e9))
        except Exception as exc:
            self.get_logger().warning(f'Reference not recorded: {exc}')

    def on_map(self, msg):
        self.grid = msg; self.grid_wall = time.monotonic()
        try:
            if msg.header.frame_id != self.p('fixed_frame'):
                raise ValueError('map frame does not match fixed_frame')
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
            if not -.1 <= age <= self.p('max_map_age'):
                raise ValueError('map timestamp stale or future')
            p, _ = self.pose(stamp)
            origin = msg.info.origin
            result = self.detector.update(np.array(msg.data).reshape(msg.info.height, msg.info.width),
                                         msg.info.resolution,
                                         [origin.position.x, origin.position.y, yaw(origin.orientation)],
                                         p, stamp.nanoseconds/1e9)
            if result['reason'] != 'duplicate_or_reversed_map_stamp':
                self.result = result
            self.error = ''
        except Exception as exc:
            self.error = str(exc); self.result = {}

    def tick(self):
        now = self.get_clock().now().nanoseconds/1e9; wall = time.monotonic()
        event = dict(event='sample', time=now, diagnostic_only=True, error=self.error)
        try:
            p, t = self.pose(); stamp = Time.from_msg(t.header.stamp).nanoseconds/1e9
            age = now-stamp
            scan_age = now-Time.from_msg(self.scan.header.stamp).nanoseconds/1e9 if self.scan else None
            fresh = (self.scan is not None and -.1 <= age <= self.p('max_age') and
                     -.1 <= scan_age <= self.p('max_age') and wall-self.scan_wall <= self.p('max_age') and
                     self.grid is not None and wall-self.grid_wall <= self.p('max_map_age') and
                     -.1 <= now-Time.from_msg(self.grid.header.stamp).nanoseconds/1e9 <= self.p('max_map_age'))
            support = {}
            if self.scan:
                r = np.asarray(self.scan.ranges); a = self.scan.angle_min+np.arange(len(r))*self.scan.angle_increment
                valid = np.isfinite(r) & (r >= self.scan.range_min) & (r <= self.scan.range_max)
                event['scan_clusters'] = dict(frame=self.scan.header.frame_id, objects=scan_clusters(
                    self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                    self.scan.range_min, self.scan.range_max))
                support = dict(valid_returns=int(valid.sum()), total_beams=len(r),
                               occupied_angular_sectors=int(len(np.unique(np.floor((a[valid] % (2*math.pi))/(math.pi/4))))))
            event.update(pose=p, pose_age=age, scan_age=scan_age, inputs_fresh=fresh, scan_support=support,
                         perception=self.result if fresh and not self.error else {})
            if self.previous and stamp > self.previous[0]:
                event['pose_step_m'] = float(np.linalg.norm(np.array(p[:2])-self.previous[1][:2]))
                event['yaw_step_rad'] = abs(math.atan2(math.sin(p[2]-self.previous[1][2]), math.cos(p[2]-self.previous[1][2])))
            if not self.previous or stamp > self.previous[0]:
                self.previous = (stamp, p)
        except Exception as exc:
            event.update(inputs_fresh=False, error=str(exc))
        self.pub.publish(String(data=json.dumps(event, allow_nan=False))); self.write(event)
        markers = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; markers.markers.append(clear)
        for i, gap in enumerate(event.get('perception', {}).get('gaps', [])):
            marker = Marker(); marker.header.frame_id = self.p('fixed_frame')
            marker.header.stamp = self.get_clock().now().to_msg(); marker.ns = 'gap_hypotheses'; marker.id = i
            marker.type = Marker.CUBE; marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = gap['center']; marker.pose.position.z = .05
            marker.pose.orientation.z = math.sin(gap['yaw']/2); marker.pose.orientation.w = math.cos(gap['yaw']/2)
            marker.scale.x = gap['length']; marker.scale.y = gap['width']; marker.scale.z = .1
            marker.color.a = .45; marker.color.g = 1. if gap['accepted'] else 0.; marker.color.r = 0. if gap['accepted'] else 1.
            markers.markers.append(marker)
        self.markers.publish(markers)

    def destroy_node(self):
        if self.file:
            self.file.close()
        super().destroy_node()


def main():
    rclpy.init(); node = WalllessValidation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
