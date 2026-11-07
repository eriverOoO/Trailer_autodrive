#!/usr/bin/env python3
"""Observe real ROS topics only. Never publish synthetic sensor data or commands."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def summary(values):
    if not values:
        return None
    ordered = sorted(values)
    return {'min': ordered[0], 'median': statistics.median(ordered),
            'p95': ordered[int((len(ordered) - 1) * .95)], 'max': ordered[-1]}


class Probe(Node):
    def __init__(self):
        super().__init__('g6_real_data_probe')
        self.scans = []
        self.last_scan = None
        self.maps = []
        self.matched = []
        self.edges = Counter()
        self.static_parents = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_ok = 0
        self.tf_failures = Counter()
        self.latest_pose = None
        self.poses = []
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/scan_matched_points2', self.match,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/tf', self.tf, 100)
        self.create_subscription(TFMessage, '/tf_static', self.static_tf,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.1, self.lookup)

    def scan(self, msg):
        finite = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        now = time.monotonic()
        self.scans.append({
            'wall': now, 'stamp_ns': stamp_ns(msg.header.stamp), 'frame': msg.header.frame_id,
            'age_s': (self.get_clock().now().nanoseconds - stamp_ns(msg.header.stamp)) / 1e9,
            'beams': len(msg.ranges), 'valid': len(finite),
            'valid_min_m': min(finite) if finite else None,
            'valid_max_m': max(finite) if finite else None,
            'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment, 'scan_time': msg.scan_time,
            'time_increment': msg.time_increment,
            'range_min': msg.range_min, 'range_max': msg.range_max,
            'angle_count_error_beams': abs(msg.angle_max - msg.angle_min -
                                           (len(msg.ranges)-1)*msg.angle_increment) /
                                       msg.angle_increment if msg.angle_increment else None})
        self.last_scan = msg

    def match(self, msg):
        self.matched.append({'stamp_ns': stamp_ns(msg.header.stamp),
                             'points': msg.width * msg.height, 'frame': msg.header.frame_id})

    def map(self, msg):
        self.maps.append({
            'stamp_ns': stamp_ns(msg.header.stamp), 'frame': msg.header.frame_id,
            'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
            'occupied': sum(v >= 50 for v in msg.data),
            'known': sum(v >= 0 for v in msg.data),
            'sha256': hashlib.sha256(bytes(v + 1 for v in msg.data)).hexdigest()})

    def tf(self, msg):
        for transform in msg.transforms:
            self.edges[f'{transform.header.frame_id} -> {transform.child_frame_id}'] += 1

    def static_tf(self, msg):
        for transform in msg.transforms:
            self.static_parents.setdefault(transform.child_frame_id, set()).add(transform.header.frame_id)

    def lookup(self):
        if self.last_scan is None:
            return
        try:
            transform = self.buffer.lookup_transform('map', 'laser_frame', Time())
            # The map must also be available at the real scan's acquisition stamp.
            self.buffer.lookup_transform('map', 'laser_frame', Time.from_msg(self.last_scan.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            self.latest_pose = {'stamp_ns': stamp_ns(transform.header.stamp),
                                'xyz': [t.x, t.y, t.z], 'xyzw': [q.x, q.y, q.z, q.w]}
            self.poses.append(self.latest_pose)
            self.tf_ok += 1
        except TransformException as error:
            self.tf_failures[str(error).split('\n')[0]] += 1

    def result(self, duration, require_slam):
        scans = self.scans
        elapsed = scans[-1]['wall']-scans[0]['wall'] if len(scans) > 1 else 0
        gaps = [b['wall']-a['wall'] for a, b in zip(scans, scans[1:])]
        stamp_gaps = [(b['stamp_ns']-a['stamp_ns'])/1e9 for a, b in zip(scans, scans[1:])]
        scan_ok = bool(elapsed >= 30 and len(scans) >= 200 and
                       all(m['valid'] > 0 and m['frame'] == 'laser_frame' and
                           m['stamp_ns'] > 0 and m['angle_increment'] > 0 and
                           m['angle_count_error_beams'] <= 1.1 for m in scans) and
                       all(g > 0 for g in stamp_gaps) and max(gaps) < .5 and
                       min(m['age_s'] for m in scans) >= -.03 and
                       max(m['age_s'] for m in scans) < .5)
        map_hashes = len({m['sha256'] for m in self.maps})
        slam_ok = bool(scan_ok and self.tf_ok > 100 and len(self.matched) > 10 and
                       len(self.maps) > 2 and map_hashes > 1 and
                       all(m['occupied'] > 0 and m['frame'] == 'map' for m in self.maps))
        endpoints = {}
        for topic in ('/scan', '/map', '/tf', '/tf_static', '/scan_matched_points2'):
            endpoints[topic] = [{'name': e.node_name, 'namespace': e.node_namespace,
                                 'type': e.topic_type, 'reliability': str(e.qos_profile.reliability)}
                                for e in self.get_publishers_info_by_topic(topic)]
        subscriptions = [{'name': e.node_name, 'type': e.topic_type}
                         for e in self.get_subscriptions_info_by_topic('/scan')]
        return {
            'source': 'LIVE_HARDWARE_ONLY; observer publishes no scans or commands',
            'test_scope': 'stationary sensor-origin SLAM; no vehicle motion',
            'requested_duration_s': duration, 'scan_pass': scan_ok,
            'slam_pass': slam_ok if require_slam else None,
            'scan_count': len(scans), 'continuous_scan_span_s': elapsed,
            'measured_scan_hz': (len(scans)-1)/elapsed if elapsed else 0,
            'receipt_gap_s': summary(gaps), 'stamp_gap_s': summary(stamp_gaps),
            'timestamp_age_s': summary([m['age_s'] for m in scans]),
            'valid_beams': summary([m['valid'] for m in scans]),
            'beam_count': summary([m['beams'] for m in scans]),
            'first_scan': scans[0] if scans else None, 'last_scan': scans[-1] if scans else None,
            'map_count': len(self.maps), 'distinct_map_contents': map_hashes,
            'maps': self.maps, 'matched_cloud_count': len(self.matched),
            'last_matched_cloud': self.matched[-1] if self.matched else None,
            'dynamic_tf_edges': dict(self.edges),
            'static_tf_parents': {k: sorted(v) for k, v in self.static_parents.items()},
            'scan_time_tf_lookup_successes': self.tf_ok,
            'tf_lookup_failures': dict(self.tf_failures), 'latest_map_to_laser_pose': self.latest_pose,
            'publishers': endpoints, 'scan_subscribers': subscriptions,
            'nodes': self.get_node_names(),
            'vehicle_extrinsics_verified': False, 'moving_slam_verified': False,
            'physical_direction_verified': False,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=40)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-slam', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        result = node.result(args.duration, args.require_slam)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        print(json.dumps({key: result[key] for key in (
            'scan_pass', 'slam_pass', 'scan_count', 'continuous_scan_span_s', 'measured_scan_hz',
            'map_count', 'distinct_map_contents', 'matched_cloud_count', 'scan_time_tf_lookup_successes')}))
        return 0 if result['scan_pass'] and (not args.require_slam or result['slam_pass']) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
