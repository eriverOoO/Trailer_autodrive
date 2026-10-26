#!/usr/bin/env python3
"""Read-only live scans. Physical target positions must be confirmed separately."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import rclpy
from g6_probe import Probe

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=10)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
rclpy.init()

class DirectionProbe(Probe):
    def __init__(self):
        super().__init__()
        self.bins = defaultdict(list)
        self.sectors = defaultdict(list)
        self.increments = []
    def scan(self, msg):
        super().scan(msg)
        self.increments.append(math.degrees(msg.angle_increment))
        for i, value in enumerate(msg.ranges):
            if not math.isfinite(value) or not msg.range_min <= value <= msg.range_max:
                continue
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)
            key = int(math.floor(angle + .5))
            key = (key + 180) % 360 - 180
            self.bins[key].append(value)
            for label, center in [('front_ros_x',0),('left_ros_y',90),('rear_ros_minus_x',180),('right_ros_minus_y',-90)]:
                delta = (angle - center + 180) % 360 - 180
                if abs(delta) <= 5:
                    self.sectors[label].append(value)

node = DirectionProbe()
try:
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    result = node.result(args.seconds, False)
    def describe(values):
        return {'valid_samples':len(values),'min_m':min(values),'median_m':statistics.median(values),'max_m':max(values),
                'fraction_in_0p8_to_1p2m':sum(.8 <= v <= 1.2 for v in values)/len(values)} if values else {'valid_samples':0}
    result['raw_laser_sectors_plus_minus_5_deg'] = {k:describe(v) for k,v in node.sectors.items()}
    result['raw_laser_1_degree_bins'] = {str(k):describe(v) for k,v in sorted(node.bins.items())}
    result['angle_increment_deg'] = {'min':min(node.increments),'max':max(node.increments)} if node.increments else None
    result['physical_target_position_confirmed'] = False
    result['direction_interpretation'] = 'ROS axes only; background objects can also be near 1 m. Do not infer physical cable orientation without user placement confirmation.'
    if node.poses:
        first = node.poses[0]['xyz']
        yaws = [math.degrees(2*math.atan2(p['xyzw'][2],p['xyzw'][3])) for p in node.poses]
        result['stationary_pose_observation']={'yaw_deg_min':min(yaws),'yaw_deg_max':max(yaws),'max_translation_from_first_m':max(math.dist(p['xyz'][:2],first[:2]) for p in node.poses)}
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ['scan_count','measured_scan_hz','first_scan','map_count','matched_cloud_count','scan_time_tf_lookup_successes','tf_lookup_failures','dynamic_tf_edges','static_tf_parents','raw_laser_sectors_plus_minus_5_deg','angle_increment_deg','latest_map_to_laser_pose','physical_target_position_confirmed']},indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
