#!/usr/bin/env python3
"""Read-only G6 scan/SLAM performance measurement; no sensor or control output."""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud

from g6_probe import Probe, stamp_ns, summary


class PerformanceProbe(Probe):
    def __init__(self, processes):
        super().__init__()
        self.raw_clouds = []
        self.cpu_rows = []
        self.processes = processes
        self.cpu_previous = {}
        self.cpu_ticks = os.sysconf('SC_CLK_TCK')
        self.create_subscription(PointCloud, '/point_cloud', self.cloud, qos_profile_sensor_data)
        self.create_timer(1., self.cpu)
        self.cpu()

    def scan(self, msg):
        super().scan(msg)
        self.scans[-1].update({
            'finite_over_15m': sum(math.isfinite(v) and v > 15.00001 for v in msg.ranges),
            'zero_bins': sum(v == 0 for v in msg.ranges),
            'nan_bins': sum(math.isnan(v) for v in msg.ranges),
            'infinite_bins': sum(math.isinf(v) for v in msg.ranges),
            'below_min_positive_bins': sum(math.isfinite(v) and 0 < v < msg.range_min
                                           for v in msg.ranges),
        })

    def cloud(self, msg):
        distances = [math.hypot(p.x, p.y) for p in msg.points]
        self.raw_clouds.append({'wall': time.monotonic(), 'stamp_ns': stamp_ns(msg.header.stamp),
                                'points': len(msg.points),
                                'finite_over_15m': sum(d > 15.00001 for d in distances)})

    def cpu(self):
        wall = time.monotonic()
        for label, pid in self.processes.items():
            try:
                # /proc stat fields following the parenthesised command start at field 3.
                fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                ticks = int(fields[11]) + int(fields[12])
                starttime = int(fields[19])
                rss_mib = int(fields[21]) * os.sysconf('SC_PAGE_SIZE') / 1024**2
                previous = self.cpu_previous.get(label)
                if previous and previous[2] == starttime:
                    self.cpu_rows.append({'wall': wall, 'process': label, 'pid': pid,
                                          'cpu_percent_one_core': (ticks-previous[1]) /
                                          self.cpu_ticks / (wall-previous[0]) * 100,
                                          'rss_mib': rss_mib, 'alive': True})
                self.cpu_previous[label] = (wall, ticks, starttime)
            except (FileNotFoundError, ProcessLookupError):
                self.cpu_rows.append({'wall': wall, 'process': label, 'pid': pid, 'alive': False})

    def performance_result(self, duration, require_slam):
        result = self.result(duration, require_slam)
        scans = self.scans
        span = result['continuous_scan_span_s']
        gaps = [(b['stamp_ns']-a['stamp_ns'])/1e9 for a, b in zip(scans, scans[1:])]
        overlaps = [a['stamp_ns']/1e9 + (a['beams']-1)*a['time_increment'] -
                    b['stamp_ns']/1e9 for a, b in zip(scans, scans[1:])]
        total_bins = sum(m['beams'] for m in scans)
        total_valid = sum(m['valid'] for m in scans)
        clouds = self.raw_clouds
        cloud_span = clouds[-1]['wall']-clouds[0]['wall'] if len(clouds) > 1 else 0
        cpu_stats = {}
        for label in self.processes:
            rows = [r for r in self.cpu_rows if r['process'] == label]
            cpu_stats[label] = {
                'cpu_percent_one_core': summary([r['cpu_percent_one_core'] for r in rows if r['alive']]),
                'rss_mib': summary([r['rss_mib'] for r in rows if r['alive']]),
                'process_missing_samples': sum(not r['alive'] for r in rows),
            }
        over_range = sum(m['finite_over_15m'] for m in scans)
        performance_pass = bool(result['scan_pass'] and span >= 60 and not over_range and
                                scans and all(abs(m['range_max']-15) < 1e-5 for m in scans) and
                                all(not r['process_missing_samples'] for r in cpu_stats.values()))
        pose_yaws = [2*math.atan2(p['xyzw'][2], p['xyzw'][3]) for p in self.poses]
        relative_yaws = [0.]
        for previous, current in zip(pose_yaws, pose_yaws[1:]):
            relative_yaws.append(relative_yaws[-1] + math.atan2(
                math.sin(current-previous), math.cos(current-previous)))
        pose_stability = None
        if self.poses:
            origin = self.poses[0]['xyz']
            pose_stability = {
                'samples': len(self.poses), 'first_pose': self.poses[0],
                'max_xy_displacement_from_first_m': max(math.hypot(
                    p['xyz'][0]-origin[0], p['xyz'][1]-origin[1]) for p in self.poses),
                'yaw_change_from_first_degrees': summary([math.degrees(y) for y in relative_yaws]),
                'last_yaw_change_from_first_degrees': math.degrees(relative_yaws[-1]),
                'note': 'Estimated pose stability; does not prove physical immobility or absolute accuracy.',
            }
        result.update({
            'source': 'ROS_TOPIC_OBSERVATION; confirm live G6 identity in accompanying driver log',
            'performance_pass': performance_pass,
            'instantaneous_stamp_hz': summary([1/g for g in gaps if g > 0]),
            'instantaneous_receipt_hz': summary([1/(b['wall']-a['wall']) for a, b in zip(scans, scans[1:])
                                                if b['wall'] > a['wall']]),
            'total_scan_bins': total_bins, 'total_valid_scan_returns': total_valid,
            'valid_scan_return_ratio': total_valid/total_bins if total_bins else 0,
            'valid_ratio_per_scan': summary([m['valid']/m['beams'] for m in scans if m['beams']]),
            'estimated_scan_bins_per_second': sum(m['beams'] for m in scans[1:])/span if span else 0,
            'estimated_valid_returns_per_second': sum(m['valid'] for m in scans[1:])/span if span else 0,
            'positive_returns_over_15m': over_range,
            'zero_scan_bins': sum(m['zero_bins'] for m in scans),
            'nan_scan_bins': sum(m['nan_bins'] for m in scans),
            'infinite_scan_bins': sum(m['infinite_bins'] for m in scans),
            'range_bin_note': 'LaserScan angular bins include invalid zero slots; they are not '
                              'independent physical measurements. fixed_resolution remains false.',
            'filtered_sdk_point_cloud_count': len(clouds),
            'filtered_sdk_points_per_cloud': summary([m['points'] for m in clouds]),
            'estimated_filtered_sdk_points_per_second': sum(m['points'] for m in clouds[1:]) /
                                                       cloud_span if cloud_span else None,
            'filtered_sdk_points_over_15m': sum(m['finite_over_15m'] for m in clouds),
            'positive_scan_time_overlap_count': sum(v > 1e-6 for v in overlaps),
            'scan_time_overlap_s': summary(overlaps),
            'matching_messages_per_scan': len(self.matched)/len(scans) if scans else 0,
            'process_cpu': cpu_stats,
            'pose_stability': pose_stability,
        })
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=65)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--require-slam', action='store_true')
    parser.add_argument('--pid', action='append', default=[], help='process_label:PID')
    args = parser.parse_args()
    processes = {label: int(pid) for label, pid in (s.rsplit(':', 1) for s in args.pid)}
    rclpy.init()
    node = PerformanceProbe(processes)
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        result = node.performance_result(args.duration, args.require_slam)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        args.output.with_suffix('.samples.json').write_text(json.dumps({
            'scans': node.scans, 'filtered_sdk_clouds': node.raw_clouds,
            'matched_clouds': node.matched, 'cpu': node.cpu_rows,
            'poses': node.poses,
        }, indent=2, allow_nan=False)+'\n')
        print(json.dumps({key: result[key] for key in (
            'performance_pass', 'slam_pass', 'continuous_scan_span_s', 'scan_count',
            'measured_scan_hz', 'beam_count', 'valid_scan_return_ratio',
            'estimated_scan_bins_per_second', 'map_count', 'matched_cloud_count')}))
        return 0 if result['performance_pass'] and (not args.require_slam or result['slam_pass']) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
