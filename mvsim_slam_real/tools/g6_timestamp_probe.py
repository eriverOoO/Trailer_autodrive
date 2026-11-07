#!/usr/bin/env python3
"""Observe complete scans and reproduce Cartographer's time conversion, read-only."""
import argparse
from array import array
import gzip
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from g6_performance_probe import PerformanceProbe
from g6_probe import stamp_ns, summary


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def cartographer_ticks(stamp, increment, indices):
    """100ns ticks, float32 offsets and truncating FromSeconds, subdivision=1."""
    if not indices:
        return []
    duration = f32(indices[-1] * increment)
    end = stamp // 100 + int(duration * 1e7)
    return [end + int(f32(f32(i * increment)-duration)*1e7) for i in indices]


class TimestampProbe(PerformanceProbe):
    def __init__(self, processes):
        self.previous_carto_end = None
        self.complete_scans = []
        self.source_cloud_samples = []
        super().__init__(processes)

    def scan(self, msg):
        super().scan(msg)
        valid = [i for i, v in enumerate(msg.ranges) if msg.range_min <= v <= msg.range_max]
        ticks = cartographer_ticks(stamp_ns(msg.header.stamp), msg.time_increment, valid)
        row = self.scans[-1]
        previous_end = self.previous_carto_end
        discarded = sum(t < previous_end for t in ticks) if previous_end is not None else 0
        rejected = bool(ticks and previous_end is not None and ticks[-1] <= previous_end)
        row.update({
            'first_valid_ray_index': valid[0] if valid else None,
            'last_valid_ray_index': valid[-1] if valid else None,
            'cartographer_first_valid_tick': ticks[0] if ticks else None,
            'cartographer_last_valid_tick': ticks[-1] if ticks else None,
            'cartographer_previous_valid_end_tick': previous_end,
            'predicted_discarded_earlier_points': discarded,
            'predicted_subdivision_rejection': rejected,
        })
        if ticks and not rejected:
            self.previous_carto_end = ticks[-1]
        self.complete_scans.append(array('f', msg.ranges))

    def cloud(self, msg):
        super().cloud(msg)
        if len(self.source_cloud_samples) >= 32:
            return
        offsets = next((list(c.values) for c in msg.channels if c.name == 'stamps'), [])
        self.source_cloud_samples.append({
            'stamp_ns': stamp_ns(msg.header.stamp),
            'angles': [math.atan2(p.y, p.x) for p in msg.points],
            'source_ordinal_time_offsets': offsets,
        })

    def timing_result(self, duration):
        result = self.performance_result(duration, True)
        pairs = []
        by_stamp = {s['stamp_ns']: s for s in self.scans}
        for cloud in self.source_cloud_samples:
            scan = by_stamp.get(cloud['stamp_ns'])
            if scan is None:
                continue
            bins = [math.ceil((a-scan['angle_min'])/scan['angle_increment']) for a in cloud['angles']]
            errors = [b*scan['time_increment']-t for b, t in
                      zip(bins, cloud['source_ordinal_time_offsets']) if 0 <= b < scan['beams']]
            pairs.append({
                'stamp_ns': scan['stamp_ns'], 'source_points': len(bins),
                'negative_bin_index_steps_in_acquisition_order': sum(b < a for a, b in zip(bins, bins[1:])),
                'positive_bin_index_steps_in_acquisition_order': sum(b > a for a, b in zip(bins, bins[1:])),
                'bin_model_minus_source_ordinal_model_s': summary(errors),
            })
        result.update({
            'source': 'LIVE_G6_READ_ONLY_TIMESTAMP_DIAGNOSTICS; no corrections or sensor output',
            'behavior_change_applied': False,
            'predicted_crop_warning_count': sum(s['predicted_discarded_earlier_points'] > 0 for s in self.scans),
            'predicted_crop_point_count': sum(s['predicted_discarded_earlier_points'] for s in self.scans),
            'predicted_subdivision_rejections': sum(s['predicted_subdivision_rejection'] for s in self.scans),
            'source_order_vs_angular_bins': pairs,
            'source_order_note': 'PointCloud stamps channel uses SDK ordinal*i time model, '
                                 'not independent per-ray hardware timestamps.',
            'hardware_timestamp_readback_claimed': False,
            'full_scan_arrays_recorded': True,
        })
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=125)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--pid', action='append', default=[], help='process_label:PID')
    args = parser.parse_args()
    processes = {label: int(pid) for label, pid in (s.rsplit(':', 1) for s in args.pid)}
    rclpy.init()
    node = TimestampProbe(processes)
    try:
        end = time.monotonic()+args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        result = node.timing_result(args.duration)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        args.output.with_suffix('.samples.json').write_text(json.dumps({
            'scans': node.scans, 'filtered_sdk_clouds': node.raw_clouds,
            'source_cloud_samples': node.source_cloud_samples,
            'matched_clouds': node.matched, 'cpu': node.cpu_rows, 'poses': node.poses,
        }, indent=2, allow_nan=False)+'\n')
        with gzip.open(args.output.with_suffix('.ranges.jsonl.gz'), 'wt') as stream:
            for index, (metadata, ranges) in enumerate(zip(node.scans, node.complete_scans)):
                stream.write(json.dumps({
                    'index': index, 'metadata': metadata,
                    'ranges': [v if math.isfinite(v) else None for v in ranges],
                }, allow_nan=False)+'\n')
        print(json.dumps({k: result[k] for k in (
            'performance_pass', 'slam_pass', 'continuous_scan_span_s', 'scan_count',
            'measured_scan_hz', 'positive_scan_time_overlap_count',
            'predicted_crop_warning_count', 'predicted_crop_point_count', 'map_count')}))
        return 0 if result['performance_pass'] and result['slam_pass'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
