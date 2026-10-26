#!/usr/bin/env python3
"""Join observed Cartographer warnings with read-only per-scan predictions."""
import argparse
import csv
import json
from pathlib import Path
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.samples.read_text())
    summary = json.loads(args.summary.read_text())
    scans = data['scans']
    log = args.log.read_text()
    receipts = [s['stamp_ns']/1e9+s['age_s'] for s in scans]
    warnings = []
    for line in log.splitlines():
        match = re.search(r'\[(\d+\.\d+)\].*Dropped (\d+) earlier points', line)
        if match:
            warnings.append({'time_s': float(match[1]), 'points': int(match[2])})
    observed = [w for w in warnings if receipts[0] <= w['time_s'] <= receipts[-1]]
    by_index = {}
    for warning in observed:
        index = min(range(len(scans)), key=lambda i: abs(receipts[i]-warning['time_s']))
        assert abs(receipts[index]-warning['time_s']) < .015
        by_index.setdefault(index, []).append(warning)
    rows = []
    for i, scan in enumerate(scans):
        predicted = scan['predicted_discarded_earlier_points']
        actual = sum(w['points'] for w in by_index.get(i, []))
        if not (predicted or actual):
            continue
        previous = scans[i-1] if i else None
        row = {
            'scan_index_zero_based': i, 'first_ns': scan['stamp_ns'],
            'previous_last_bin_ns': previous['stamp_ns']+round(
                (previous['beams']-1)*previous['time_increment']*1e9) if previous else None,
            'previous_last_valid_ns': scan['cartographer_previous_valid_end_tick']*100
            if scan['cartographer_previous_valid_end_tick'] is not None else None,
            'first_valid_ns': scan['cartographer_first_valid_tick']*100,
            'last_valid_ns': scan['cartographer_last_valid_tick']*100,
            'first_valid_index': scan['first_valid_ray_index'],
            'last_valid_index': scan['last_valid_ray_index'],
            'predicted_earlier_points': predicted, 'observed_earlier_points': actual,
            'warning_log_time_s': by_index[i][0]['time_s'] if by_index.get(i) else None,
            'exact_match': predicted == actual,
        }
        rows.append(row)
    cpu = {}
    for label in summary['process_cpu']:
        values = [r['cpu_percent_one_core'] for r in data['cpu'] if r['process'] == label and r['alive']]
        cpu[label] = sum(values)/len(values) if values else None
    geometry = [a['stamp_ns']+round((a['beams']-1)*a['time_increment']*1e9)-b['stamp_ns']
                for a, b in zip(scans, scans[1:])]
    source = summary['source_order_vs_angular_bins']
    result = {
        'source_behavior_changed': False,
        'all_observed_warnings_reproduced_exactly': all(r['exact_match'] for r in rows),
        'predicted_warning_count': sum(s['predicted_discarded_earlier_points'] > 0 for s in scans),
        'observed_warning_count': len(observed),
        'predicted_point_count': sum(s['predicted_discarded_earlier_points'] for s in scans),
        'observed_point_count': sum(w['points'] for w in observed),
        'strict_positive_bin_time_overlap_count': sum(x > 0 for x in geometry),
        'bin_time_overlap_above_1us_count': sum(x > 1000 for x in geometry),
        'maximum_bin_time_overlap_ns': max(geometry),
        'geometry_without_valid_point_warning_count': sum(x > 0 for x in geometry)-len(observed),
        'header_stamps_not_millisecond_aligned': sum(s['stamp_ns'] % 1000000 != 0 for s in scans),
        'cpu_mean_percent_one_core': cpu,
        'source_clouds_compared': len(source),
        'negative_angle_bin_steps_in_source_order': sum(s['negative_bin_index_steps_in_acquisition_order'] for s in source),
        'positive_angle_bin_steps_in_source_order': sum(s['positive_bin_index_steps_in_acquisition_order'] for s in source),
        'bin_model_vs_source_model_min_s': min((s['bin_model_minus_source_ordinal_model_s']['min'] for s in source), default=None),
        'bin_model_vs_source_model_max_s': max((s['bin_model_minus_source_ordinal_model_s']['max'] for s in source), default=None),
        'checksum_entire_run': log.count('Checksum error'),
        'driver_scan_failures': log.count('Failed to get scan'),
        'driver_connections': log.count('Lidar successfully connected'),
        'driver_reconnect_log_entries': len(re.findall(r'(?i)reconnect(?:ing|ion)|attempt.*reconnect', log)),
        'warnings_entire_run': len(warnings), 'points_entire_run': sum(w['points'] for w in warnings),
        'warnings_before_observation': sum(w['time_s'] < receipts[0] for w in warnings),
        'warnings_after_observation': sum(w['time_s'] > receipts[-1] for w in warnings),
        'warning_rate_hz': len(observed)/summary['continuous_scan_span_s'],
        'earlier_points_per_second': sum(w['points'] for w in observed)/summary['continuous_scan_span_s'],
        'clean_process_exits': log.count('process has finished cleanly'),
        'rows': rows,
    }
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    with args.output.with_suffix('.csv').open('w') as stream:
        fields = list(rows[0]) if rows else ['scan_index_zero_based', 'predicted_earlier_points',
                                            'observed_earlier_points', 'exact_match']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({k: v for k, v in result.items() if k != 'rows'}, indent=2))
    assert result['all_observed_warnings_reproduced_exactly']
    assert result['predicted_point_count'] == result['observed_point_count']


if __name__ == '__main__':
    main()
