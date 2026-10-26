#!/usr/bin/env python3
"""Offline scan boundary analysis. Does not change or publish measurements."""
import argparse
import csv
import json
from pathlib import Path
import re
import statistics


def analyze(samples, log):
    scans = samples['scans']
    warnings = []
    for line in log.splitlines():
        match = re.search(r'\[(\d+\.\d+)\].*Dropped (\d+) earlier points', line)
        if match:
            warnings.append({'log_time_s': float(match[1]), 'points': int(match[2])})
    boundaries = []
    for i, (previous, current) in enumerate(zip(scans, scans[1:]), 1):
        # Work relative to the integer epoch to avoid loss of sub-microsecond precision.
        last_ns = previous['stamp_ns'] + round((previous['beams']-1) *
                                               previous['time_increment'] * 1e9)
        overlap_ns = last_ns-current['stamp_ns']
        if overlap_ns <= 0:
            continue
        receipt_s = current['stamp_ns']/1e9+current['age_s']
        near = [w for w in warnings if abs(w['log_time_s']-receipt_s) < .015]
        boundaries.append({
            'previous_scan_index_zero_based': i-1, 'next_scan_index_zero_based': i,
            'previous_first_ns': previous['stamp_ns'], 'previous_last_bin_ns': last_ns,
            'next_first_ns': current['stamp_ns'], 'overlap_ns': overlap_ns,
            'previous_time_increment_s': previous['time_increment'],
            'next_time_increment_s': current['time_increment'],
            'next_scan_time_s': current['scan_time'],
            'next_start_gap_s': (current['stamp_ns']-previous['stamp_ns'])/1e9,
            'warning_count': len(near), 'warning_points': sum(w['points'] for w in near),
        })
    span = scans[-1]['wall']-scans[0]['wall']
    first_receipt = scans[0]['stamp_ns']/1e9+scans[0]['age_s']
    last_receipt = scans[-1]['stamp_ns']/1e9+scans[-1]['age_s']
    inside = [w for w in warnings if first_receipt <= w['log_time_s'] <= last_receipt]
    nominal_overlaps = [((a['beams']-1)/18000 - (b['stamp_ns']-a['stamp_ns'])/1e9)
                        for a, b in zip(scans, scans[1:])]
    start_gaps = [(b['stamp_ns']-a['stamp_ns'])/1e9 for a, b in zip(scans, scans[1:])]
    period_errors = [b['scan_time']-gap for b, gap in zip(scans[1:], start_gaps)]
    result = {
        'scan_count': len(scans), 'span_s': span,
        'header_stamps_strictly_increasing': all(g > 0 for g in start_gaps),
        'header_stamps_not_millisecond_aligned': sum(s['stamp_ns'] % 1000000 != 0 for s in scans),
        'positive_bin_time_overlaps': len(boundaries),
        'max_bin_time_overlap_ns': max([b['overlap_ns'] for b in boundaries], default=0),
        'warnings_entire_log': len(warnings), 'points_entire_log': sum(w['points'] for w in warnings),
        'warnings_during_observation': len(inside), 'points_during_observation': sum(w['points'] for w in inside),
        'warnings_per_second': len(inside)/span,
        'dropped_points_per_second': sum(w['points'] for w in inside)/span,
        'matched_warnings_at_positive_boundaries': sum(b['warning_count'] for b in boundaries),
        'scan_time_minus_current_start_period_s': {
            'min': min(period_errors), 'median': statistics.median(period_errors), 'max': max(period_errors)},
        'counterfactual_nominal_18k_increment': {
            'positive_overlaps': sum(o > 0 for o in nominal_overlaps),
            'max_overlap_s': max(nominal_overlaps),
            'adopted': False,
            'reason': 'Nominal sample rate alone does not establish per-ray measurement times.'},
        'boundaries': boundaries,
        'limitation': 'Old metadata lacks valid ray indices/ranges; exact valid-point cropping needs full scans.',
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', required=True, type=Path)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = analyze(json.loads(args.samples.read_text()), args.log.read_text())
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    if result['boundaries']:
        with args.output.with_suffix('.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(result['boundaries'][0]))
            writer.writeheader()
            writer.writerows(result['boundaries'])
    print(json.dumps({k: v for k, v in result.items() if k != 'boundaries'}))


if __name__ == '__main__':
    main()
