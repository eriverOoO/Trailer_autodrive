#!/usr/bin/env python3
"""Summarize field logs; repeated physical marks are required for repeatability."""
import argparse
import json
import math
from collections import defaultdict
import numpy as np


def summarize(events):
    samples = [e for e in events if e.get('event') == 'sample']
    groups = defaultdict(list)
    for e in events:
        if e.get('event') == 'reference':
            groups[e['label']].append(e['pose'])
    references = {}
    for label, poses in groups.items():
        p = np.asarray(poses)
        distances = np.linalg.norm(p[:, None, :2]-p[None, :, :2], axis=-1)
        angles = p[:, None, 2]-p[None, :, 2]
        references[label] = dict(visits=len(p), max_separation_m=float(distances.max()),
                                 max_yaw_difference_deg=float(np.rad2deg(np.abs(np.arctan2(np.sin(angles), np.cos(angles)))).max()),
                                 repeatability_measured=len(p) > 1)
    return dict(samples=len(samples), stale_or_unavailable=sum(not e.get('inputs_fresh') for e in samples),
                accepted_gap_samples=sum(bool(e.get('perception', {}).get('ready')) for e in samples),
                largest_pose_step_m=max((e.get('pose_step_m', 0.) for e in samples), default=0.),
                physical_references=references,
                interpretation='No ground-truth accuracy or parking feasibility claim. Compare marked-point visits and inspect bag/RViz.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('log'); args = parser.parse_args()
    with open(args.log) as stream:
        result = summarize([json.loads(line) for line in stream if line.strip()])
    print(json.dumps(result, indent=2, ensure_ascii=False))
