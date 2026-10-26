#!/usr/bin/env python3
"""Measure actual diagnostic IMU messages during a user-controlled motion trial."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from imu_common import quaternion_euler


def unwrap(values):
    result = [values[0]]
    for a,b in zip(values, values[1:]):
        result.append(result[-1]+(b-a+180)%360-180)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--test', choices=['stationary','roll','pitch','yaw','return'], required=True)
    p.add_argument('--seconds', type=float, default=15.)
    p.add_argument('--namespace', default='/ebimu_diag/tractor')
    p.add_argument('--frame', default='ebimu_diag_tractor_sensor')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not args.namespace.startswith('/ebimu_diag/') or not 1 <= args.seconds <= 60:
        p.error('diagnostic namespace and 1..60 seconds required')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    node = Node('ebimu_motion_measurement')
    rows = []
    rejected = 0
    def receive(m):
        nonlocal rejected
        q = [m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w]
        norm = math.sqrt(sum(x*x for x in q))
        age = (node.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e9
        if (m.header.frame_id != args.frame or m.orientation_covariance[0] == -1
                or not math.isfinite(norm) or abs(norm-1.) > .01 or not -.05 <= age <= .5):
            rejected += 1
            return
        rpy = list(map(math.degrees, quaternion_euler([x/norm for x in q])))
        g = [m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z] if m.angular_velocity_covariance[0] != -1 else [None]*3
        rows.append([time.monotonic(),*rpy,norm,*g,age*1000])
    sub = node.create_subscription(Imu, args.namespace+'/imu/data', receive, qos_profile_sensor_data)
    start = time.monotonic()
    print('Recording REAL topic for',args.seconds,'seconds; trial:',args.test,flush=True)
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = dict(test=args.test, samples=len(rows), rejected=rejected,
                  status='MEASURED_AWAITING_PHYSICAL_RVIZ_OBSERVATION' if len(rows)>1 else 'NO_VALID_DATA',
                  axis_sign_verified=False, rviz_direction_verified=False,
                  absolute_accuracy='unmeasured; external reference required')
    if len(rows)>1:
        rpy = [unwrap([r[i] for r in rows]) for i in (1,2,3)]
        result.update(rate_hz=(len(rows)-1)/(rows[-1][0]-rows[0][0]),
                      rpy_span_deg=[max(v)-min(v) for v in rpy],
                      rpy_end_minus_start_deg=[v[-1]-v[0] for v in rpy],
                      rpy_stddev_deg=[statistics.pstdev(v) for v in rpy],
                      gyro_stddev_rad_s=[statistics.pstdev([r[i] for r in rows]) if rows[0][i] is not None else None for i in (5,6,7)],
                      quaternion_norm_range=[min(r[4] for r in rows),max(r[4] for r in rows)],
                      host_stamp_age_ms_range=[min(r[8] for r in rows),max(r[8] for r in rows)])
    with (args.output/'samples.csv').open('x') as f:
        w=csv.writer(f)
        w.writerow(['receipt_monotonic_s','roll_deg','pitch_deg','yaw_deg','quaternion_norm',
                    'gx_rad_s','gy_rad_s','gz_rad_s','host_stamp_age_ms'])
        w.writerows(rows)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return 0 if len(rows)>1 else 2


if __name__ == '__main__':
    raise SystemExit(main())
