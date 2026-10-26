"""Bounded live verification, both real IMUs; isolated ROS domain, no actuation."""
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parent


def main():
    with (ROOT/'dual_imu_live.log').open('w') as log:
        process = subprocess.Popen(['ros2','launch','mvsim_slam_real_bringup','dual_imu.launch.py'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node('verify_real_dual_imu')
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        samples = {role: [] for role in ('tractor','trailer')}
        def receive(role, msg):
            samples[role].append((time.monotonic(), msg))
        for role in samples:
            node.create_subscription(Imu, f'/{role}/imu/data',
                                     lambda msg, role=role: receive(role, msg), qos_profile_sensor_data)
        try:
            end = time.monotonic()+8
            while time.monotonic()<end:
                if process.poll() is not None:
                    raise RuntimeError('Dual IMU launch exited early; see log')
                rclpy.spin_once(node, timeout_sec=.02)
            report = {'source':'REAL_HARDWARE', 'synthetic_samples':False,
                      'configuration_writes':0, 'physical_axis_alignment_verified':False,
                      'sensors':{}}
            for role, records in samples.items():
                assert len(records)>300, (role,len(records))
                stamps=[m.header.stamp.sec*10**9+m.header.stamp.nanosec for _,m in records]
                norms=[math.sqrt(sum(getattr(m.orientation,k)**2 for k in ('x','y','z','w'))) for _,m in records]
                accels=[math.sqrt(sum(getattr(m.linear_acceleration,k)**2 for k in ('x','y','z'))) for _,m in records]
                hz=(len(records)-1)/(records[-1][0]-records[0][0])
                assert 85<hz<115, (role,hz)
                assert all(b>a for a,b in zip(stamps,stamps[1:]))
                assert all(m.header.frame_id==role+'_imu_link' and m.linear_acceleration_covariance[0]>0 for _,m in records)
                assert all(abs(n-1)<1e-6 for n in norms)
                parent='base_footprint' if role=='tractor' else 'trailer_link'
                tf=buffer.lookup_transform(parent,role+'_imu_link',Time()).transform
                q=tf.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                if role=='trailer':assert abs(abs(yaw)-math.pi)<1e-9
                else:assert abs(yaw)<1e-9
                report['sensors'][role]={'messages':len(records),'rate_hz':hz,
                    'host_stamps_increasing':True,'quaternion_norm_range':[min(norms),max(norms)],
                    'acceleration_norm_m_s2_range':[min(accels),max(accels)],
                    'mount_xyz_m':[tf.translation.x,tf.translation.y,tf.translation.z],
                    'mount_yaw_deg':math.degrees(yaw)}
            report['status']='PASS'
            (ROOT/'dual_imu_live_summary.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2))
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait(timeout=2)


if __name__=='__main__':
    main()
