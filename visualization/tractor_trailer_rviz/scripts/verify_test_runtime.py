#!/usr/bin/env python3
"""Explicit synthetic integration test, isolated domain, no serial/control processes."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('source isolated overlay and set ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[]);node=rclpy.create_node('vehicle_viz_test_observer')
    rows=[];phase='BOOT';resources=set();logs=[];procs=[];shots=set();phase_started=time.monotonic()
    def phase_callback(m):
        nonlocal phase,phase_started
        if m.data!=phase:phase_started=time.monotonic()
        phase=m.data
    def status(m):rows.append(dict(phase=phase,**json.loads(m.data)))
    def models(m):
        resources.update(x.mesh_resource for x in m.markers if x.type==10 and x.action==0)
    node.create_subscription(String,'/vehicle_viz_test/status',status,100)
    node.create_subscription(String,'/vehicle_viz_test/phase',phase_callback,100)
    node.create_subscription(MarkerArray,'/vehicle_viz_test/model',models,10)
    try:
        log=(args.output/'launch.log').open('x');logs.append(log)
        procs.append(subprocess.Popen(['ros2','launch','tractor_trailer_rviz','test_preview.launch.py',
                                      'output_directory:='+str(args.output/'node')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        started=time.monotonic()
        while time.monotonic()-started<31.:
            rclpy.spin_once(node,timeout_sec=.05)
            if phase in ('A_TRACTOR_ONLY','B_TRAILER_ONLY') and phase not in shots and time.monotonic()-phase_started>1.4:
                try:
                    lines=subprocess.check_output(['xwininfo','-root','-tree'],text=True).splitlines()
                    windows=[s for s in lines if 'test_preview.rviz' in s and 'RViz' in s]
                    if windows:
                        subprocess.run(['import','-window',windows[0].strip().split()[0],str(args.output/(phase+'.png'))],capture_output=True,timeout=5)
                        shots.add(phase)
                except (OSError,subprocess.SubprocessError):pass
        checks={}
        def valid(name):return [r for r in rows if r['phase']==name and r['valid']]
        a=valid('A_TRACTOR_ONLY')
        checks['tractor_only']=any(abs(r['values']['tractor_delta'])>.25 and abs(r['values']['trailer_delta'])<.001 for r in a)
        b=valid('B_TRAILER_ONLY')
        checks['trailer_only']=any(abs(r['values']['trailer_delta'])>.2 and abs(r['values']['tractor_delta'])<.001 for r in b)
        c=valid('C_DIFFERENT')
        checks['different_rotations']=any(r['values']['tractor_delta']>.3 and r['values']['trailer_delta']>.1 and abs(r['values']['articulation']-(r['values']['tractor_delta']-r['values']['trailer_delta']))<1e-7 for r in c)
        d=valid('D_SAME')
        checks['same_angle_zero_articulation']=bool(d) and max(abs(r['values']['articulation']) for r in d)<1e-6
        e=[r for r in rows if r['phase']=='E_TRACTOR_STOP']
        checks['separate_stale']=any(r['imu']=={'tractor':'STALE','trailer':'LIVE'} and not r['valid'] and r['values'] is None for r in e)
        f=[r for r in rows if r['phase']=='F_INVALID_TRAILER']
        checks['separate_invalid']=any(r['imu']['trailer']=='INVALID' and not r['valid'] for r in f)
        checks['alignment_lost_after_gap']=any('ALIGNMENT LOST' in r['reason'] for r in rows if r['phase']=='RECOVER_REQUIRES_REALIGN')
        g=valid('G_SLAM_YAW_ONLY')
        checks['slam_yaw_does_not_rotate_bodies']=bool(g) and max(abs(r['values']['tractor_delta'])+abs(r['values']['trailer_delta']) for r in g)<1e-6
        all_valid=[r for r in rows if r['valid']]
        checks['hitch_position_invariant']=bool(all_valid) and all(math.dist(r['poses']['hitch'],r['poses']['trailer'])<1e-9 for r in all_valid)
        checks['test_label']=bool(rows) and all(r['mode']=='TEST/SIMULATED' for r in rows)
        checks['all_five_original_mesh_resources']=len(resources)==5
        topics={name for name,_ in node.get_topic_names_and_types()}
        checks['no_real_sensor_or_control_topics']=not (topics & {'/cmd_vel','/scan','/map','/tractor/imu/data','/trailer/imu/data','/vehicle_viz/tractor/imu/data','/vehicle_viz/trailer/imu/data'})
        result=dict(kind='SYNTHETIC SOFTWARE TEST ONLY; no hardware validation',checks=checks,rows=len(rows),valid_rows=len(all_valid),resources=sorted(resources),screenshots=sorted(shots))
        (args.output/'summary.json').write_text(json.dumps(result,indent=2))
        with (args.output/'observed_status.jsonl').open('x') as f:
            for row in rows:f.write(json.dumps(row)+'\n')
        print(json.dumps(result,indent=2))
        return 0 if all(checks.values()) else 1
    finally:
        for proc in procs:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        for proc in procs:
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=3)
        for log in logs:log.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
