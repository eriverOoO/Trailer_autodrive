"""Bounded real sensor startup test; no motor nodes, no simulated measurements."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

ROOT=Path(__file__).resolve().parent


def main():
    with (ROOT/'launch.log').open('w') as log, (ROOT/'status.jsonl').open('w') as output:
        proc=subprocess.Popen(['ros2','launch','tractor_trailer_rviz_2d','relative_yaw.launch.py','rviz:=false'],
                              stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=Node('verify_real_relative_yaw')
        statuses=[];markers=[]
        def on_status(m):
            item=json.loads(m.data);statuses.append(item);output.write(m.data+'\n');output.flush()
        node.create_subscription(String,'/vehicle_viz_2d/status',on_status,10)
        node.create_subscription(MarkerArray,'/vehicle_viz_2d/model',lambda m:markers.append(m),10)
        try:
            start=time.monotonic();first_ready=None;last_reason=None
            while time.monotonic()-start<45:
                if proc.poll() is not None:raise RuntimeError('Launch exited; inspect launch.log')
                rclpy.spin_once(node,timeout_sec=.05)
                if statuses:
                    s=statuses[-1]
                    if s['reason']!=last_reason:
                        print(s['reason'],flush=True);last_reason=s['reason']
                    if s.get('ready') and first_ready is None:first_ready=time.monotonic()
                    if first_ready is not None and time.monotonic()-first_ready>3:break
            ready=[s for s in statuses if s.get('ready')]
            subscriptions={topic:[i.node_name for i in node.get_subscriptions_info_by_topic(topic)]
                           for topic in ('/tractor/imu/data','/trailer/imu/data')}
            assert not any('cartographer' in name for names in subscriptions.values() for name in names)
            result={'real_hardware':True,'synthetic_samples':False,'status_count':len(statuses),
                    'ready_count':len(ready),'ready_reached':bool(ready),
                    'last_status':statuses[-1] if statuses else None,
                    'imu_subscribers':subscriptions,'marker_batches':len(markers),
                    'physical_turn_accuracy_verified':False}
            if ready:
                result['first_ready']=ready[0]
                result['ready_after_process_start_s']=first_ready-start
                result['status']='PASS'
            else:result['status']='NOT_READY'
            (ROOT/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps({k:result[k] for k in ('status','ready_reached','status_count','ready_count','marker_batches','imu_subscribers')},indent=2),flush=True)
        finally:
            node.destroy_node();rclpy.shutdown()
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2)


if __name__=='__main__':main()
