#!/usr/bin/env python3
"""Passive checks of scan start/end TF availability, with explicit warmup."""
import json,time,math
from pathlib import Path
from collections import Counter
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener,TransformException

rclpy.init();n=Node('g6_tf_timing_diagnostics');b=Buffer();listener=TransformListener(b,n);begin=time.monotonic();queue=[];rows=[]
def check(stamp):
 try:b.lookup_transform('map','laser_frame',Time(nanoseconds=stamp));return True,None
 except TransformException as e:return False,str(e).split('\n')[0]
def record(m):
 if time.monotonic()-begin<2:return
 start=m.header.stamp.sec*10**9+m.header.stamp.nanosec;end=start+int((len(m.ranges)-1)*m.time_increment*1e9)
 row={'receipt_elapsed_s':time.monotonic()-begin,'start_ns':start,'end_ns':end,'duration_s':(end-start)/1e9,'time_increment':m.time_increment,'beams':len(m.ranges),'scan_time':m.scan_time,'header_age_at_receipt_s':(n.get_clock().now().nanoseconds-start)/1e9,'start_now':check(start)[0],'end_now':check(end)[0]}
 try:
  t=b.lookup_transform('map','laser_frame',Time());latest=t.header.stamp.sec*10**9+t.header.stamp.nanosec;row['end_minus_latest_common_tf_ms']=(end-latest)/1e6
 except TransformException:row['end_minus_latest_common_tf_ms']=None
 queue.append((time.monotonic(),row));rows.append(row)
def timer():
 now=time.monotonic()
 for received,row in queue:
  for delay in [.03,.1]:
   key=f'end_after_{int(delay*1000)}ms'
   if now-received>=delay and key not in row:row[key]=check(row['end_ns'])[0]
 queue[:]=[(t,row) for t,row in queue if now-t<.2]
n.create_subscription(LaserScan,'/scan',record,qos_profile_sensor_data);n.create_timer(.01,timer)
try:
 while time.monotonic()-begin<25:rclpy.spin_once(n,timeout_sec=.02)
 def stat(vals):
  a=np.asarray(vals,dtype=float);return {'min':float(a.min()),'median':float(np.median(a)),'max':float(a.max())} if len(a) else None
 summary={'warmup_s':2,'scans':len(rows),'checks':{key:{'checked':sum(key in r for r in rows),'failed':sum(key in r and not r[key] for r in rows)} for key in ['start_now','end_now','end_after_30ms','end_after_100ms']},'end_minus_latest_common_tf_ms':stat([r['end_minus_latest_common_tf_ms'] for r in rows if r['end_minus_latest_common_tf_ms'] is not None]),'scan_duration_s':stat([r['duration_s'] for r in rows]),'raw_scan_frame':'laser_frame','physical_target_confirmed':False}
 out=Path(__file__).parent;(out/'timing_rows.json').write_text(json.dumps(rows,indent=2)+'\n');(out/'timing_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
finally:n.destroy_node();rclpy.shutdown()
