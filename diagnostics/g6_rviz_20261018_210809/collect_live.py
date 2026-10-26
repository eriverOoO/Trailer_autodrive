#!/usr/bin/env python3
"""Passive hardware observations only; publishes no sensor, TF or vehicle commands."""
import argparse
from collections import Counter, defaultdict, deque
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
from cartographer_ros_msgs.msg import SubmapList


def ns(s): return s.sec*10**9+s.nanosec

def stats(v):
    a=np.asarray(v,dtype=float)
    return {'min':float(a.min()),'median':float(np.median(a)),'mean':float(a.mean()),'max':float(a.max())} if len(a) else None

def rpy(q):
    return [math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))), math.degrees(math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))), math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))]

class Collector(Node):
    def __init__(self,out):
        super().__init__('g6_rviz_diagnostics_observer')
        self.out=out; self.start=time.monotonic(); self.scans=[]; self.maps=[]; self.last_map=None
        self.last_scans=deque(maxlen=20); self.bin_values=defaultdict(list);self.clouds=[];self.submaps=[]
        self.edges=Counter();self.tf_samples=[];self.static=[];self.last_tf={};self.poses=[];self.tf_fail=[];self.tf_ok=0
        self.scanlog=gzip.open(out/'scan_raw.jsonl.gz','wt')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage,'/tf',self.tf,100)
        self.create_subscription(TFMessage,'/tf_static',self.static_tf,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointCloud2,'/scan_matched_points2',self.cloud,qos_profile_sensor_data)
        self.create_subscription(SubmapList,'/submap_list',self.submap,10)
        self.create_timer(.1,self.lookup)
    def scan(self,m):
        a=np.asarray(m.ranges,dtype=float);angles=m.angle_min+np.arange(len(a))*m.angle_increment
        valid=np.isfinite(a)&(a>=m.range_min)&(a<=m.range_max)
        t=time.monotonic();row={'elapsed_s':t-self.start,'wall':t,'stamp_ns':ns(m.header.stamp),'age_s':(self.get_clock().now().nanoseconds-ns(m.header.stamp))/1e9,'frame':m.header.frame_id,'angle_min':m.angle_min,'angle_max':m.angle_max,'angle_increment':m.angle_increment,'time_increment':m.time_increment,'scan_time':m.scan_time,'range_min':m.range_min,'range_max':m.range_max,'beams':len(a),'valid':int(valid.sum()),'zero':int((a==0).sum()),'nan':int(np.isnan(a).sum()),'inf':int(np.isinf(a).sum()),'positive_below_min':int(((a>0)&(a<m.range_min)).sum()),'above_max_finite':int((np.isfinite(a)&(a>m.range_max)).sum()),'valid_min':float(a[valid].min()) if valid.any() else None,'valid_max':float(a[valid].max()) if valid.any() else None}
        self.scans.append(row);self.last_scans.append(m)
        self.scanlog.write(json.dumps({**row,'ranges':a.tolist(),'intensities':list(m.intensities)},allow_nan=True)+'\n')
        for ang,d in zip(np.degrees(angles[valid]),a[valid]):
            b=int(math.floor(ang+.5));b=(b+180)%360-180;self.bin_values[b].append(float(d))
    def map(self,m):
        data=np.asarray(m.data,dtype=np.int16);now=self.get_clock().now().nanoseconds
        row={'wall':time.monotonic(),'elapsed_s':time.monotonic()-self.start,'stamp_ns':ns(m.header.stamp),'age_s':(now-ns(m.header.stamp))/1e9,'frame':m.header.frame_id,'resolution':m.info.resolution,'width':m.info.width,'height':m.info.height,'origin_xyz':[m.info.origin.position.x,m.info.origin.position.y,m.info.origin.position.z],'origin_xyzw':[m.info.origin.orientation.x,m.info.origin.orientation.y,m.info.origin.orientation.z,m.info.origin.orientation.w],'unknown':int((data<0).sum()),'free_exact_0':int((data==0).sum()),'free_under_50':int(((data>=0)&(data<50)).sum()),'occupied_ge_50':int((data>=50).sum()),'occupied_ge_65':int((data>=65).sum()),'histogram':{str(int(x)):int(n) for x,n in zip(*np.unique(data,return_counts=True))},'sha256':hashlib.sha256(data.tobytes()).hexdigest()}
        idx=len(self.maps);self.maps.append(row);self.last_map=m
        np.savez_compressed(self.out/f'map_{idx:03d}.npz',data=data.reshape(m.info.height,m.info.width),metadata=json.dumps(row))
    def tf(self,m):
        now=time.monotonic()
        for t in m.transforms:
            key=t.header.frame_id+' -> '+t.child_frame_id;self.edges[key]+=1
            if now-self.last_tf.get(key,0)<.05:continue
            self.last_tf[key]=now;q=t.transform.rotation;v=t.transform.translation
            self.tf_samples.append({'elapsed_s':now-self.start,'stamp_ns':ns(t.header.stamp),'parent':t.header.frame_id,'child':t.child_frame_id,'xyz':[v.x,v.y,v.z],'xyzw':[q.x,q.y,q.z,q.w],'rpy_deg':rpy(q)})
    def static_tf(self,m):
        for t in m.transforms:
            q=t.transform.rotation;v=t.transform.translation
            self.static.append({'parent':t.header.frame_id,'child':t.child_frame_id,'xyz':[v.x,v.y,v.z],'xyzw':[q.x,q.y,q.z,q.w],'rpy_deg':rpy(q)})
    def cloud(self,m):self.clouds.append({'wall':time.monotonic(),'stamp_ns':ns(m.header.stamp),'frame':m.header.frame_id,'points':m.width*m.height})
    def submap(self,m):
        self.submaps.append({'wall':time.monotonic(),'stamp_ns':ns(m.header.stamp),'frame':m.header.frame_id,'submaps':[{'trajectory_id':s.trajectory_id,'submap_index':s.submap_index,'submap_version':s.submap_version,'is_frozen':s.is_frozen,'xyz':[s.pose.position.x,s.pose.position.y,s.pose.position.z],'xyzw':[s.pose.orientation.x,s.pose.orientation.y,s.pose.orientation.z,s.pose.orientation.w]} for s in m.submap]})
    def lookup(self):
        if not self.last_scans:return
        try:
            m=self.last_scans[-1];t=self.buffer.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp))
            q=t.transform.rotation;v=t.transform.translation
            self.poses.append({'elapsed_s':time.monotonic()-self.start,'xyz':[v.x,v.y,v.z],'rpy_deg':rpy(q)});self.tf_ok+=1
        except TransformException as e:self.tf_fail.append({'elapsed_s':time.monotonic()-self.start,'error':str(e).split('\n')[0]})
    def result(self):
        s=self.scans
        def hz(rows):return (len(rows)-1)/(rows[-1]['wall']-rows[0]['wall']) if len(rows)>1 else None
        endpoints={}
        for topic in ['/scan','/map','/tf','/tf_static','/submap_list','/scan_matched_points2']:
            endpoints[topic]={'publishers':[{'node':e.node_name,'type':e.topic_type,'reliability':str(e.qos_profile.reliability),'durability':str(e.qos_profile.durability),'depth':e.qos_profile.depth} for e in self.get_publishers_info_by_topic(topic)],'subscribers':[{'node':e.node_name,'type':e.topic_type,'reliability':str(e.qos_profile.reliability),'durability':str(e.qos_profile.durability),'depth':e.qos_profile.depth} for e in self.get_subscriptions_info_by_topic(topic)]}
        result={'source':'LIVE_HARDWARE_ONLY, PASSIVE_SUBSCRIPTIONS','physical_target_placement_confirmed':False,'scan_count':len(s),'scan_hz':hz(s),'scan_stamp_hz':(len(s)-1)/((s[-1]['stamp_ns']-s[0]['stamp_ns'])/1e9) if len(s)>1 else None,'scan_age_s':stats([a['age_s'] for a in s]),'first_scan':s[0] if s else None,'last_scan':s[-1] if s else None,'valid_beams':stats([a['valid'] for a in s]),'valid_fraction':stats([a['valid']/a['beams'] for a in s]),'zero_beams':stats([a['zero'] for a in s]),'above_max_total':sum(a['above_max_finite'] for a in s),'nan_total':sum(a['nan'] for a in s),'inf_total':sum(a['inf'] for a in s),'map_count':len(self.maps),'map_hz':hz(self.maps),'map_header_age_s':stats([a['age_s'] for a in self.maps]),'distinct_map_hashes':len({a['sha256'] for a in self.maps}),'matched_cloud_count':len(self.clouds),'matched_cloud_hz':hz(self.clouds),'submap_list_count':len(self.submaps),'submap_list_hz':hz(self.submaps),'tf_edges':dict(self.edges),'static_tf':self.static,'tf_lookup_successes':self.tf_ok,'tf_lookup_failures':self.tf_fail,'map_to_laser_pose_first':self.poses[0] if self.poses else None,'map_to_laser_pose_last':self.poses[-1] if self.poses else None,'map_to_laser_yaw_deg':stats([p['rpy_deg'][2] for p in self.poses]),'endpoints':endpoints,'topics':self.get_topic_names_and_types(),'nodes':self.get_node_names(),'one_degree_bins':{str(k):{'samples':len(v),'distance_m':stats(v),'fraction_near_1m':sum(.8<=x<=1.2 for x in v)/len(v)} for k,v in sorted(self.bin_values.items())}}
        self.scanlog.close()
        for name,values in [('scan_summaries',s),('maps',self.maps),('tf_samples',self.tf_samples),('submap_lists',self.submaps),('poses',self.poses),('matched_clouds',self.clouds)]:
            (self.out/f'{name}.json').write_text(json.dumps(values,indent=2)+'\n')
        (self.out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:result[k] for k in ['scan_count','scan_hz','scan_age_s','valid_beams','valid_fraction','map_count','map_hz','matched_cloud_count','submap_list_count','tf_edges','static_tf','tf_lookup_successes','tf_lookup_failures','map_to_laser_yaw_deg']},indent=2))
        return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=45);parser.add_argument('--label',default='baseline');args=parser.parse_args()
    out=Path(__file__).resolve().parent/args.label;out.mkdir(exist_ok=False)
    rclpy.init();node=Collector(out)
    try:
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        node.result()
    finally:node.destroy_node();rclpy.shutdown()
