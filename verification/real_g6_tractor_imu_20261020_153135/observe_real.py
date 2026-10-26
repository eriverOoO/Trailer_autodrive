#!/usr/bin/python3
"""Read-only real ROS evidence collection: publishes no sensor, TF, or control data."""
import collections, hashlib, json, math, sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import LaserScan,Imu,PointCloud2
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import MarkerArray
from cartographer_ros_msgs.msg import SubmapList
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
rclpy.init();node=Node('real_sensor_readonly_verification')
output=Path(sys.argv[1]);duration=float(sys.argv[2])
records=collections.defaultdict(list);edges=collections.Counter();last_scan=None
buf=Buffer();listener=TransformListener(buf,node)
def ns(s):return s.sec*10**9+s.nanosec
def add(key,stamp,**v):
 records[key].append(dict(receipt=time.monotonic(),stamp_ns=stamp,**v))
def scan(m):
 global last_scan
 values=[x for x in m.ranges if math.isfinite(x) and m.range_min<=x<=m.range_max]
 add('scan',ns(m.header.stamp),frame=m.header.frame_id,rays=len(m.ranges),valid=len(values),minimum=min(values) if values else None,maximum=max(values) if values else None,range_min=m.range_min,range_max=m.range_max,scan_time=m.scan_time,time_increment=m.time_increment,age_s=(node.get_clock().now().nanoseconds-ns(m.header.stamp))/1e9)
 last_scan=m
 def finite(x):return x if math.isfinite(x) else None
 if len(records['scan'])==1:(output/'scan_first.json').write_text(json.dumps(dict(header=dict(stamp_ns=ns(m.header.stamp),frame_id=m.header.frame_id),angle_min=m.angle_min,angle_max=m.angle_max,angle_increment=m.angle_increment,ranges=[finite(x) for x in m.ranges]),indent=2))
def grid(m):
 counts=collections.Counter(m.data)
 add('map',ns(m.header.stamp),frame=m.header.frame_id,width=m.info.width,height=m.info.height,resolution=m.info.resolution,known=len(m.data)-counts[-1],occupied=sum(n for v,n in counts.items() if v>=65),free=sum(n for v,n in counts.items() if 0<=v<=25),digest=hashlib.sha256(bytes((x+1)%256 for x in m.data)).hexdigest(),origin=[m.info.origin.position.x,m.info.origin.position.y])
def tf(m):
 for t in m.transforms:
  key=t.header.frame_id+' -> '+t.child_frame_id;edges[key]+=1
  q=t.transform.rotation;v=t.transform.translation
  add('tf:'+key,ns(t.header.stamp),xy=[v.x,v.y],yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
def imu(m):
 q=m.orientation
 add('imu',ns(m.header.stamp),frame=m.header.frame_id,yaw_deg=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))),norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w),age_s=(node.get_clock().now().nanoseconds-ns(m.header.stamp))/1e9)
def markers(m):add('markers',node.get_clock().now().nanoseconds,items=[dict(ns=x.ns,type=x.type,points=len(x.points),text=x.text,frame=x.header.frame_id) for x in m.markers])
subs=[node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data),node.create_subscription(OccupancyGrid,'/map',grid,QoSProfile(depth=5,durability=DurabilityPolicy.TRANSIENT_LOCAL)),node.create_subscription(TFMessage,'/tf',tf,100),node.create_subscription(Imu,'/vehicle_viz_2d/tractor/imu/data',imu,qos_profile_sensor_data),node.create_subscription(PointCloud2,'/scan_matched_points2',lambda m:add('matched_points',ns(m.header.stamp),frame=m.header.frame_id,points=m.width*m.height),qos_profile_sensor_data),node.create_subscription(SubmapList,'/submap_list',lambda m:add('submaps',ns(m.header.stamp),versions=[dict(trajectory_id=s.trajectory_id,submap_index=s.submap_index,version=s.submap_version) for s in m.submap]),10),node.create_subscription(String,'/vehicle_viz_2d/status',lambda m:add('viz_status',node.get_clock().now().nanoseconds,value=json.loads(m.data)),10),node.create_subscription(DiagnosticArray,'/vehicle_viz_2d/tractor/diagnostics',lambda m:add('imu_diagnostics',ns(m.header.stamp),status=[dict(message=d.message,values={v.key:v.value for v in d.values}) for d in m.status]),10),node.create_subscription(MarkerArray,'/vehicle_viz_2d/model',markers,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
start=time.monotonic()
while time.monotonic()-start<duration:rclpy.spin_once(node,timeout_sec=.05)
summary={}
for k,rows in records.items():
 times=[r['receipt'] for r in rows];stamps=[r['stamp_ns'] for r in rows]
 summary[k]=dict(count=len(rows),receipt_hz=(len(rows)-1)/(times[-1]-times[0]) if len(rows)>1 and times[-1]>times[0] else None,stamp_hz=(len(rows)-1)*1e9/(stamps[-1]-stamps[0]) if len(rows)>1 and stamps[-1]>stamps[0] else None,nonincreasing_stamps=sum(b<=a for a,b in zip(stamps,stamps[1:])),first=rows[0],last=rows[-1])
 if k=='map':summary[k]['unique_grids']=len(set(r['digest'] for r in rows))
 if k=='imu':summary[k]['yaw_min_deg']=min(r['yaw_deg'] for r in rows);summary[k]['yaw_max_deg']=max(r['yaw_deg'] for r in rows)
 if k=='scan':summary[k]['valid_min']=min(r['valid'] for r in rows);summary[k]['valid_max']=max(r['valid'] for r in rows)
summary['tf_edges']=dict(edges)
summary['lookup']={}
for frame in ['laser_frame','odom','base_footprint','base_link']:
 try:
  t=buf.lookup_transform('map',frame,Time());summary['lookup'][frame]=dict(available=True,stamp_ns=ns(t.header.stamp))
 except Exception as e:summary['lookup'][frame]=dict(available=False,reason=str(e))
if last_scan:
 try:buf.lookup_transform('map',last_scan.header.frame_id,Time(nanoseconds=ns(last_scan.header.stamp)));summary['scan_timestamp_tf']=True
 except Exception as e:summary['scan_timestamp_tf']=str(e)
summary['graph_nodes']=node.get_node_names_and_namespaces();summary['graph_topics']=node.get_topic_names_and_types()
(output/'observation_records.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
(output/'observation_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False,indent=2));node.destroy_node();rclpy.shutdown()
