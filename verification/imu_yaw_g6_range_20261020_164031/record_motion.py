"""Read-only real Imu/status/Marker capture for user-confirmed manual turns."""
from pathlib import Path
import json,math,time,signal
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
ROOT=Path('/home/venezuelachocolatemilk/mvsim_slam_real')
OUT=Path(__file__).resolve().parent
poly=json.loads((ROOT/'visualization/tractor_trailer_rviz_2d/models/reference_footprints.json').read_text())['polygons'][0]
edge=math.atan2(poly[1][1]-poly[0][1],poly[1][0]-poly[0][0])
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def ns(s):return s.sec*10**9+s.nanosec
def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi
rclpy.init();node=Node('actual_turn_readonly_recorder');log=(OUT/'actual_motion_stream.jsonl').open('a')
def emit(kind,stamp,**data):log.write(json.dumps(dict(kind=kind,stamp_ns=stamp,receipt_ns=time.time_ns(),**data))+'\n');log.flush()
def imu(m):emit('raw_imu',ns(m.header.stamp),yaw_rad=yaw(m.orientation),frame=m.header.frame_id)
def state(m):
 d=json.loads(m.data);emit('status',d['stamp_ns'],value=d)
def model(m):
 for x in m.markers:
  if x.ns=='tractor_pose_arrow':emit('arrow',ns(x.header.stamp),yaw_rad=yaw(x.pose.orientation),frame=x.header.frame_id,xy=[x.pose.position.x,x.pose.position.y])
  if x.ns=='current_body' and x.id==0 and len(x.points)>=2:
   a,b=x.points[:2];emit('body',ns(x.header.stamp),yaw_rad=wrap(math.atan2(b.y-a.y,b.x-a.x)-edge),frame=x.header.frame_id,points=len(x.points))
subs=[node.create_subscription(Imu,'/vehicle_viz_2d/tractor/imu/data',imu,qos_profile_sensor_data),node.create_subscription(String,'/vehicle_viz_2d/status',state,10),node.create_subscription(MarkerArray,'/vehicle_viz_2d/model',model,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:log.close();node.destroy_node();rclpy.try_shutdown()
