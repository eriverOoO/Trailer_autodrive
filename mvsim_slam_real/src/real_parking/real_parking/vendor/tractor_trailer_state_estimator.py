"""Minimal input adapter: Cartographer tractor state + gyro-integrated trailer yaw."""
import json,math,time
import numpy as np
from std_msgs.msg import String

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp_seconds(s):return s.sec+s.nanosec*1e-9
class StateEstimator:
 def __init__(self,node,source='local'):
  self.node=node;self.state=None;self.pose_stamp=None;self.initial_pose=None;self.last_error='waiting gyro/SLAM4D state';self.wall=0;self.imu_wall=0
  self.sub=node.create_subscription(String,'/parking/state',self.on_state,100)
 def on_state(self,msg):
  j=json.loads(msg.data)
  if not j['valid']:self.state=None;self.last_error=j['reason'];return
  self.state=np.array(j['state'],dtype=float);self.pose_stamp=j['stamp_ns']/1e9;self.wall=time.monotonic();self.imu_wall=self.wall
  if self.initial_pose is None:self.initial_pose=self.state[:3].copy()
  self.last_error=''
 def pose(self):
  now=self.node.get_clock().now().nanoseconds/1e9
  if self.state is None or time.monotonic()-self.wall>.25 or not (-.03<=now-self.pose_stamp<=.3):self.last_error='gyro/SLAM state missing/stale';return None
  return self.state[:3].copy(),0.
 def update(self,calibrate=False):
  if self.pose() is None:return None
  return self.state.copy()
