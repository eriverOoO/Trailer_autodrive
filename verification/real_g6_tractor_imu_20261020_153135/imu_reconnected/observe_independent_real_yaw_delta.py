"""Subscribe only; evaluate existing 2D YawTrack using real tractor IMU samples."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tractor_trailer_rviz_2d.core import YawTrack
rclpy.init();n=Node('real_tractor_yaw_readonly_check');track=YawTrack();records=[];initial=None
before=track.state(time.monotonic(),n.get_clock().now().nanoseconds)
def receive(m):
 global initial
 stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec;wall=time.monotonic();now=n.get_clock().now().nanoseconds
 q=m.orientation;ok=track.push(stamp,wall,now,q=(q.x,q.y,q.z,q.w))
 if ok:
  if initial is None:initial=track.value
  records.append(dict(stamp_ns=stamp,yaw_deg=math.degrees(track.value),delta_deg=math.degrees(track.value-initial),state=track.state(wall,now)))
s=n.create_subscription(Imu,'/ebimu_diag/tractor/imu/data',receive,qos_profile_sensor_data)
start=time.monotonic()
while time.monotonic()-start<8:rclpy.spin_once(n,timeout_sec=.02)
result=dict(publishes_nothing=True,source='real independent diagnostic tractor Imu topic',initial_state=before,final_state=track.state(time.monotonic(),n.get_clock().now().nanoseconds),accepted=track.accepted,rejected=track.rejected,first=records[0] if records else None,last=records[-1] if records else None,delta_min_deg=min((r['delta_deg'] for r in records),default=None),delta_max_deg=max((r['delta_deg'] for r in records),default=None),reference='First received real sample; native sensor delta only. Mounting/sign and map alignment unverified.')
p=Path(__file__).parent;(p/'yaw_delta_independent_live.json').write_text(json.dumps(result,indent=2));(p/'yaw_delta_independent_live_samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records));print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
