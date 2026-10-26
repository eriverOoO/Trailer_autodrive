"""Manufacturer independent LaserScan validation and coordinate adapter."""
import copy, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from .common import spin,publish_json

def normalize_scan(msg,frame,offset=0.,reverse=False):
    n=len(msg.ranges)
    vals=[msg.angle_min,msg.angle_max,msg.angle_increment,msg.range_min,msg.range_max,msg.time_increment,msg.scan_time]
    if n<2 or not all(math.isfinite(v) for v in vals) or msg.angle_increment<=0 or not 0<=msg.range_min<msg.range_max or min(msg.time_increment,msg.scan_time)<0:
        raise ValueError('invalid scan geometry')
    if abs(msg.angle_max-(msg.angle_min+(n-1)*msg.angle_increment))>msg.angle_increment*1.1:raise ValueError('angle/beam count mismatch')
    if msg.intensities and len(msg.intensities)!=n:raise ValueError('intensity count mismatch')
    if reverse and msg.time_increment>0:raise ValueError('time-ordered scan reflection requires upstream timestamp-aware driver')
    if not msg.header.frame_id:raise ValueError('missing source frame')
    out=copy.deepcopy(msg)
    out.ranges=[v if math.isfinite(v) and msg.range_min<=v<=msg.range_max else (float('inf') if v==float('inf') else float('nan')) for v in msg.ranges]
    if not any(math.isfinite(v) for v in out.ranges):raise ValueError('no finite scan returns')
    if reverse:
        out.ranges.reverse();out.intensities.reverse();out.angle_min=-msg.angle_max;out.angle_max=-msg.angle_min
    out.angle_min+=offset;out.angle_max+=offset;out.header.frame_id=frame
    return out

class LidarGateway(Node):
    def __init__(self,**kwargs):
        super().__init__('lidar_gateway',**kwargs)
        d={'input_topic':'/lidar_raw','output_topic':'/scan','frame_id':'laser_frame','angle_offset':0.,'reverse_direction':False,'timeout':.5}
        for k,v in d.items():self.declare_parameter(k,v)
        self.last=None;self.last_stamp=0;self.error='no scan received'
        self.pub=self.create_publisher(LaserScan,self.get_parameter('output_topic').value,QoSProfile(depth=10))
        self.status=self.create_publisher(String,'/lidar/status',10)
        self.create_subscription(LaserScan,self.get_parameter('input_topic').value,self.receive,qos_profile_sensor_data)
        self.create_timer(.2,self.tick)
    def receive(self,m):
        try:
            ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec;age=(self.get_clock().now().nanoseconds-ns)/1e9
            if ns<=self.last_stamp or not -.03<=age<=self.get_parameter('timeout').value:raise ValueError('scan timestamp stale/backward')
            out=normalize_scan(m,self.get_parameter('frame_id').value,self.get_parameter('angle_offset').value,self.get_parameter('reverse_direction').value)
            self.pub.publish(out);self.last=time.monotonic();self.last_stamp=ns;self.error=''
        except ValueError as e:self.error=str(e)
    def tick(self):
        valid=self.last is not None and time.monotonic()-self.last<self.get_parameter('timeout').value and not self.error
        publish_json(self.status,{'valid':valid,'reason':self.error if not valid else '', 'hardware_verified':False})

def main(args=None):spin(LidarGateway,args)
