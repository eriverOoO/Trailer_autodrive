"""EXPLICIT SYNTHETIC TEST ONLY. Publishes solely /vehicle_viz_2d_test/* and test TF."""
import math
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster


def scenario(t):
    if t<2:return 'ALIGN_STATIONARY',0.,0.
    if t<4:return 'A_TRACTOR_ONLY',(t-2)/2*30,0.
    if t<6:return 'A_RETURN',(6-t)/2*30,0.
    if t<8:return 'B_TRAILER_ONLY',0.,(t-6)/2*20
    if t<10:return 'B_RETURN',0.,(10-t)/2*20
    if t<12:return 'C_DIFFERENT',(t-10)/2*30,(t-10)/2*10
    if t<14:return 'C_RETURN',(14-t)/2*30,(14-t)/2*10
    if t<16:return 'D_SAME',(t-14)/2*45,(t-14)/2*45
    if t<18:return 'D_RETURN',(18-t)/2*45,(18-t)/2*45
    if t<20:return 'E_TRACTOR_STOP',0.,0.
    if t<22:return 'RECOVER_REQUIRES_REALIGN',0.,0.
    if t<23:return 'F_INVALID_TRAILER',0.,0.
    if t<26:return 'RECOVER_INVALID',0.,0.
    return 'G_SLAM_YAW_ONLY',0.,0.


class TestSource(Node):
    def __init__(self):
        super().__init__('vehicle_viz_explicit_test_source')
        self.declare_parameter('confirm_test',False)
        if not self.get_parameter('confirm_test').value:raise ValueError('confirm_test:=true required; synthetic only')
        self.start=time.monotonic()
        self.imus={r:self.create_publisher(Imu,'/vehicle_viz_2d_test/'+r+'/imu/data',qos_profile_sensor_data) for r in ('tractor','trailer')}
        self.state=self.create_publisher(String,'/vehicle_viz_2d_test/parking_status',10)
        self.phase=self.create_publisher(String,'/vehicle_viz_2d_test/phase',10)
        self.tf=TransformBroadcaster(self)
        self.map=self.create_publisher(OccupancyGrid,'/vehicle_viz_2d_test/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan=self.create_publisher(LaserScan,'/vehicle_viz_2d_test/scan',qos_profile_sensor_data)
        self.client=self.create_client(SetBool,'/vehicle_viz_2d_test/align_straight')
        self.requests=set()
        self.futures=[]
        self.create_timer(.01,self.tick)
        self.create_timer(1.,self.publish_map)
        self.publish_map()

    def publish_map(self):
        m=OccupancyGrid();m.header.frame_id='vehicle_viz_2d_test_map'
        m.header.stamp=self.get_clock().now().to_msg();m.info.resolution=.1
        m.info.width=m.info.height=120
        m.info.origin.position.x=m.info.origin.position.y=-6.
        m.info.origin.orientation.w=1.
        m.data=[100 if x in (0,119) or y in (0,119) else 0 for y in range(120) for x in range(120)]
        self.map.publish(m)

    def tick(self):
        t=time.monotonic()-self.start
        phase,a,b=scenario(t)
        stamp=self.get_clock().now().to_msg()
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='vehicle_viz_2d_test_map'
        tf.child_frame_id='vehicle_viz_2d_test_slam'
        slam_yaw=.6 if t>=26 else 0.
        tf.transform.rotation.z,tf.transform.rotation.w=math.sin(slam_yaw/2),math.cos(slam_yaw/2)
        self.tf.sendTransform(tf)
        for role,deg in [('tractor',a),('trailer',b)]:
            if phase=='E_TRACTOR_STOP' and role=='tractor':continue
            m=Imu();m.header.stamp=stamp;m.header.frame_id='vehicle_viz_2d_test_'+role+'_imu'
            yaw=math.radians(deg)
            m.orientation.z,m.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
            if phase=='F_INVALID_TRAILER' and role=='trailer':m.orientation.w=0.
            m.angular_velocity_covariance[0]=m.linear_acceleration_covariance[0]=-1.
            self.imus[role].publish(m)
        self.phase.publish(String(data=phase))
        if t<26:
            self.state.publish(String(data=json.dumps(dict(state='SEARCH_FORWARD',stamp_ns=stamp.sec*10**9+stamp.nanosec))))
        if int(t*100)%10==0:
            m=LaserScan();m.header.stamp=stamp;m.header.frame_id='vehicle_viz_2d_test_slam'
            m.angle_min=-math.pi;m.angle_max=math.pi;m.angle_increment=math.pi/180
            m.range_min=.15;m.range_max=15.;m.ranges=[5.]*361
            self.scan.publish(m)
        for key,lo,hi in [('initial',1.,1.8),('after_gap',21.,21.8),('after_invalid',24.,25.)]:
            if lo<t<hi and key not in self.requests and self.client.service_is_ready():
                req=SetBool.Request();req.data=True
                self.futures.append(self.client.call_async(req));self.requests.add(key)


def main(args=None):
    from .runtime import run
    run(TestSource,args)
