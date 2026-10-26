"""Only new code converts original LQR Twist into protected MotionCommand."""
import math,time,json
from dataclasses import dataclass
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu
from interfaces_pkg.msg import MotionCommand
from rclpy.qos import qos_profile_sensor_data
from .geometry import VehicleGeometry
from .common import spin,publish_json

@dataclass
class AdapterConfig:
    wheelbase:float;max_steer:float;rate:float;forward:float;reverse:float
    scale:float=1000.;steering_sign:float=-1.;left_sign:float=1.;right_sign:float=1.;max_pwm:int=255

class Conversion:
    def __init__(self,config):self.c=config;self.delta=0.
    def convert(self,v,w,dt):
        c=self.c
        if not all(math.isfinite(x) for x in [v,w,dt]) or dt<=0:raise ValueError('nonfinite control')
        if abs(v)<1e-6:
            if abs(w)>1e-6:raise ValueError('Ackermann vehicle cannot rotate in place')
            # Stop motors but HOLD last steering; centering could actuate steering during an E-stop.
            return int(round(c.steering_sign*self.delta/c.max_steer*7)),0,0
        delta=math.atan(c.wheelbase*w/v)
        delta=max(-c.max_steer,min(c.max_steer,delta));delta=max(self.delta-c.rate*dt,min(self.delta+c.rate*dt,delta));self.delta=delta
        v=max(-c.reverse,min(c.forward,v));pwm=max(-c.max_pwm,min(c.max_pwm,int(round(v*c.scale))))
        return max(-7,min(7,int(round(c.steering_sign*delta/c.max_steer*7)))),int(c.left_sign*pwm),int(c.right_sign*pwm)

class ControlAdapter(Node):
    def __init__(self,**kwargs):
        super().__init__('control_adapter',**kwargs)
        defaults={'geometry_file':'','enable_actuation':False,'motor_scale_pwm_per_m_s':1000.,'motor_scale_confirmed':False,'polarity_confirmed':False,'steering_sign':-1.,'left_motor_sign':1.,'right_motor_sign':1.,'command_timeout':.25,'state_timeout':.25,'require_tractor_imu':True,'max_pwm':255}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.g=VehicleGeometry(self.get_parameter('geometry_file').value);g=self.g.values
        self.c=AdapterConfig(g['tractor_wheelbase'],g['maximum_steering_angle'],g['steering_rate_limit'],g['forward_velocity'],g['reverse_velocity'],self.get_parameter('motor_scale_pwm_per_m_s').value,self.get_parameter('steering_sign').value,self.get_parameter('left_motor_sign').value,self.get_parameter('right_motor_sign').value,self.get_parameter('max_pwm').value)
        if not math.isfinite(self.c.scale) or self.c.scale<=0 or not 0<self.c.max_pwm<=255 or any(s not in [-1.,1.] for s in [self.c.steering_sign,self.c.left_sign,self.c.right_sign]):raise ValueError('invalid adapter scaling/sign/range')
        self.converter=Conversion(self.c);self.command=None;self.last_cmd=0.;self.last_state=0.;self.valid=False;self.imu_wall=0.;self.estop=False;self.terminal=False;self.previous=time.monotonic()
        self.pub=self.create_publisher(MotionCommand,'/topic_control_signal',1);self.preview=self.create_publisher(MotionCommand,'/parking/motion_preview',1);self.status=self.create_publisher(String,'/parking/adapter_status',10)
        self.create_subscription(Twist,'/parking/request_cmd_vel',self.receive,10);self.create_subscription(String,'/parking/state',self.state,100);self.create_subscription(String,'/parking/status',self.parking_status,10)
        self.create_subscription(Imu,'/tractor/imu/data',self.imu,qos_profile_sensor_data);self.create_subscription(Bool,'/parking/emergency_stop',self.emergency,10);self.create_service(Trigger,'/parking/reset_emergency_stop',self.reset)
        self.create_timer(.05,self.tick)
    def receive(self,m):
        values=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
        if not all(math.isfinite(x) for x in values) or any(abs(x)>1e-8 for x in [m.linear.y,m.linear.z,m.angular.x,m.angular.y]):self.command=None;self.estop=True;return
        self.command=(m.linear.x,m.angular.z);self.last_cmd=time.monotonic()
    def state(self,m):
        try:
            j=json.loads(m.data);s=j.get('state');ns=j['stamp_ns'];age=(self.get_clock().now().nanoseconds-ns)/1e9
            self.valid=bool(j['valid']) and s is not None and len(s)==4 and all(math.isfinite(x) for x in s) and -.03<=age<=self.get_parameter('state_timeout').value
            if self.valid and abs(math.atan2(math.sin(s[3]-s[2]),math.cos(s[3]-s[2])))>=math.radians(35):self.valid=False;self.estop=True
            self.last_state=time.monotonic()
        except (ValueError,KeyError,TypeError):self.valid=False
    def imu(self,m):
        age=(self.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e9
        if -.03<=age<=.2 and m.angular_velocity_covariance[0]!=-1 and all(math.isfinite(v) for v in [m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]):self.imu_wall=time.monotonic()
    def parking_status(self,m):
        try:self.terminal=json.loads(m.data).get('state') in ['FAILED','EMERGENCY_STOP','PARKED']
        except ValueError:self.terminal=True
    def emergency(self,m):
        if m.data:self.estop=True
    def reset(self,request,response):
        stopped=self.command is None or all(abs(x)<1e-6 for x in self.command)
        response.success=stopped and not self.terminal
        if response.success:self.estop=False
        response.message='reset' if response.success else 'stop command and nonterminal parking state required';return response
    def tick(self):
        now=time.monotonic();dt=min(.1,max(.001,now-self.previous));self.previous=now
        reason='';ready=self.valid and now-self.last_state<self.get_parameter('state_timeout').value
        if self.get_parameter('require_tractor_imu').value:ready=ready and now-self.imu_wall<.2
        if not ready:reason='state/IMU unavailable'
        elif self.command is None or now-self.last_cmd>self.get_parameter('command_timeout').value:reason='command timeout'
        elif self.estop or self.terminal:reason='emergency or terminal stop'
        v,w=(0.,0.) if reason else self.command
        try:steering,left,right=self.converter.convert(v,w,dt)
        except ValueError as e:reason=str(e);self.estop=True;steering,left,right=self.converter.convert(0.,0.,dt)
        m=MotionCommand();m.steering=steering;m.left_speed=left;m.right_speed=right;self.preview.publish(m)
        enabled=self.get_parameter('enable_actuation').value and self.get_parameter('motor_scale_confirmed').value and self.get_parameter('polarity_confirmed').value
        if enabled:self.pub.publish(m)
        publish_json(self.status,{'enabled':bool(enabled),'valid':not bool(reason),'reason':reason,'steering_steps':steering,'left_pwm':left,'right_pwm':right,'scale_provisional':not self.get_parameter('motor_scale_confirmed').value})
    def close(self):
        if self.get_parameter('enable_actuation').value:
            m=MotionCommand();m.steering=int(round(self.c.steering_sign*self.converter.delta/self.c.max_steer*7));self.pub.publish(m)

def main(args=None):spin(ControlAdapter,args)
