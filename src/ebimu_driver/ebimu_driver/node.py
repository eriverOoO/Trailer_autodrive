import math,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from .protocol import AsciiParser

class EbimuNode(Node):
    def __init__(self,**kwargs):
        super().__init__('ebimu_node',**kwargs)
        d={'port':'','enable_device':False,'baud':115200,'frame_id':'tractor_imu_link','output_topic':'/tractor/imu/data','mounting_rpy':[0.,0.,0.],'publish_orientation':True,'timeout':.2,'orientation_variance':.01,'gyro_variance':.0001,'output_period':.01,'include_acceleration':False,'acceleration_variance':.04}
        d.update(orientation_format='euler',include_timestamp=False)
        for k,v in d.items():self.declare_parameter(k,v)
        for key in ['timeout','output_period','orientation_variance','gyro_variance','acceleration_variance']:
            value=self.get_parameter(key).value
            if not math.isfinite(value) or value<=0:raise ValueError('invalid '+key)
        rpy=self.get_parameter('mounting_rpy').value
        if len(rpy)!=3 or not all(math.isfinite(x) for x in rpy):raise ValueError('mounting_rpy must be finite XYZ radians')
        self.rotation=Rotation.from_euler('xyz',rpy);self.parser=AsciiParser(include_acceleration=self.get_parameter('include_acceleration').value,orientation_format=self.get_parameter('orientation_format').value,include_timestamp=self.get_parameter('include_timestamp').value);self.serial=None;self.last=None;self.last_stamp=0;self.last_warn=0;self.last_byte=None
        self.pub=self.create_publisher(Imu,self.get_parameter('output_topic').value,qos_profile_sensor_data)
        if self.get_parameter('enable_device').value:
            port=self.get_parameter('port').value
            if not port:raise ValueError('explicit independent IMU port required')
            import serial
            self.serial=serial.Serial(port=port,baudrate=self.get_parameter('baud').value,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0,xonxoff=False,rtscts=False,dsrdtr=False,exclusive=True)
            # No configuration write and no calibration on startup.
        else:self.get_logger().warning('IMU hardware disabled; no samples will be fabricated')
        self.create_timer(.002,self.poll)
    def message(self,sample,stamp):
        m=Imu();m.header.frame_id=self.get_parameter('frame_id').value;m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9
        g=self.rotation.apply(sample.gyro_rad_s);m.angular_velocity.x=float(g[0]);m.angular_velocity.y=float(g[1]);m.angular_velocity.z=float(g[2])
        gv=self.get_parameter('gyro_variance').value
        m.angular_velocity_covariance=[gv,0.,0.,0.,gv,0.,0.,0.,gv]
        m.linear_acceleration_covariance[0]=-1.
        if sample.ax is not None:
            a=self.rotation.apply(np.asarray([sample.ax,sample.ay,sample.az])*9.80665)
            m.linear_acceleration.x=float(a[0]);m.linear_acceleration.y=float(a[1]);m.linear_acceleration.z=float(a[2]);av=self.get_parameter('acceleration_variance').value
            m.linear_acceleration_covariance=[av,0.,0.,0.,av,0.,0.,0.,av]
        if self.get_parameter('publish_orientation').value:
            attitude=(Rotation.from_quat(sample.quaternion_xyzw) if sample.quaternion_xyzw is not None else Rotation.from_euler('xyz',sample.euler_rad))
            q=(attitude*self.rotation.inv()).as_quat()
            m.orientation.x=float(q[0]);m.orientation.y=float(q[1]);m.orientation.z=float(q[2]);m.orientation.w=float(q[3]);ov=self.get_parameter('orientation_variance').value
            m.orientation_covariance=[ov,0.,0.,0.,ov,0.,0.,0.,ov]
        else:m.orientation_covariance[0]=-1.
        return m
    def poll(self):
        if self.serial is None:return
        try:
            data=self.serial.read(min(4096,self.serial.in_waiting));wall=time.monotonic()
            if self.last_byte is not None and wall-self.last_byte>self.get_parameter('timeout').value:self.parser.buffer.clear()
            if data:self.last_byte=wall
            samples=self.parser.feed(data)
            now=self.get_clock().now().nanoseconds;period=int(self.get_parameter('output_period').value*1e9)
            for i,s in enumerate(samples):
                stamp=now-(len(samples)-1-i)*period
                # Bursts use host receipt time, no sensor hardware clock claimed.
                if stamp<=self.last_stamp:continue
                self.pub.publish(self.message(s,stamp));self.last_stamp=stamp;self.last=time.monotonic()
            if (self.last is None or time.monotonic()-self.last>self.get_parameter('timeout').value) and time.monotonic()-self.last_warn>2:
                self.last_warn=time.monotonic();self.get_logger().warning(f'No valid EBIMU data; rejected frames={self.parser.rejected}')
        except (OSError,ValueError) as e:
            self.get_logger().error(f'IMU receive failed: {e}');self.serial.close();self.serial=None
    def destroy_node(self):
        if self.serial:self.serial.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);node=None
    try:node=EbimuNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
