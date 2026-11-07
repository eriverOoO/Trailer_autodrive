"""Optional RPLidar source using preserved original UART library; no model default."""
import threading,queue,time,math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from .common import spin,publish_json

class RplidarSource(Node):
    def __init__(self,**kwargs):
        super().__init__('rplidar_source',**kwargs)
        for k,v in {'port':'','enable_device':False,'frame_id':'laser_frame','bins':720,'range_min':.15,'range_max':12.,'clockwise':True}.items():self.declare_parameter(k,v)
        self.pub=self.create_publisher(LaserScan,'/lidar_raw',qos_profile_sensor_data);self.status=self.create_publisher(String,'/lidar/source_status',10)
        self.stop=threading.Event();self.q=queue.Queue(maxsize=2);self.lidar=None;self.error='device disabled';self.thread=None
        if self.get_parameter('enable_device').value:
            if not self.get_parameter('port').value:raise ValueError('explicit LiDAR port required')
            self.thread=threading.Thread(target=self.worker,daemon=True);self.thread.start()
        self.create_timer(.02,self.tick);self.create_timer(1.,lambda:publish_json(self.status,{'connected':self.lidar is not None,'reason':self.error}))
    def worker(self):
        try:
            from lidar_perception_pkg.lib.lidar_perception_func_lib import RPLidar
            self.lidar=RPLidar(self.get_parameter('port').value);self.error=''
            previous=time.monotonic()
            for scan in self.lidar.iter_scans():
                if self.stop.is_set():break
                now=time.monotonic();item=(self.get_clock().now().to_msg(),now-previous,scan);previous=now
                try:self.q.put_nowait(item)
                except queue.Full:
                    try:self.q.get_nowait()
                    except queue.Empty:pass
                    self.q.put_nowait(item)
        except Exception as e:self.error=f'{type(e).__name__}: {e}'
        finally:
            if self.lidar:
                try:self.lidar.stop();self.lidar.stop_motor();self.lidar.disconnect()
                except Exception:pass
                self.lidar=None
    def tick(self):
        try:stamp,period,scan=self.q.get_nowait()
        except queue.Empty:return
        n=int(self.get_parameter('bins').value)
        if n<2:return
        m=LaserScan();m.header.stamp=stamp;m.header.frame_id=self.get_parameter('frame_id').value;m.angle_min=-math.pi;m.angle_increment=2*math.pi/n;m.angle_max=m.angle_min+(n-1)*m.angle_increment;m.range_min=float(self.get_parameter('range_min').value);m.range_max=float(self.get_parameter('range_max').value);m.scan_time=float(period)
        # Original UART library has no per-beam timestamps. Binned scan is instantaneous approximation.
        m.time_increment=0.;m.ranges=[float('nan')]*n;m.intensities=[0.]*n
        sign=-1 if self.get_parameter('clockwise').value else 1
        for quality,angle,distance in scan:
            if quality<=0 or distance<=0:continue
            a=(sign*math.radians(angle)+math.pi)%(2*math.pi)-math.pi;i=min(n-1,int((a+math.pi)/m.angle_increment));r=distance/1000.
            if m.range_min<=r<=m.range_max and (math.isnan(m.ranges[i]) or r<m.ranges[i]):m.ranges[i]=r;m.intensities[i]=float(quality)
        if any(math.isfinite(v) for v in m.ranges):self.pub.publish(m)
    def close(self):
        self.stop.set()
        if self.thread:self.thread.join(timeout=2.)

def main(args=None):spin(RplidarSource,args)
