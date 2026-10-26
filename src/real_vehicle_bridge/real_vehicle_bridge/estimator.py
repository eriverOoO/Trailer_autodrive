"""SLAM-authoritative tractor pose + preserved Final B gyro integration."""
import copy,json,math,time
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from geometry_msgs.msg import Polygon,Point32,PoseStamped,TransformStamped
from sensor_msgs.msg import Imu,LaserScan,PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .common import spin,publish_json
from .geometry import VehicleGeometry
from .yaw_estimator import GyroYaw

def yaw(q):
    vals=[q.x,q.y,q.z,q.w]
    if not np.isfinite(vals).all() or abs(sum(x*x for x in vals)-1)>.01:raise ValueError('invalid quaternion')
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def ns(stamp):return stamp.sec*10**9+stamp.nanosec

class StateEstimator(Node):
    def __init__(self,**kwargs):
        super().__init__('state_estimator',**kwargs)
        for k,v in {'geometry_file':'','pose_timeout':.3,'imu_timeout':.15,'scan_timeout':.4,'map_timeout':2.5,'require_tractor_imu':True}.items():self.declare_parameter(k,v)
        self.vehicle=VehicleGeometry(self.get_parameter('geometry_file').value);g=self.vehicle.values
        self.gyro=GyroYaw();self.gyro.bias=float(g['trailer_gyro_bias_rad_s']);self.gyro.bias_sigma=None
        self.aligned=bool(g['initial_alignment_confirmed']);self.biased=bool(g['trailer_bias_confirmed']);self.initial_phi=float(g['initial_articulation_rad'])
        if not math.isfinite(self.initial_phi) or abs(self.initial_phi)>=math.radians(30) or not math.isfinite(self.gyro.bias):raise ValueError('invalid initial yaw alignment/bias')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self);self.dynamic=TransformBroadcaster(self)
        self.imu_wall=0.;self.tractor_wall=0.;self.scan_wall=0.;self.match_wall=0.;self.match_stamp=0;self.scan=None;self.map=None;self.map_wall=0.;self.state=None;self.last_pose=None;self.last_pose_ns=0;self.pose_history=[];self.tf_packets={};self.pose_fault=None;self.error='waiting SLAM/IMU'
        self.pub=self.create_publisher(String,'/parking/state',100);self.status=self.create_publisher(String,'/tractor/localization/status',20)
        self.tracking_pub=self.create_publisher(RosPath,'/parking/tracking_path',10);self.tracking=RosPath();self.tracking.header.frame_id='map';self.tracking_stamp=0
        self.tractor_pose=self.create_publisher(PoseStamped,'/parking/tractor_slam_pose',20);self.trailer_pose=self.create_publisher(PoseStamped,'/parking/trailer_pose',20)
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.local_map=self.create_publisher(OccupancyGrid,'/tractor/localization/local_map',q);self.detection_map=self.create_publisher(OccupancyGrid,'/parking/detection_map',q)
        self.polypubs=[]
        for name,poly in zip(['tractor','trailer'],self.vehicle.polygons()):
            pub=self.create_publisher(Polygon,f'/{name}/chassis_polygon',q);m=Polygon();m.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in poly];pub.publish(m);self.polypubs.append(pub)
        self.create_subscription(TFMessage,'/tf',self.capture_tf,100)
        self.create_subscription(Imu,'/trailer/imu/data',self.trailer_imu,qos_profile_sensor_data);self.create_subscription(Imu,'/tractor/imu/data',self.tractor_imu,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data);self.create_subscription(PointCloud2,'/scan_matched_points2',self.matched,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,q)
        self.create_service(Trigger,'/parking/reset_trailer_integrator',self.reset)
        self.create_timer(.05,self.tick)
    def capture_tf(self,message):
        # Buffer actual Cartographer poses at TF publication rate, not the 20Hz control timer.
        # Both same-stamp edges are composed; dynamic trailer TF is not a pose source.
        for t in message.transforms:
            key=(t.header.frame_id,t.child_frame_id)
            if key not in [('map','odom'),('odom','base_footprint')]:continue
            stamp=ns(t.header.stamp)
            try:
                theta=yaw(t.transform.rotation);v=t.transform.translation
                value=np.array([v.x,v.y,theta])
                if not np.isfinite(value).all():raise ValueError('invalid TF translation')
                packet=self.tf_packets.setdefault(stamp,{})
                packet[key]=value
                if len(packet)==2:
                    a=packet[('map','odom')];b=packet[('odom','base_footprint')];c,s=math.cos(a[2]),math.sin(a[2]);angle=math.atan2(math.sin(a[2]+b[2]),math.cos(a[2]+b[2]));center=np.array([a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1]]);pose=np.r_[self.vehicle.tractor_axle(center,angle),angle]
                    if not self.pose_history or stamp>self.pose_history[-1][0]:
                        self.pose_history.append((stamp,pose));self.pose_history=self.pose_history[-1000:]
            except ValueError as e:self.pose_fault=str(e)
        if len(self.tf_packets)>40:self.tf_packets={k:self.tf_packets[k] for k in sorted(self.tf_packets)[-20:]}
    def reset(self,request,response):
        # Reset never performs sensor calibration; requires user-confirmed geometry alignment and stationary SLAM.
        moving=self.last_pose is not None and getattr(self,'measured_speed',0.)>.01
        response.success=self.aligned and self.biased and not moving
        if response.success:
            old=self.gyro;self.gyro=GyroYaw();self.gyro.bias=old.bias;self.imu_wall=0.;self.state=None
        response.message='integrator reset; waiting new alignment anchor' if response.success else 'confirmed stationary initial alignment/bias required';return response
    def acceptable_imu(self,m):
        age=(self.get_clock().now().nanoseconds-ns(m.header.stamp))/1e9
        return -.03<=age<=self.get_parameter('imu_timeout').value and m.angular_velocity_covariance[0]!=-1 and np.isfinite([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]).all()
    def trailer_imu(self,m):
        if m.header.frame_id!='trailer_imu_link' or not self.acceptable_imu(m):return
        # physical sensor-link orientation rotated into trailer body; driver axis correction is already applied.
        from scipy.spatial.transform import Rotation
        r=Rotation.from_euler('xyz',self.vehicle.values['trailer_imu_offset'][3:]);v=r.apply([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
        before=self.gyro.count;self.gyro.push(ns(m.header.stamp),float(v[2]),-1.)
        if self.gyro.count>before:self.imu_wall=time.monotonic()
    def tractor_imu(self,m):
        if m.header.frame_id=='tractor_imu_link' and self.acceptable_imu(m):self.tractor_wall=time.monotonic()
    def on_scan(self,m):
        self.scan=m;self.scan_wall=time.monotonic()
    def matched(self,m):
        self.match_wall=time.monotonic();self.match_stamp=ns(m.header.stamp)
    def on_map(self,m):
        if m.header.frame_id!='map' or m.info.resolution<=0 or len(m.data)!=m.info.width*m.info.height:return
        self.map=m;self.map_wall=time.monotonic()
        observed=copy.deepcopy(m);observed.header.frame_id='local_map';observed.data=np.where(np.array(m.data)<0,-1,np.where(np.array(m.data)>=50,100,0)).astype(int).tolist();self.detection_map.publish(observed)
        local=copy.deepcopy(m);local.header.frame_id='local_map';local.data=self.collision_data(m).ravel().astype(int).tolist();self.local_map.publish(local)
    def collision_data(self,m):
        # Preserve original measured-ray unknown→free augmentation, .15m endpoint margin.
        data=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width);scan=self.scan
        if scan is None or time.monotonic()-self.scan_wall>self.get_parameter('scan_timeout').value:return data
        try:tf=self.buffer.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp))
        except TransformException:return data
        try:angle=yaw(tf.transform.rotation);origin_yaw=yaw(m.info.origin.orientation)
        except ValueError:return data
        rs=np.asarray(scan.ranges);maxfree=np.where(np.isposinf(rs),scan.range_max-.15,rs-.15);valid=(rs>scan.range_min)&~np.isnan(rs)&(maxfree>scan.range_min)
        # Batch rays bounds memory for dense sensors while retaining original half-cell spacing.
        distances=np.arange(max(.02,scan.range_min),scan.range_max-.14,m.info.resolution*.5)
        for lo in range(0,len(rs),128):
            values=maxfree[lo:lo+128];keep=valid[lo:lo+128,None]&(distances[None,:]<values[:,None]);angles=angle+scan.angle_min+np.arange(lo,min(lo+128,len(rs)))*scan.angle_increment
            aa=np.broadcast_to(angles[:,None],keep.shape)[keep];dd=np.broadcast_to(distances[None,:],keep.shape)[keep]
            x=tf.transform.translation.x+dd*np.cos(aa);y=tf.transform.translation.y+dd*np.sin(aa);o=m.info.origin;c,s=math.cos(origin_yaw),math.sin(origin_yaw);dx=x-o.position.x;dy=y-o.position.y;i=np.floor((c*dx+s*dy)/m.info.resolution).astype(int);j=np.floor((-s*dx+c*dy)/m.info.resolution).astype(int);inside=(i>=0)&(j>=0)&(i<m.info.width)&(j<m.info.height);i=i[inside];j=j[inside];unknown=data[j,i]<0;data[j[unknown],i[unknown]]=0
        return data
    def evaluate(self):
        now=self.get_clock().now().nanoseconds;wall=time.monotonic()
        if not self.aligned or not self.biased:raise ValueError('initial articulated alignment and trailer bias not confirmed')
        if self.gyro.fault:raise ValueError(self.gyro.fault)
        if self.pose_fault:raise ValueError(self.pose_fault)
        if wall-self.imu_wall>self.get_parameter('imu_timeout').value:raise ValueError('trailer IMU missing/stale')
        if self.get_parameter('require_tractor_imu').value and wall-self.tractor_wall>self.get_parameter('imu_timeout').value:raise ValueError('tractor IMU missing/stale')
        if self.scan is None or wall-self.scan_wall>self.get_parameter('scan_timeout').value or not -.03<=(now-ns(self.scan.header.stamp))/1e9<=self.get_parameter('scan_timeout').value:raise ValueError('scan missing/stale')
        if wall-self.match_wall>self.get_parameter('scan_timeout').value or not -.03<=(now-self.match_stamp)/1e9<=self.get_parameter('scan_timeout').value:raise ValueError('Cartographer actual scan matching missing/stale')
        if self.map is None or wall-self.map_wall>self.get_parameter('map_timeout').value or not any(v>=50 for v in self.map.data):raise ValueError('SLAM map unavailable')
        tf=self.buffer.lookup_transform('map','base_footprint',Time());stamp=ns(tf.header.stamp)
        if not -.03<=(now-stamp)/1e9<=self.get_parameter('pose_timeout').value:raise ValueError('SLAM pose stale')
        theta=yaw(tf.transform.rotation);xy=self.vehicle.tractor_axle([tf.transform.translation.x,tf.transform.translation.y],theta);pose=np.r_[xy,theta]
        if not np.isfinite(pose).all():raise ValueError('nonfinite SLAM pose')
        if stamp>self.last_pose_ns:
            if self.last_pose is not None:
                dt=(stamp-self.last_pose_ns)/1e9;dp=np.linalg.norm(pose[:2]-self.last_pose[:2]);da=abs(math.atan2(math.sin(theta-self.last_pose[2]),math.cos(theta-self.last_pose[2])))
                if dp>.15 or da>math.radians(5):raise ValueError('SLAM pose jump')
                self.measured_speed=float(dp/max(dt,1e-9))
            self.last_pose=pose.copy();self.last_pose_ns=stamp;self.pose_history.append((stamp,pose.copy()));self.pose_history=self.pose_history[-1000:]
        elif stamp<self.last_pose_ns:raise ValueError('SLAM clock reversed')
        # Final B coherent pairing: choose actual buffered pose covered by gyro, within 30ms.
        choices=[(t,p) for t,p in self.pose_history if self.gyro.last_ns is not None and t<=self.gyro.last_ns and 0<=stamp-t<=30_000_000]
        if not choices:raise ValueError(f'gyro/SLAM timestamp pairing unavailable; pose={stamp}, gyro={self.gyro.last_ns}, lag_ms={(stamp-self.gyro.last_ns)/1e6 if self.gyro.last_ns else None}, buffered={len(self.pose_history)}')
        stamp,pose=choices[-1]
        if not self.gyro.initialize(stamp,pose[2]+self.initial_phi):raise ValueError('waiting gyro anchor')
        ty=self.gyro.at(stamp)
        if ty is None:raise ValueError('gyro/SLAM integration interval unavailable')
        if abs(math.atan2(math.sin(ty-pose[2]),math.cos(ty-pose[2])))>=math.radians(35):raise ValueError('articulation emergency limit')
        return np.r_[pose,ty],stamp
    def pose_message(self,xy,angle,stamp):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9;m.pose.position.x=float(xy[0]);m.pose.position.y=float(xy[1]);m.pose.orientation.z=math.sin(angle/2);m.pose.orientation.w=math.cos(angle/2);return m
    def tick(self):
        state=None;stamp=self.get_clock().now().nanoseconds
        try:state,stamp=self.evaluate();self.error=''
        except (ValueError,TransformException) as e:self.error=str(e)
        self.state=state
        publish_json(self.pub,{'valid':state is not None,'state':state.tolist() if state is not None else None,'stamp_ns':stamp,'reason':self.error,'tractor_source':'Cartographer rear axle','trailer_source':'Final B gyro integration','articulation_rad':float(math.atan2(math.sin(state[3]-state[2]),math.cos(state[3]-state[2]))) if state is not None else None})
        publish_json(self.status,{'initialized':state is not None,'source':'Cartographer+trailerGyro','error':self.error})
        if state is not None:
            self.tractor_pose.publish(self.pose_message(state[:2],state[2],stamp))
            if stamp>self.tracking_stamp:
                self.tracking_stamp=stamp;self.tracking.header.stamp=self.pose_message(state[:2],state[2],stamp).header.stamp;self.tracking.poses.append(self.pose_message(state[:2],state[2],stamp));self.tracking.poses=self.tracking.poses[-3000:];self.tracking_pub.publish(self.tracking)
            center=self.vehicle.trailer_center(state);self.trailer_pose.publish(self.pose_message(center,state[3],stamp))
            g=self.vehicle.values;rear=g['tractor_rear_axle_position'];c,s=math.cos(state[2]),math.sin(state[2]);body=state[:2]-np.array([c*rear[0]-s*rear[1],s*rear[0]+c*rear[1]]);diff=center-body;local=np.array([[c,s],[-s,c]])@diff
            tf=TransformStamped();tf.header.stamp=self.pose_message([0,0],0,stamp).header.stamp;tf.header.frame_id='base_footprint';tf.child_frame_id='trailer_link';tf.transform.translation.x=float(local[0]);tf.transform.translation.y=float(local[1]);a=state[3]-state[2];tf.transform.rotation.z=math.sin(a/2);tf.transform.rotation.w=math.cos(a/2);self.dynamic.sendTransform(tf)

def main(args=None):spin(StateEstimator,args)
