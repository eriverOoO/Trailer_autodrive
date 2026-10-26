"""Read-only SLAM and independent IMU inputs; only final-B 2D Marker outputs."""
import json
import math
from pathlib import Path
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer,TransformListener,TransformException
from visualization_msgs.msg import MarkerArray
from .core import YawTrack,Alignment,vehicle_poses,validate_geometry,yaw_quat
from .render_adapter import Renderer,unavailable


def stamp_ns(s):return s.sec*10**9+s.nanosec
def quaternion(q):return (q.x,q.y,q.z,q.w)


class Vehicle2D(Node):
    def __init__(self,**kwargs):
        super().__init__('tractor_trailer_2d_visualizer',**kwargs)
        share=Path(get_package_share_directory('tractor_trailer_rviz_2d'))
        for k,v in dict(config_file=str(share/'config/live.yaml'),tractor_topic='',trailer_topic='',slam_frame='',output_directory='').items():
            self.declare_parameter(k,v)
        c=self.cfg=yaml.safe_load(Path(self.get_parameter('config_file').value).read_text())
        self.test=c['mode']=='TEST'
        if c['mode'] not in ('LIVE','TEST') or not c['namespace'].startswith('/vehicle_viz_2d'):
            raise ValueError('explicit mode and separate 2D namespace required')
        if self.test and (not c['namespace'].startswith('/vehicle_viz_2d_test') or not c['fixed_frame'].startswith('vehicle_viz_2d_test_')):
            raise ValueError('TEST requires isolated topics/frames')
        if not self.test and ('test' in c['namespace'] or c['fixed_frame']!='map'):
            raise ValueError('LIVE requires map and real input namespace')
        self.ns,self.frame=c['namespace'],c['fixed_frame']
        self.slam_frame=self.get_parameter('slam_frame').value or c['slam_frame']
        self.geom=c['geometry'];self.geometry_error=''
        try:
            validate_geometry(self.geom)
            if self.geom['confirmed'] is not True:raise ValueError('geometry unverified')
            if not self.test and self.geom['source']!='MEASURED_REAL_VEHICLE':
                raise ValueError('LIVE requires measured real vehicle geometry')
        except ValueError as exc:self.geometry_error=str(exc)
        self.tracks={};self.source_fault={r:'' for r in ('tractor','trailer')}
        for role in ('tractor','trailer'):
            s=c[role]
            self.tracks[role]=YawTrack(s['source'],s['sign'],s['sensor_to_body_rpy'],s['gyro_bias_rad_s'],c['imu_timeout_s'])
            topic=self.get_parameter(role+'_topic').value or s['topic']
            if not self.test and any(prefix in topic for prefix in ('/vehicle_viz_2d_test/','/vehicle_viz_test/')):
                raise ValueError('LIVE cannot use TEST topics')
            self.create_subscription(Imu,topic,lambda m,r=role:self.on_imu(r,m),qos_profile_sensor_data)
            if not self.test:
                self.create_subscription(DiagnosticArray,'/vehicle_viz_2d/'+role+'/diagnostics',lambda m,r=role:self.on_diag(r,m),10)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(MarkerArray,self.ns+'/model',
                                      QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status=self.create_publisher(String,self.ns+'/status',10)
        self.align=Alignment()
        self.create_service(SetBool,self.ns+'/align_straight',self.on_align)
        self.create_subscription(String,c['state_topic'],self.on_state,10)
        design=json.loads((share/'models/reference_footprints.json').read_text())
        self.renderer=Renderer(self,design['polygons'],self.geom['rear_axle_to_hitch_xy'])
        self.state_name=None;self.state_stamp=None;self.state_wall=None
        self.last_slam_stamp=self.last_slam_advance=None
        self.last_model=None;self.last_xy=(10.,0.);self.last_reason=None
        self.log=None
        directory=self.get_parameter('output_directory').value
        if directory:
            p=Path(directory);p.mkdir(parents=True,exist_ok=False);self.log=(p/'status.jsonl').open('x')
        self.create_timer(.05,self.tick)

    def on_imu(self,role,m):
        t=self.tracks[role]
        if m.header.frame_id!=self.cfg[role]['frame']:
            t.reject('unexpected sensor frame '+m.header.frame_id);return
        q=quaternion(m.orientation) if m.orientation_covariance[0]!=-1 else None
        g=(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z) if m.angular_velocity_covariance[0]!=-1 else None
        t.push(stamp_ns(m.header.stamp),time.monotonic(),self.get_clock().now().nanoseconds,q=q,gyro=g)

    def on_diag(self,role,m):
        for d in m.status:
            if d.name!='/vehicle_viz_2d/'+role+'/UART':continue
            if d.message.startswith('INVALID'):
                if not self.source_fault[role]:self.tracks[role].reject(d.message)
                self.source_fault[role]=d.message
            elif d.message.startswith('LIVE'):self.source_fault[role]=''

    def on_state(self,m):
        try:
            d=json.loads(m.data);state=d['state'];stamp=d['stamp_ns']
            age=(self.get_clock().now().nanoseconds-stamp)/1e9
            if (not isinstance(state,str) or not state or len(state)>64
                    or not isinstance(stamp,int) or not -.05<=age<=self.cfg['state_timeout_s']):
                raise ValueError('invalid state machine status')
            self.state_name,self.state_stamp,self.state_wall=state,stamp,time.monotonic()
        except (ValueError,KeyError,TypeError):
            self.state_name=None

    def stage(self):
        if (self.state_name is not None and time.monotonic()-self.state_wall<=self.cfg['state_timeout_s']
                and -.05<=(self.get_clock().now().nanoseconds-self.state_stamp)/1e9<=self.cfg['state_timeout_s']):
            return self.state_name
        return 'STATE_UNKNOWN'

    def get_slam(self):
        p=self.buffer.lookup_transform(self.frame,self.slam_frame,Time())
        n=stamp_ns(p.header.stamp);now=self.get_clock().now().nanoseconds;wall=time.monotonic()
        if self.last_slam_stamp is not None and n<self.last_slam_stamp:
            self.align.anchor=None;raise ValueError('SLAM time reversed; realign')
        if n!=self.last_slam_stamp:self.last_slam_stamp=n;self.last_slam_advance=wall
        if n<=0 or not -.05<=(now-n)/1e9<=self.cfg['pose_timeout_s'] or wall-self.last_slam_advance>self.cfg['pose_timeout_s']:
            raise ValueError('SLAM pose stale')
        if not all(math.isfinite(v) for v in (p.transform.translation.x,p.transform.translation.y)):
            raise ValueError('SLAM position invalid')
        yaw_quat(quaternion(p.transform.rotation))
        return p

    def coherent(self):
        now,wall=self.get_clock().now().nanoseconds,time.monotonic()
        a,b=self.tracks['tractor'],self.tracks['trailer']
        if any(self.source_fault.values()) or any(t.state(wall,now)!='LIVE' for t in (a,b)):
            raise ValueError('both IMUs must be LIVE')
        if abs(a.last_stamp-b.last_stamp)/1e9>self.cfg['max_pair_skew_s']:
            raise ValueError('IMU timestamps not paired')
        p=self.get_slam()
        common=min(a.last_stamp,b.last_stamp,stamp_ns(p.header.stamp))
        if (now-common)/1e9>min(self.cfg['imu_timeout_s'],self.cfg['pose_timeout_s']):
            raise ValueError('paired sample stale')
        a.at(common);b.at(common)
        p=self.buffer.lookup_transform(self.frame,self.slam_frame,Time(nanoseconds=common))
        if not all(math.isfinite(v) for v in (p.transform.translation.x,p.transform.translation.y)):
            raise ValueError('paired SLAM position invalid')
        yaw_quat(quaternion(p.transform.rotation))
        return common,p

    def require_alignment(self):
        if self.geometry_error:raise ValueError(self.geometry_error)
        for role in ('tractor','trailer'):
            s=self.cfg[role]
            if s['mounting_and_sign_confirmed'] is not True:
                raise ValueError(role+' mounting/sign unverified')
            if s['source']=='gyro_z' and s['gyro_bias_confirmed'] is not True:
                raise ValueError(role+' gyro bias unverified')

    def on_align(self,req,res):
        if not req.data:
            self.align.anchor=None;self.last_model=None;self.renderer.last_viz=0.
            res.success=True;res.message='alignment cleared'
            return res
        try:
            self.require_alignment();common,p=self.coherent();before=common-300_000_000
            a,b=self.tracks['tractor'],self.tracks['trailer']
            if any(abs(t.at(common)-t.at(before))>math.radians(1.) for t in (a,b)):
                raise ValueError('hold both sensors stationary')
            old=self.buffer.lookup_transform(self.frame,self.slam_frame,Time(nanoseconds=before))
            if math.hypot(p.transform.translation.x-old.transform.translation.x,p.transform.translation.y-old.transform.translation.y)>.01:
                raise ValueError('SLAM position moving')
            heading=yaw_quat(quaternion(p.transform.rotation))+self.geom['slam_to_body_yaw_rad']
            self.align.align(a,b,common,heading)
            self.renderer.last_viz=0.
            res.success=True;res.message='operator-confirmed straight reference recorded; no sensor settings changed'
        except (ValueError,TransformException) as exc:
            res.success=False;res.message=str(exc)
        return res

    def tick(self):
        now,wall=self.get_clock().now().nanoseconds,time.monotonic()
        info=dict(mode='TEST/SIMULATED' if self.test else 'REAL_INPUT',stamp_ns=now,valid=False,
                  imu={r:'INVALID' if self.source_fault[r] else t.state(wall,now) for r,t in self.tracks.items()},
                  imu_age_s={r:None if t.last_wall is None else wall-t.last_wall for r,t in self.tracks.items()},
                  imu_reason={r:t.error for r,t in self.tracks.items()},
                  yaw_source={r:t.source for r,t in self.tracks.items()},
                  slam='NO DATA',reason='',values=None,state_machine=self.stage(),tf_published=False)
        try:self.get_slam();info['slam']='LIVE'
        except (ValueError,TransformException):info['slam']='STALE' if self.last_slam_stamp is not None else 'NO DATA'
        batch=None
        try:
            self.require_alignment();common,p=self.coherent()
            values=self.align.evaluate(self.tracks['tractor'],self.tracks['trailer'],common)
            xy=p.transform.translation
            poses=vehicle_poses((xy.x,xy.y),yaw_quat(quaternion(p.transform.rotation)),values['tractor_yaw'],values['trailer_yaw'],self.geom)
            state=[*poses['tractor'],values['tractor_yaw'],values['trailer_yaw']]
            stage=('TEST/SIMULATED ' if self.test else '')+self.stage()
            if self.renderer.stage!=stage:self.renderer.last_viz=0.
            batch=self.renderer.render(state,stage,self.frame)
            self.last_model=batch;self.last_xy=poses['tractor']
            info.update(valid=True,values=values,poses=poses,paired_stamp_ns=common,
                        paired_age_ms=(now-common)/1e6,reason='OK',marker_update_period_s=.4)
        except (ValueError,TransformException) as exc:
            info['reason']=str(exc)
            brief='T:'+info['imu']['tractor'].replace(' ','_')+' R:'+info['imu']['trailer'].replace(' ','_')+' SLAM:'+info['slam'].replace(' ','_')
            if all(s=='LIVE' for s in info['imu'].values()) and info['slam']=='LIVE':brief='UNALIGNED_OR_GEOMETRY_UNVERIFIED'
            batch=unavailable(self,self.frame,brief,self.last_xy,self.last_model,self.test)
        if batch:self.pub.publish(batch)
        text=json.dumps(info,allow_nan=False);self.status.publish(String(data=text))
        if self.log:self.log.write(text+'\n');self.log.flush()
        if info['reason']!=self.last_reason:self.get_logger().info(info['reason']);self.last_reason=info['reason']

    def destroy_node(self):
        if self.log:self.log.close()
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(Vehicle2D,args)
