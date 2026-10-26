"""IMU-only body yaw, SLAM XY, separate TF and original final-B FBX markers."""
from collections import deque
import json
import math
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker,MarkerArray
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException
from .core import YawTrack,Alignment,vehicle_poses,validate_geometry,yaw_quat,rpy_quat,quat_mul,rot2,wrap


def stamp_ns(stamp):return stamp.sec*10**9+stamp.nanosec
def q_tuple(q):return (q.x,q.y,q.z,q.w)


class VehicleViz(Node):
    def __init__(self, **kwargs):
        super().__init__('tractor_trailer_visualizer',**kwargs)
        share=Path(get_package_share_directory('tractor_trailer_rviz'))
        self.declare_parameter('config_file',str(share/'config/live.yaml'))
        self.declare_parameter('tractor_topic','')
        self.declare_parameter('trailer_topic','')
        self.declare_parameter('slam_frame','')
        self.declare_parameter('output_directory','')
        self.cfg=yaml.safe_load(Path(self.get_parameter('config_file').value).read_text())
        c=self.cfg
        if c['mode'] not in ('LIVE','TEST'):raise ValueError('explicit LIVE or TEST mode required')
        self.test=c['mode']=='TEST'
        if not c['namespace'].startswith('/vehicle_viz') or not c['tf_prefix'].startswith('vehicle_viz_'):
            raise ValueError('separate visualization namespace/frame prefix required')
        if self.test and (not c['namespace'].startswith('/vehicle_viz_test') or not c['fixed_frame'].startswith('vehicle_viz_test_')):
            raise ValueError('TEST must use isolated topics and frames')
        if not self.test and ('test' in c['namespace'] or c['fixed_frame']!='map'):
            raise ValueError('LIVE uses actual map and dedicated real sensor topics')
        self.frame=c['fixed_frame']
        self.slam_frame=self.get_parameter('slam_frame').value or c['slam_frame']
        if self.slam_frame.startswith(c['tf_prefix']) and not self.test:
            raise ValueError('visualization TF cannot be used as SLAM pose source')
        self.ns=c['namespace']
        self.design=json.loads((share/'config/design.json').read_text())
        self.share=share
        self.geom=c['geometry']
        self.geometry_error=''
        try:
            validate_geometry(self.geom)
            if self.geom.get('confirmed') is not True:raise ValueError('real geometry not confirmed')
            if not self.test and self.geom.get('source')!='MEASURED_REAL_VEHICLE':
                raise ValueError('LIVE requires geometry source MEASURED_REAL_VEHICLE')
        except ValueError as exc:self.geometry_error=str(exc)
        self.tracks={}
        self.source_fault={r:'' for r in ('tractor','trailer')}
        for role in ('tractor','trailer'):
            s=c[role]
            self.tracks[role]=YawTrack(s['source'],s['sign'],s['sensor_to_body_rpy'],s['gyro_bias_rad_s'],c['imu_timeout_s'])
            topic=self.get_parameter(role+'_topic').value or s['topic']
            if not self.test and 'vehicle_viz_test' in topic:raise ValueError('LIVE cannot use TEST source topics')
            self.create_subscription(Imu,topic,lambda m,r=role:self.on_imu(r,m),qos_profile_sensor_data)
            if not self.test:
                self.create_subscription(DiagnosticArray,'/vehicle_viz/'+role+'/diagnostics',
                                         lambda m,r=role:self.on_diagnostic(r,m),10)
        self.align=Alignment()
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.tf=TransformBroadcaster(self) if c.get('publish_tf',True) else None
        self.sent=set()
        self.sent_order=deque()
        self.tf_conflict=''
        self.create_subscription(TFMessage,'/tf',self.audit_tf,100)
        self.create_subscription(TFMessage,'/tf_static',self.audit_tf,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.markers=self.create_publisher(MarkerArray,self.ns+'/model',1)
        self.status_pub=self.create_publisher(String,self.ns+'/status',10)
        self.create_service(SetBool,self.ns+'/align_straight',self.align_service)
        self.last_pose=None
        self.last_tf_stamp=None
        self.last_slam_stamp=None
        self.last_slam_advance=None
        self.log=None
        output=self.get_parameter('output_directory').value
        if output:
            p=Path(output);p.mkdir(parents=True,exist_ok=False)
            self.log=(p/'status.jsonl').open('x')
        self.last_reason=None
        self.create_timer(.05,self.tick)

    def on_imu(self,role,m):
        t=self.tracks[role]
        if m.header.frame_id!=self.cfg[role]['frame']:
            t.reject('unexpected sensor frame: '+m.header.frame_id)
            return
        q=q_tuple(m.orientation) if m.orientation_covariance[0]!=-1 else None
        g=(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z) if m.angular_velocity_covariance[0]!=-1 else None
        t.push(stamp_ns(m.header.stamp),time.monotonic(),self.get_clock().now().nanoseconds,q=q,gyro=g)

    def on_diagnostic(self,role,msg):
        for d in msg.status:
            if d.name!='/vehicle_viz/'+role+'/UART':continue
            if d.message.startswith('INVALID'):
                if not self.source_fault[role]:self.tracks[role].reject('UART source invalid: '+d.message)
                self.source_fault[role]=d.message
            elif d.message.startswith('LIVE'):
                self.source_fault[role]=''

    def get_slam(self):
        transform=self.buffer.lookup_transform(self.frame,self.slam_frame,Time())
        n=stamp_ns(transform.header.stamp)
        now=self.get_clock().now().nanoseconds
        wall=time.monotonic()
        if self.last_slam_stamp is not None and n<self.last_slam_stamp:
            self.align.anchor=None
            raise ValueError('SLAM time reversed; alignment reset')
        if n!=self.last_slam_stamp:
            self.last_slam_stamp=n;self.last_slam_advance=wall
        if n<=0 or not -.05<=(now-n)/1e9<=self.cfg['pose_timeout_s'] or wall-self.last_slam_advance>self.cfg['pose_timeout_s']:
            raise ValueError('SLAM pose stale or not dynamic')
        return transform

    def coherent(self):
        a,b=self.tracks['tractor'],self.tracks['trailer']
        now,wall=self.get_clock().now().nanoseconds,time.monotonic()
        if any(self.source_fault.values()) or any(t.state(wall,now)!='LIVE' for t in (a,b)):
            raise ValueError('both IMUs must be LIVE')
        if abs(a.last_stamp-b.last_stamp)/1e9>self.cfg['max_pair_skew_s']:
            raise ValueError('IMU latest timestamps exceed pair skew limit')
        latest=self.get_slam()
        common=min(a.last_stamp,b.last_stamp,stamp_ns(latest.header.stamp))
        if (now-common)/1e9>min(self.cfg['pose_timeout_s'],self.cfg['imu_timeout_s']):
            raise ValueError('common timestamp stale')
        a.at(common);b.at(common)
        # Full TF interpolation at the SAME timestamp used for both yaws.
        pose=self.buffer.lookup_transform(self.frame,self.slam_frame,Time(nanoseconds=common))
        return common,pose

    def alignment_requirements(self):
        if self.geometry_error:raise ValueError(self.geometry_error)
        for role in ('tractor','trailer'):
            s=self.cfg[role]
            if s.get('mounting_and_sign_confirmed') is not True:
                raise ValueError(role+' mounting, vertical axis and sign unverified')
            if s['source']=='gyro_z' and s.get('gyro_bias_confirmed') is not True:
                raise ValueError(role+' gyro integration requires confirmed stationary bias')
        if self.tf_conflict:raise ValueError(self.tf_conflict)

    def align_service(self,req,res):
        if not req.data:
            self.align.anchor=None
            self.last_pose=None
            res.success=True;res.message='Alignment cleared; no sensor calibration or settings changed'
            return res
        try:
            self.alignment_requirements()
            common,p=self.coherent()
            a,b=self.tracks['tractor'],self.tracks['trailer']
            # Software stationarity check complements operator's straight confirmation.
            before=common-300_000_000
            if any(abs(t.at(common)-t.at(before))>math.radians(1.) for t in (a,b)):
                raise ValueError('IMUs moving; hold stationary before alignment')
            old=self.buffer.lookup_transform(self.frame,self.slam_frame,Time(nanoseconds=before))
            if math.hypot(p.transform.translation.x-old.transform.translation.x,p.transform.translation.y-old.transform.translation.y)>.01:
                raise ValueError('SLAM position moving; hold stationary')
            self.align.align(a,b,common,yaw_quat(q_tuple(p.transform.rotation))+self.geom['slam_to_body_yaw_rad'])
            res.success=True
            res.message='Straight alignment accepted at '+str(common)+'; later body yaw increments come ONLY from each IMU'
        except (ValueError,TransformException) as exc:
            res.success=False;res.message=str(exc)
        self.get_logger().info(res.message)
        return res

    def marker(self,ident,kind,stamp):
        m=Marker();m.header.frame_id=self.frame
        m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9)
        m.ns=self.ns+('/TEST_SIMULATED' if self.test else '/REAL_INPUT')
        m.id,m.type,m.action=ident,kind,Marker.ADD
        m.pose.orientation.w=1.;m.color.a=1.
        m.lifetime.sec=1
        return m

    def draw(self,record,stamp):
        ms=[]
        pose=self.last_pose
        if pose:
            fresh=record['valid']
            for i,part in enumerate(self.design['parts']):
                role=part['role'];yaw=pose[role+'_yaw'];origin=pose['poses'][role]
                offset=rot2(yaw,part['xyz'][:2])
                m=self.marker(i,Marker.MESH_RESOURCE,stamp)
                m.mesh_resource=(self.share/'models/tractor-trailer-kidbi'/part['mesh']).as_uri()
                m.mesh_use_embedded_materials=fresh
                m.scale.x=m.scale.y=m.scale.z=part['scale']
                m.pose.position.x,m.pose.position.y=origin[0]+offset[0],origin[1]+offset[1]
                m.pose.position.z=part['xyz'][2]
                q=quat_mul(rpy_quat(0,0,yaw),rpy_quat(*part['rpy']))
                m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
                # RViz Humble: all-zero RGBA preserves embedded material without
                # adding the extra tint pass. White RGBA washes the mesh colors out.
                m.color.r=m.color.g=m.color.b=0. if fresh else .45
                m.color.a=0. if fresh else .45
                ms.append(m)
        else:
            for i,_ in enumerate(self.design['parts']):
                m=self.marker(i,Marker.MESH_RESOURCE,stamp);m.action=Marker.DELETE;ms.append(m)
        state=record['imu']
        values=record.get('values')
        lines=['TEST/SIMULATED' if self.test else 'TRACTOR/TRAILER_REAL_INPUT',
               ('Tractor_delta:%+.1fdeg'%math.degrees(values['tractor_delta'])) if values else 'Tractor_delta:--',
               ('Trailer_delta:%+.1fdeg'%math.degrees(values['trailer_delta'])) if values else 'Trailer_delta:--',
               ('Articulation:%+.1fdeg'%math.degrees(values['articulation'])) if values else 'Articulation:UNAVAILABLE',
               'Tractor_IMU:'+state['tractor'],'Trailer_IMU:'+state['trailer'],
               'SLAM_Pose:'+record['slam'],record['reason'][:45]]
        if pose and not record['valid']:lines.append('LAST_MODEL_POSE_NOT_LIVE')
        x,y=pose['poses']['tractor'] if pose else (10.,0.)
        for i,line in enumerate(lines):
            m=self.marker(100+i,Marker.TEXT_VIEW_FACING,stamp)
            m.pose.position.x,m.pose.position.y,m.pose.position.z=x+.3,y-1.25-i*.24,1.7
            m.scale.z=.17
            m.color.r,m.color.g,m.color.b=(.2,1.,.3) if record['valid'] else (1.,.25,.15)
            m.text=line.replace(' ','_') if line else 'OK'
            ms.append(m)
        # Remove the stale-only last line when live again.
        if len(lines)==8:
            m=self.marker(108,Marker.TEXT_VIEW_FACING,stamp);m.action=Marker.DELETE;ms.append(m)
        self.markers.publish(MarkerArray(markers=ms))

    @staticmethod
    def signature(t):
        v,q=t.transform.translation,t.transform.rotation
        return (t.header.frame_id,t.child_frame_id,stamp_ns(t.header.stamp),v.x,v.y,v.z,q.x,q.y,q.z,q.w)

    def audit_tf(self,msg):
        for t in msg.transforms:
            if t.child_frame_id.startswith(self.cfg['tf_prefix']) and t.child_frame_id not in (self.slam_frame,self.cfg['tractor']['frame'],self.cfg['trailer']['frame']):
                if self.signature(t) not in self.sent:
                    self.tf_conflict='TF CONFLICT: visualization child already published externally'

    def publish_transforms(self,stamp,values,poses):
        if not self.tf or stamp==self.last_tf_stamp:return
        pre=self.cfg['tf_prefix']
        relative=values['trailer_yaw']-values['tractor_yaw']
        anchor=self.geom['trailer_origin_to_hitch_xy']
        offset=rot2(relative,[-anchor[0],-anchor[1]])
        edges=[(self.frame,pre+'tractor',poses['tractor'],values['tractor_yaw']),
               (pre+'tractor',pre+'hitch',self.geom['rear_axle_to_hitch_xy'],0.),
               (pre+'hitch',pre+'trailer',offset,relative),
               (pre+'trailer',pre+'trailer_axle',self.geom['trailer_origin_to_axle_xy'],0.)]
        out=[]
        for parent,child,xy,yaw in edges:
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
            t.header.stamp.sec,t.header.stamp.nanosec=divmod(stamp,10**9)
            t.transform.translation.x,t.transform.translation.y=map(float,xy)
            t.transform.rotation.z,t.transform.rotation.w=math.sin(yaw/2),math.cos(yaw/2)
            sig=self.signature(t);self.sent.add(sig);self.sent_order.append(sig);out.append(t)
        while len(self.sent_order)>4096:self.sent.discard(self.sent_order.popleft())
        self.tf.sendTransform(out);self.last_tf_stamp=stamp

    def tick(self):
        now=self.get_clock().now().nanoseconds;wall=time.monotonic()
        record=dict(mode='TEST/SIMULATED' if self.test else 'LIVE_HARDWARE_INPUT',stamp_ns=now,
                    valid=False,imu={r:'INVALID' if self.source_fault[r] else t.state(wall,now) for r,t in self.tracks.items()},
                    imu_age_s={r:None if t.last_wall is None else wall-t.last_wall for r,t in self.tracks.items()},
                    imu_rejected={r:t.rejected for r,t in self.tracks.items()},
                    imu_reason={r:t.error for r,t in self.tracks.items()},
                    yaw_source={r:t.source for r,t in self.tracks.items()},
                    slam='NO DATA',reason='',values=None)
        try:
            self.get_slam();record['slam']='LIVE'
        except (ValueError,TransformException):
            record['slam']='STALE' if self.last_slam_stamp is not None else 'NO DATA'
        try:
            self.alignment_requirements()
            common,p=self.coherent()
            values=self.align.evaluate(self.tracks['tractor'],self.tracks['trailer'],common)
            xy=p.transform.translation
            poses=vehicle_poses((xy.x,xy.y),yaw_quat(q_tuple(p.transform.rotation)),values['tractor_yaw'],values['trailer_yaw'],self.geom)
            self.last_pose=dict(**values,poses=poses)
            record.update(valid=True,values=values,poses=poses,paired_stamp_ns=common,
                          paired_age_ms=(now-common)/1e6,
                          latest_imu_skew_ms=abs(self.tracks['tractor'].last_stamp-self.tracks['trailer'].last_stamp)/1e6)
            self.publish_transforms(common,values,poses)
            record['reason']='TEST/SIMULATED' if self.test else 'LIVE: independent IMU yaw; geometry confirmed'
        except (ValueError,TransformException) as exc:record['reason']=str(exc)
        self.draw(record,now)
        text=json.dumps(record,allow_nan=False)
        self.status_pub.publish(String(data=text))
        if self.log:self.log.write(text+'\n');self.log.flush()
        if record['reason']!=self.last_reason:
            self.get_logger().info(record['reason']);self.last_reason=record['reason']

    def destroy_node(self):
        if self.log:self.log.close()
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(VehicleViz,args)
