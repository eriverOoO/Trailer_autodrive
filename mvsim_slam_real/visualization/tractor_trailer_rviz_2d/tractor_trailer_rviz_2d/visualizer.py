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
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String,Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer,TransformListener,TransformException
from visualization_msgs.msg import MarkerArray
from .core import YawTrack,Alignment,StationaryReference,vehicle_poses,validate_geometry,yaw_quat,rot2
from .render_adapter import Renderer,unavailable
from .dimensions import default_geometry_file,sized_polygons


def stamp_ns(s):return s.sec*10**9+s.nanosec
def quaternion(q):return (q.x,q.y,q.z,q.w)


class Vehicle2D(Node):
    def __init__(self,**kwargs):
        super().__init__('tractor_trailer_2d_visualizer',**kwargs)
        share=Path(get_package_share_directory('tractor_trailer_rviz_2d'))
        for k,v in dict(config_file=str(share/'config/live.yaml'),geometry_file='',tractor_topic='',trailer_topic='',slam_frame='',output_directory='',visualization_mode='').items():
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
        startup=c.get('startup_reference',{})
        self.auto_reference=startup.get('enabled',False)
        self.show_straight_startup=startup.get('show_straight_preview',False)
        self.relative_only=c.get('relative_heading_only',False)
        if self.auto_reference and not self.relative_only:
            raise ValueError('automatic startup requires relative_heading_only')
        self.reference=StationaryReference(duration=startup.get('duration_s',5.))
        self.reference_locked=False
        self.reference_epochs=None
        self.gyro_limit=math.radians(startup.get('stationary_gyro_deg_s',2.))
        if not math.isfinite(self.gyro_limit) or self.gyro_limit<=0:
            raise ValueError('invalid stationary gyro threshold')
        self.gyro_norms={r:None for r in ('tractor','trailer')}
        self.scan_wall=self.scan_stamp=self.scan_valid=None
        self.scan_times=[]
        self.map_wall=None
        self.map_valid=False
        if self.auto_reference:
            self.create_subscription(LaserScan,c.get('scan_topic','/scan'),self.on_scan,qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid,'/map',self.on_map,qos_profile_sensor_data)
        self.visualization_mode=(self.get_parameter('visualization_mode').value
                                 or c.get('visualization_mode','FULL_VEHICLE'))
        if self.visualization_mode not in ('FULL_VEHICLE','TRACTOR_ONLY_DIAGNOSTIC'):
            raise ValueError('unknown visualization_mode')
        self.diagnostic_anchor=None
        self.slam_frame=self.get_parameter('slam_frame').value or c['slam_frame']
        geometry_file=self.get_parameter('geometry_file').value or default_geometry_file()
        if c.get('geometry_from_vehicle_file',False):
            if self.slam_frame!='base_footprint':
                raise ValueError('measured shared geometry requires base_footprint SLAM')
            g=yaml.safe_load(Path(geometry_file).read_text())
            c['geometry']={
                'confirmed':True,'source':'MEASURED_REAL_VEHICLE',
                'slam_to_rear_axle_xy':g['tractor_rear_axle_position'][:2],
                'slam_to_body_yaw_rad':0.,
                'rear_axle_to_hitch_xy':[g['hitch_position'][i]-g['tractor_rear_axle_position'][i] for i in (0,1)],
                'trailer_origin_to_hitch_xy':[0.,0.],
                'trailer_origin_to_axle_xy':[-g['hitch_to_axle_distance'],0.],
            }
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
            rate_limit=math.radians(startup.get('max_yaw_rate_deg_s',120.)) if self.auto_reference else None
            self.tracks[role]=YawTrack(s['source'],s['sign'],s['sensor_to_body_rpy'],s['gyro_bias_rad_s'],
                                      c['imu_timeout_s'],max_yaw_rate=rate_limit)
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
        self.ready_pub=self.create_publisher(Bool,self.ns+'/ready',1)
        self.align=Alignment()
        self.create_service(SetBool,self.ns+'/align_straight',self.on_align)
        self.create_service(SetBool,self.ns+'/reset_diagnostic_reference',self.on_diagnostic_reset)
        self.create_subscription(String,c['state_topic'],self.on_state,10)
        design=json.loads((share/'models/reference_footprints.json').read_text())
        geometry_file=self.get_parameter('geometry_file').value or default_geometry_file()
        polygons=sized_polygons(design['polygons'],geometry_file)
        if c.get('use_measured_polygon_origins',False):
            measured=yaml.safe_load(Path(geometry_file).read_text())
            centres=(-measured['tractor_rear_axle_position'][0],
                     -(measured['trailer_length']/2+measured['trailer_front_to_hitch_distance']))
            # Preserve the existing outline style while placing it at measured axle/hinge origins.
            for poly,centre in zip(polygons,centres):
                poly[:,0]+=centre-(poly[:,0].min()+poly[:,0].max())/2
                poly[:,1]-=(poly[:,1].min()+poly[:,1].max())/2
        self.renderer=Renderer(self,polygons,self.geom['rear_axle_to_hitch_xy'])
        self.get_logger().info(f'Display dimensions loaded from {geometry_file}: '
                               f'tractor {tuple(polygons[0].max(axis=0)-polygons[0].min(axis=0))} m, '
                               f'trailer {tuple(polygons[1].max(axis=0)-polygons[1].min(axis=0))} m')
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
        if self.auto_reference:
            self.gyro_norms[role]=(math.sqrt(sum(v*v for v in g)) if g is not None
                                   and all(math.isfinite(v) for v in g) else None)
        t.push(stamp_ns(m.header.stamp),time.monotonic(),self.get_clock().now().nanoseconds,q=q,gyro=g)

    def on_scan(self,m):
        now=self.get_clock().now().nanoseconds;stamp=stamp_ns(m.header.stamp)
        self.scan_valid=(stamp>0 and -.05<=(now-stamp)/1e9<=.5
                         and (self.scan_stamp is None or stamp>self.scan_stamp)
                         and m.header.frame_id=='laser_frame' and len(m.ranges)>0
                         and m.angle_increment>0 and m.range_max>m.range_min>=0
                         and any(math.isfinite(v) and m.range_min<=v<=m.range_max for v in m.ranges))
        self.scan_stamp=stamp;self.scan_wall=time.monotonic()
        self.scan_times.append(self.scan_wall)
        self.scan_times=[v for v in self.scan_times if self.scan_wall-v<=1.]

    def on_map(self,m):
        self.map_wall=time.monotonic()
        self.map_valid=(m.header.frame_id==self.frame and m.info.resolution>0
                        and len(m.data)==m.info.width*m.info.height and any(v>=0 for v in m.data))

    def automatic_alignment(self,common,p,wall):
        now=self.get_clock().now().nanoseconds
        if (not self.scan_valid or self.scan_wall is None or wall-self.scan_wall>.5
                or self.scan_stamp is None or not -.05<=(now-self.scan_stamp)/1e9<=.5
                or len(self.scan_times)<5):
            raise ValueError('waiting for fresh LiDAR scans')
        if not self.map_valid or self.map_wall is None or wall-self.map_wall>3.:
            raise ValueError('waiting for fresh SLAM map')
        self.buffer.lookup_transform(self.frame,'laser_frame',Time(nanoseconds=self.scan_stamp))
        a,b=self.tracks['tractor'],self.tracks['trailer']
        if self.reference_locked:
            if (a.epoch,b.epoch)!=self.reference_epochs:
                raise ValueError('REFERENCE LOST: stop, align straight and reset reference')
            return
        if self.reference_epochs is not None and self.reference_epochs!=(a.epoch,b.epoch):
            self.reference.clear('IMU discontinuity during startup')
        self.reference_epochs=(a.epoch,b.epoch)
        stationary=all(v is not None and v<=self.gyro_limit for v in self.gyro_norms.values())
        xy=p.transform.translation
        initial=self.reference.push(common,wall,a.at(common),b.at(common),xy.x,xy.y,
                                    yaw_quat(quaternion(p.transform.rotation)),stationary)
        if initial is None:raise ValueError(self.reference.reason)
        _,_,heading=initial
        # The stable window authorizes the reference; its final paired sample
        # is exactly zero for both bodies on the first READY frame.
        self.align.align(a,b,common,heading+self.geom['slam_to_body_yaw_rad'])
        self.reference_locked=True
        self.renderer.last_viz=0.

    def straight_startup_preview(self, reason):
        """Assumed straight startup icon only; never publishes pose/TF/READY."""
        axle=self.geom['slam_to_rear_axle_xy']
        batch=self.renderer.render([*axle,0.,0.], 'START STRAIGHT / HOLD STILL', self.frame)
        if batch is not None:
            for marker in batch.markers:
                if marker.ns in ('current_body','trailer_pose_arrow'):
                    marker.color.r=marker.color.g=marker.color.b=.65
                if marker.ns=='articulation':
                    marker.text='START STRAIGHT / HOLD STILL\n'+reason
        return batch

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
        if getattr(self,'relative_only',False):
            if any(self.cfg[r]['source']!='orientation' for r in ('tractor','trailer')):
                raise ValueError('relative startup uses orientation deltas, not gyro integration')
            return
        for role in ('tractor','trailer'):
            s=self.cfg[role]
            if s['mounting_and_sign_confirmed'] is not True:
                raise ValueError(role+' mounting/sign unverified')
            if s['source']=='gyro_z' and s['gyro_bias_confirmed'] is not True:
                raise ValueError(role+' gyro bias unverified')

    def on_align(self,req,res):
        if not req.data:
            self.align.anchor=None;self.last_model=None;self.renderer.last_viz=0.
            if self.auto_reference:
                self.reference_locked=False;self.reference_epochs=None
                self.reference.clear('new straight startup reference requested')
            res.success=True;res.message='alignment cleared'
            return res
        if self.auto_reference:
            res.success=False
            res.message='automatic stable startup enabled; data:false clears reference after straight alignment'
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

    def on_diagnostic_reset(self,req,res):
        if self.visualization_mode!='TRACTOR_ONLY_DIAGNOSTIC':
            res.success=False;res.message='diagnostic mode only; full alignment unchanged'
        else:
            self.diagnostic_anchor=None;self.last_model=None;self.renderer.last_viz=0.
            res.success=True
            res.message='diagnostic first-sample reference cleared; no physical alignment claimed'
        return res

    def tractor_diagnostic(self,info,now,wall):
        """A display-only icon; full vehicle validity remains false."""
        a=self.tracks['tractor']
        if self.source_fault['tractor'] or a.state(wall,now)!='LIVE':
            raise ValueError('tractor IMU must be LIVE')
        if a.source!='orientation':
            raise ValueError('tractor diagnostic requires measured orientation')
        p=self.get_slam();common=min(a.last_stamp,stamp_ns(p.header.stamp))
        if (now-common)/1e9>min(self.cfg['imu_timeout_s'],self.cfg['pose_timeout_s']):
            raise ValueError('tractor/SLAM sample stale')
        native=a.at(common)
        p=self.buffer.lookup_transform(self.frame,self.slam_frame,Time(nanoseconds=common))
        slam_yaw=yaw_quat(quaternion(p.transform.rotation))
        cfg=self.cfg.get('tractor_diagnostic',{})
        measured=cfg.get('extrinsics_confirmed',False) is True
        offset=cfg.get('slam_to_rear_axle_xy');mount_yaw=cfg.get('slam_to_body_yaw_rad')
        if measured:
            if (not isinstance(offset,list) or len(offset)!=2
                    or not all(type(v) in (int,float) and math.isfinite(v) for v in offset)
                    or type(mount_yaw) not in (int,float) or not math.isfinite(mount_yaw)):
                raise ValueError('confirmed tractor extrinsics need measured XY and yaw')
        elif cfg.get('allow_sensor_anchor') is not True:
            raise ValueError('measure tractor extrinsics or explicitly select sensor-anchor diagnostic')
        if self.diagnostic_anchor is None:
            self.diagnostic_anchor=(native,slam_yaw+(mount_yaw if measured else 0.),a.epoch,common)
        initial,heading,epoch,reference_stamp=self.diagnostic_anchor
        if epoch!=a.epoch:raise ValueError('DIAGNOSTIC REFERENCE LOST; reset diagnostic reference')
        delta=native-initial;display_yaw=heading+delta
        xy=(p.transform.translation.x,p.transform.translation.y)
        if measured:
            lever=rot2(slam_yaw,offset);xy=(xy[0]+lever[0],xy[1]+lever[1])
        if not all(math.isfinite(v) for v in (*xy,display_yaw)):
            raise ValueError('diagnostic pose invalid')
        mounting=self.cfg['tractor']['mounting_and_sign_confirmed'] is True
        position_reference='MEASURED_REAR_AXLE' if measured else 'UNMEASURED_SENSOR_ANCHOR'
        text='TRACTOR DIAG\nphi=N/A\n'+('REAR AXLE' if measured else 'SENSOR ANCHOR')
        if not measured or not mounting:text+='\nUNALIGNED'
        text+='\nTRAILER NO DATA' if info['imu']['trailer']=='NO DATA' else '\nTRAILER UNUSED'
        batch=self.renderer.render_tractor(xy,display_yaw,text,self.frame)
        self.last_model=batch;self.last_xy=xy
        info.update(valid=False,diagnostic_valid=True,vehicle_pose_valid=measured and mounting,
                    full_vehicle_valid=False,visualization_mode=self.visualization_mode,
                    position_reference=position_reference,position_source=self.slam_frame,
                    mounting_and_sign_confirmed=mounting,extrinsics_confirmed=measured,
                    heading_reference='INITIAL_SLAM_TRACKING_YAW_PLUS_NATIVE_IMU_DELTA',
                    heading_alignment_verified=measured and mounting,
                    display_anchor_xy=xy,display_yaw_rad=display_yaw,
                    native_imu_delta_rad=delta,reference_stamp_ns=reference_stamp,
                    values={'tractor_delta':delta,'tractor_yaw':display_yaw if measured and mounting else None,
                            'trailer_yaw':None,'phi':None},reason='DIAGNOSTIC_ONLY',
                    paired_stamp_ns=common,paired_age_ms=(now-common)/1e6)
        return batch

    def tick(self):
        now,wall=self.get_clock().now().nanoseconds,time.monotonic()
        info=dict(mode='TEST/SIMULATED' if self.test else 'REAL_INPUT',stamp_ns=now,valid=False,
                  visualization_mode=self.visualization_mode,diagnostic_valid=False,
                  imu={r:'INVALID' if self.source_fault[r] else t.state(wall,now) for r,t in self.tracks.items()},
                  imu_age_s={r:None if t.last_wall is None else wall-t.last_wall for r,t in self.tracks.items()},
                  imu_reason={r:t.error for r,t in self.tracks.items()},
                  yaw_source={r:t.source for r,t in self.tracks.items()},
                  slam='NO DATA',reason='',values=None,state_machine=self.stage(),tf_published=False)
        try:self.get_slam();info['slam']='LIVE'
        except (ValueError,TransformException):info['slam']='STALE' if self.last_slam_stamp is not None else 'NO DATA'
        batch=None
        try:
            if self.visualization_mode=='TRACTOR_ONLY_DIAGNOSTIC':
                batch=self.tractor_diagnostic(info,now,wall)
            else:
                self.require_alignment();common,p=self.coherent()
                if self.auto_reference:self.automatic_alignment(common,p,wall)
                values=self.align.evaluate(self.tracks['tractor'],self.tracks['trailer'],common)
                xy=p.transform.translation
                poses=vehicle_poses((xy.x,xy.y),yaw_quat(quaternion(p.transform.rotation)),values['tractor_yaw'],values['trailer_yaw'],self.geom)
                state=[*poses['tractor'],values['tractor_yaw'],values['trailer_yaw']]
                stage=('TEST/SIMULATED ' if self.test else '')+self.stage()
                if self.auto_reference:
                    stage=(f"READY T={math.degrees(values['tractor_delta']):+.1f} "
                           f"R={math.degrees(values['trailer_delta']):+.1f} deg")
                if self.renderer.stage!=stage:self.renderer.last_viz=0.
                batch=self.renderer.render(state,stage,self.frame)
                self.last_model=batch;self.last_xy=poses['tractor']
                info.update(valid=True,values=values,poses=poses,paired_stamp_ns=common,
                            paired_age_ms=(now-common)/1e6,reason='OK',marker_update_period_s=.4)
        except (ValueError,TransformException) as exc:
            if self.auto_reference and not self.reference_locked:
                # A normal collecting window is kept; all upstream failures restart it.
                if str(exc)!=self.reference.reason:self.reference.clear(str(exc))
            info['reason']=str(exc)
            brief='T:'+info['imu']['tractor'].replace(' ','_')+' R:'+info['imu']['trailer'].replace(' ','_')+' SLAM:'+info['slam'].replace(' ','_')
            if all(s=='LIVE' for s in info['imu'].values()) and info['slam']=='LIVE':brief='UNALIGNED_OR_GEOMETRY_UNVERIFIED'
            batch=unavailable(self,self.frame,brief,self.last_xy,self.last_model,self.test)
            if self.auto_reference:
                brief=('REFERENCE LOST / INPUT FAULT' if self.reference_locked else 'WAIT / HOLD STILL')
                if not self.reference_locked and self.show_straight_startup:
                    batch=self.straight_startup_preview(str(exc))
                    info['startup_pose_assumed']=True
                else:
                    batch=unavailable(self,self.frame,brief+' '+str(exc),self.last_xy,self.last_model,self.test)
        if self.auto_reference:
            info.update(ready=bool(info['valid'] and self.reference_locked),
                        reference_recorded=self.reference_locked,
                        startup_progress=self.reference.progress,
                        heading_reference='INDEPENDENT_STATIONARY_FINAL_SAMPLE_YAW_DELTAS',
                        heading_alignment_verified=False,
                        sign_convention='configured native-yaw sign -1 for both; physical direction needs confirmation',
                        control_authorized=False)
            if info['values'] is not None:
                info['degrees']={name:math.degrees(info['values'][name])
                                 for name in ('tractor_delta','trailer_delta','phi')}
        self.ready_pub.publish(Bool(data=bool(info.get('ready',False))))
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
