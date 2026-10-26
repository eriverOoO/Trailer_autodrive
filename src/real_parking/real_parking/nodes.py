import math,json,time,os
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from std_msgs.msg import String
from real_vehicle_bridge.geometry import VehicleGeometry
from real_vehicle_bridge.common import spin,publish_json,init_ros
from .bootstrap import load,install_b
load()
from tractor_trailer_model import Footprints
from tractor_parking_perception import planning_grid
from tractor_local_parking_gap_detector import observe_gap
from tractor_trailer_hybrid_astar import Path as RigPath
from tractor_parking_controller import Controller

def grid_from_message(m):
    from real_vehicle_bridge.estimator import yaw
    if m.info.width<=0 or m.info.height<=0 or len(m.data)!=m.info.width*m.info.height:raise ValueError('invalid map')
    o=m.info.origin;return planning_grid(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,[o.position.x,o.position.y,yaw(o.orientation)],simulation_clearance=True)

def finite_state(value):
    s=np.asarray(value,dtype=float)
    if s.shape!=(4,) or not np.isfinite(s).all():raise ValueError('state must be finite [rear_x,rear_y,tractor_yaw,trailer_yaw]')
    return s

def path_from_json(j):
    states=np.asarray(j['states'],dtype=float);gears=np.asarray(j['gears'],dtype=int);steering=np.asarray(j['steering'],dtype=float)
    if states.ndim!=2 or states.shape[1]!=4 or len(states)<2 or gears.shape!=(len(states),) or steering.shape!=(len(states),) or not np.isfinite(states).all() or not np.isfinite(steering).all() or not np.isin(gears,[-1,1]).all():raise ValueError('invalid planned 4D path')
    return RigPath(states,gears,steering)

def auto_parking_main(args=None):
    install_b()
    from tractor_trailer_model import Geometry
    from tractor_parallel_parking_auto import AutoParking
    # Original AutoParking calls Geometry.from_world(world_file). Adapt the input factory only.
    Geometry.from_world=classmethod(lambda cls,filename:VehicleGeometry(filename).model())
    init_ros(args);node=None
    try:
        node=AutoParking();directory=Path(node.get_parameter('blocked_debug_directory').value)
        if not str(node.get_parameter('blocked_debug_directory').value):raise ValueError('explicit diagnostic output directory required')
        directory.mkdir(parents=True,exist_ok=True)
        budget=directory/'planner_supervisor_budget.json';budget.write_text(json.dumps({'deadline_monotonic':time.monotonic()+3600,'kind':'preserved Final B outer supervisor; internal CDE/search caps unchanged'}));os.environ['RESEARCH_BUDGET_FILE']=str(budget)
        rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:
            try:
                if rclpy.ok():node.send(0.,0.)
            finally:
                node.planner_pool.shutdown(wait=True,cancel_futures=True);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

class GapDetector(Node):
    def __init__(self,**kwargs):
        super().__init__('gap_detector',**kwargs)
        for k,v in {'slot_length':6.2,'slot_width':2.1}.items():self.declare_parameter(k,v)
        self.map=None;self.road=None;self.state_wall=0.;self.map_wall=0.
        self.pub=self.create_publisher(String,'/parking/gap_observation',10);self.target=self.create_publisher(PoseStamped,'/parking/diagnostic_target',10)
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/parking/detection_map',self.on_map,q);self.create_subscription(String,'/parking/state',self.on_state,100);self.create_timer(.5,self.tick)
    def on_map(self,m):self.map=m;self.map_wall=time.monotonic()
    def on_state(self,m):
        try:
            j=json.loads(m.data)
            if j['valid']:
                state=finite_state(j['state']);self.state_wall=time.monotonic()
                if self.road is None:self.road=state[:3].copy()
            else:self.state_wall=0.
        except (ValueError,KeyError,TypeError):self.state_wall=0.
    def tick(self):
        if self.map is None or self.road is None or time.monotonic()-self.state_wall>.3 or time.monotonic()-self.map_wall>2.5:
            publish_json(self.pub,{'valid':False,'reason':'map/state unavailable'});return
        try:
            obs=observe_gap(grid_from_message(self.map),self.road,self.get_parameter('slot_length').value,self.get_parameter('slot_width').value);t=obs.target
            publish_json(self.pub,{'valid':t is not None,'reason':obs.reason,'first_obstacle':obs.first_obstacle,'gap_seen':obs.gap_seen,'second_obstacle':obs.second_obstacle,'center':t.center.tolist() if t else None,'heading':t.heading if t else None,'length':t.length if t else None,'width':t.width if t else None})
            if t:
                m=PoseStamped();m.header=self.map.header;m.pose.position.x=float(t.center[0]);m.pose.position.y=float(t.center[1]);m.pose.orientation.z=math.sin(t.heading/2);m.pose.orientation.w=math.cos(t.heading/2);self.target.publish(m)
        except (ValueError,RuntimeError) as e:publish_json(self.pub,{'valid':False,'reason':str(e)})

def gap_detector_main(args=None):spin(GapDetector,args)

class Planner(Node):
    def __init__(self,**kwargs):
        super().__init__('hybrid_astar',**kwargs);self.declare_parameter('geometry_file','');self.declare_parameter('timeout',90.)
        self.mod=install_b();self.vehicle=VehicleGeometry(self.get_parameter('geometry_file').value);self.geometry=self.vehicle.model();self.footprints=Footprints(*self.vehicle.polygons(),self.geometry);self.map=None;self.pool=ThreadPoolExecutor(max_workers=1);self.future=None;self.map_wall=0.
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(String,'/parking/planner_result',q);self.path=self.create_publisher(RosPath,'/parking/diagnostic_path',q)
        self.create_subscription(OccupancyGrid,'/tractor/localization/local_map',self.on_map,q);self.create_subscription(String,'/parking/planner_request',self.request,10);self.create_timer(.1,self.tick)
    def on_map(self,m):self.map=m;self.map_wall=time.monotonic()
    def request(self,m):
        try:
            if self.future is not None:raise ValueError('planner busy')
            if self.map is None or time.monotonic()-self.map_wall>2.5:raise ValueError('map unavailable')
            j=json.loads(m.data);start=finite_state(j['start']);goal=finite_state(j['goal']);gears=tuple(j.get('gears',[1,-1]))
            if not gears or any(x not in [-1,1] for x in gears):raise ValueError('invalid gears')
            planner=self.mod.HybridAStar(self.geometry,self.footprints,grid_from_message(self.map),timeout=self.get_parameter('timeout').value)
            self.future=self.pool.submit(planner.plan,start,goal,gears)
        except (ValueError,KeyError,RuntimeError,TypeError) as e:publish_json(self.pub,{'valid':False,'reason':str(e)})
    def tick(self):
        if self.future is None or not self.future.done():return
        future=self.future;self.future=None
        try:
            p=future.result();publish_json(self.pub,{'valid':True,'states':p.states.tolist(),'gears':p.gears.tolist(),'steering':p.steering.tolist(),'profile':'Final B clearance+shot+resume20 + steering-priority'})
            m=RosPath();m.header.frame_id='local_map';m.header.stamp=self.get_clock().now().to_msg()
            for s in p.states:
                pose=PoseStamped();pose.header=m.header;pose.pose.position.x=float(s[0]);pose.pose.position.y=float(s[1]);pose.pose.orientation.z=math.sin(s[2]/2);pose.pose.orientation.w=math.cos(s[2]/2);m.poses.append(pose)
            self.path.publish(m)
        except Exception as e:publish_json(self.pub,{'valid':False,'reason':str(e)})
    def close(self):self.pool.shutdown(wait=True,cancel_futures=True)

def hybrid_astar_main(args=None):spin(Planner,args)

class Lqr(Node):
    def __init__(self,**kwargs):
        super().__init__('lqr',**kwargs);self.declare_parameter('geometry_file','');self.declare_parameter('speed_factor',1.755);self.geometry=VehicleGeometry(self.get_parameter('geometry_file').value).model();self.controller=None;self.state=None;self.wall=0.;self.fault='waiting path/state'
        self.pub=self.create_publisher(Twist,'/parking/lqr_cmd_vel',10);self.status=self.create_publisher(String,'/parking/lqr_status',10)
        self.create_subscription(String,'/parking/planner_result',self.on_path,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));self.create_subscription(String,'/parking/state',self.on_state,100);self.create_timer(.1,self.tick)
    def on_path(self,m):
        try:
            j=json.loads(m.data)
            if not j.get('valid',False):raise ValueError(j.get('reason','invalid path'))
            self.controller=Controller(path_from_json(j),self.geometry,speed_factor=self.get_parameter('speed_factor').value);self.fault=''
        except (ValueError,KeyError,TypeError) as e:self.controller=None;self.fault=str(e)
    def on_state(self,m):
        try:
            j=json.loads(m.data);age=(self.get_clock().now().nanoseconds-j['stamp_ns'])/1e9
            if not j['valid'] or not -.03<=age<=.3:raise ValueError('invalid/stale state')
            self.state=finite_state(j['state']);self.wall=time.monotonic()
        except (ValueError,KeyError,TypeError):self.state=None
    def tick(self):
        v=delta=0.;done=False;valid=self.controller is not None and self.state is not None and time.monotonic()-self.wall<.25
        if valid:
            try:v,delta,done=self.controller.command(self.state,.1)
            except Exception as e:self.fault=str(e);self.controller=None;valid=False;v=delta=0.
        m=Twist();m.linear.x=float(v);m.angular.z=float(v/self.geometry.wheelbase*math.tan(delta));self.pub.publish(m);publish_json(self.status,{'valid':valid,'done':done,'velocity_m_s':v,'steering_rad':delta,'reason':self.fault if not valid else ''})
    def close(self):self.pub.publish(Twist())

def lqr_main(args=None):spin(Lqr,args)
