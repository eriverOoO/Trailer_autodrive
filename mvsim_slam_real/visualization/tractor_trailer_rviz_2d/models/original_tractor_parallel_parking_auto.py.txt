#!/usr/bin/env python3
"""Local LiDAR pose, trailer-IMU articulation, automatic parallel parking.

No vehicle odometry or ground-truth subscription is permitted in this node.
The planner/controller use map-frame state and OccupancyGrid exclusively.
"""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
from threading import Thread
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Polygon,Twist,PoseStamped,Point
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import Bool,String
from visualization_msgs.msg import Marker,MarkerArray
from tractor_trailer_model import Geometry,Footprints,wrap,transform,hitch,step
from tractor_trailer_state_estimator import StateEstimator,yaw
from tractor_trailer_hybrid_astar import HybridAStar,PLANNING_ARTICULATION_LIMIT_DEG
from tractor_parking_perception import planning_grid,Target
from tractor_local_parking_gap_detector import observe_gap
from tractor_parking_controller import Controller
from tractor_parking_articulation_safety import ArticulationSafety
from tractor_parking_blocked_debug import evidence as blocked_evidence, save as save_blocked
from tractor_parking_clearance import (PLANNER_HARD_CLEARANCE_M,
    PLANNER_PREFERRED_CLEARANCE_M,LIVE_BLOCK_CLEARANCE_M,LIVE_BLOCK_CONFIRM_COUNT,
    LIVE_BLOCK_CONFIRM_SEC,path_metrics)
from tractor_parking_map_safety import LiveBlockConfirmation,path_block_observation
from raw_reject_event_capture.raw_reject_capture import snapshot_from_msg, capture_reject, save_next_map


POSE_FRESHNESS_THRESHOLD = .8  # Seconds of simulation time; diagnostic reference.
POSE_FRESHNESS_WARNING_INTERVAL = 5.  # Wall seconds, independent of /clock stalls.


class AutoParking(Node):
    def __init__(self):
        super().__init__('tractor_parallel_parking_auto')
        defaults={'parking_stop_policy':'legacy','detection_only':False,'auto_execute':False,'auto_detect_target':True,'world_file':'',
                  'execute_parking':False,'search_speed':.14,'parking_speed_factor':1.0,'completion_hinge_deg':6.25,'completion_settle_max_s':8.0,
                  'survey_before_parking':False,'planner_timeout':35.,'beam_gap_filling':True,
                  'slot_length':5.8,'slot_width':2.,'warning_angle_deg':30.,'emergency_angle_deg':35.,
                  'max_replans':3,'blocked_debug_directory':'',
                  'enable_pose_freshness_stop':False,'localization_source':'local'}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.auto=bool(self.get_parameter('auto_execute').value)
        self.detect=bool(self.get_parameter('auto_detect_target').value)
        if not self.detect:raise RuntimeError('Automatic gap mode requires auto_detect_target=true; no fixed-target fallback')
        self.stop_policy=self.get_parameter('parking_stop_policy').value
        if self.stop_policy not in ['legacy','observed_safe']:raise ValueError('unknown parking_stop_policy')
        self.stop_observations=[];self.stop_audit_events=[];self.stop_started=None;self.stop_completed=None
        self.detection_msg=None;self.search_started=None;self.detect_started=None
        self.create_subscription(OccupancyGrid,'/parking/detection_map',lambda m:setattr(self,'detection_msg',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pose_freshness_stop_enabled=bool(self.get_parameter('enable_pose_freshness_stop').value)
        self.max_pose_age=0.;self.pose_freshness_exceed_count=0
        self.pose_age_samples=0;self.last_pose_freshness_warning=-math.inf
        self.geometry=Geometry.from_world(self.get_parameter('world_file').value)
        self.articulation_safety=ArticulationSafety(self.geometry)
        self.est=StateEstimator(self,source=self.get_parameter('localization_source').value)
        self.stage='WAIT_FOR_LOCALIZATION';self.started=time.monotonic();self.map_msg=None
        self.map_sequence=0;self.map_snapshot=None;self.raw_grid_cache=None
        self.live_block=LiveBlockConfirmation();self.plan_context=None
        self.map_wall=0.;self.scan_wall=0.;self.tractor_imu_wall=0.;self.polygons={};self.footprints=None
        # Diagnostic-only, bounded RAW-map history; used only if a pre-execute reject occurs.
        self.raw_map_history=deque(maxlen=3);self.pending_raw_reject=None;self.raw_reject_count=0
        self.raw_diagnostic_episode=0;self.latest_scan_stamp_ns=0
        self.target=None;self.goal=None;self.path=None;self.controller=None;self.future=None
        self.planner_pool=ThreadPoolExecutor(max_workers=1);self.replans=0;self.surveyed=True;self.survey_plan=False
        self.road_pose=None;self.current=None;self.last_log=0.;self.last_detect=0.;self.last_plan_error=''
        self.max_phi=0.;self.collision_count=0;self.deviation_since=None;self.final_wait=None
        self.last_state_for_speed=None;self.measured_speed=0.;self.settle_until=0.;self.failure=''
        self.v=0.;self.delta=0.;self.planning_time=0.;self.total_planned_length=0.;self.gear_changes=0
        self.goal_shift=0.
        self.goal_lateral_shift=0.
        self.local_status_wall=0.;self.last_observation=None;self.observation_hits=0
        self.observation_stamp=None;self.observation_window=[]
        self.first_seen=False;self.gap_seen=False;self.second_seen=False
        self.max_tracking_error=0.;self.search_distance=0.
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/tractor/localization/local_map',self.on_map,q)
        for name in ('tractor','trailer'):
            self.create_subscription(Polygon,f'/{name}/chassis_polygon',lambda m,k=name:self.on_polygon(k,m),q)
            self.create_subscription(Bool,f'/{name}/collision',lambda m,k=name:self.on_collision(k,m),10)
        # Trailer freshness validated in combined state adapter; no tractor IMU.
        self.create_subscription(LaserScan,'/scan',self.on_scan,10)
        self.create_subscription(String,'/tractor/localization/status',self.on_local_status,20)
        self.cmd=self.create_publisher(Twist,'/parking/request_cmd_vel',10)
        self.status_pub=self.create_publisher(String,'/parking/status',q)
        self.tractor_path_pub=self.create_publisher(RosPath,'/parking/planned_tractor_path',q)
        self.trailer_path_pub=self.create_publisher(RosPath,'/parking/planned_trailer_path',q)
        self.target_pub=self.create_publisher(PoseStamped,'/parking/target',q)
        self.markers_pub=self.create_publisher(MarkerArray,'/parking/debug_markers',q)
        self.timer=self.create_timer(.1,self.tick)
        self.get_logger().info('POSE FRESHNESS SAFETY STOP: '+
            ('ENABLED' if self.pose_freshness_stop_enabled else 'DISABLED'))
        self.get_logger().info(f'[Parking] WAIT_FOR_LOCALIZATION auto_execute={self.auto}')

    def observe_pose_age(self):
        """Keep freshness diagnostics regardless of whether its stop is enabled.

        Count active parking timer samples above the reference threshold, not
        distinct pose messages. Do not send commands or change parking state.
        """
        if self.est.pose_stamp is None:return None
        age=self.get_clock().now().nanoseconds*1e-9-self.est.pose_stamp
        self.pose_age_samples+=1
        self.max_pose_age=max(self.max_pose_age,age)
        if age>POSE_FRESHNESS_THRESHOLD:
            self.pose_freshness_exceed_count+=1
            now=time.monotonic()
            if now-self.last_pose_freshness_warning>=POSE_FRESHNESS_WARNING_INTERVAL:
                self.last_pose_freshness_warning=now
                self.get_logger().warning(
                    f'POSE_FRESHNESS_EXCEEDED age={age:.6f}s '
                    f'stop_disabled={not self.pose_freshness_stop_enabled}')
        return age

    def pose_freshness_stop_required(self,age):
        return (self.pose_freshness_stop_enabled and age is not None and
                age>POSE_FRESHNESS_THRESHOLD)

    def transition(self,new,reason=''):
        if new==self.stage:return
        self.get_logger().info(f'[Parking] {self.stage} -> {new}: {reason}')
        if self.stage=='EXECUTE':self.live_block.reset()
        self.stage=new
        if new in ('FAILED','EMERGENCY_STOP'):self.failure=reason
        self.publish_status(force=True)

    def on_map(self,msg):
        self.map_msg=msg;self.map_wall=time.monotonic()
        self.map_sequence+=1;self.map_snapshot=(msg,self.map_sequence)
        diag_snapshot=snapshot_from_msg(msg,self.map_sequence,self.map_wall)
        if self.pending_raw_reject is not None:
            pending=self.pending_raw_reject
            self.pending_raw_reject=None
            Thread(target=self._save_raw_reject_next,args=(pending,diag_snapshot),daemon=True).start()
        self.raw_map_history.append(diag_snapshot)
        if self.live_block.pending is not None:
            o=msg.info.origin
            events=self.live_block.clear_if_cell_free(
                np.asarray(msg.data).reshape(msg.info.height,msg.info.width),
                msg.info.resolution,[o.position.x,o.position.y,yaw(o.orientation)],
                self.map_sequence,msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                self.map_wall)
            self.log_block_events(events)
        if self.get_parameter('blocked_debug_directory').value:
            data=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
            stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            if not hasattr(self,'occupied_first_stamp') or self.occupied_first_stamp.shape!=data.shape:
                self.occupied_first_stamp=np.zeros(data.shape,dtype=np.int64)
                self.occupied_first_stage={}
            new=(data>=50)&(self.occupied_first_stamp==0)
            self.occupied_first_stamp[new]=stamp
            if np.any(new):self.occupied_first_stage[stamp]=self.stage

    def on_scan(self,msg):
        self.scan_wall=time.monotonic()
        self.latest_scan_stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec

    def _save_raw_reject_next(self,pending,snapshot):
        try:
            result=save_next_map(pending,snapshot)
            self.get_logger().info('[RAW_REJECT_NEXT_MAP] '+json.dumps(result))
        except Exception as exc:
            self.get_logger().error(f'[RAW_REJECT_NEXT_MAP_FAILED] {exc}')

    def record_blocked(self,grid,start_index,reason,stop=True,predicted_states=None,
                       confirmation=None,decision='STOP_AND_REPLAN'):
        # Stop first. Diagnostics never delay an emergency braking command or
        # modify occupancy, collision criteria, path or replan decisions.
        if stop:self.send(0,0)
        directory=self.get_parameter('blocked_debug_directory').value
        if not directory:return
        try:
            result=blocked_evidence(grid,self.execution_grid,self.path.states,
                self.footprints,self.current,start_index,self.target)
            if predicted_states is not None:
                result=blocked_evidence(grid,self.execution_grid,np.asarray(predicted_states),
                    self.footprints,self.current,0,self.target)
                result['first_blocked_prediction_step']=result.get('first_blocked_path_state_index')
                result['first_blocked_path_state_index']=None
                result['path_index']=int(start_index)
                result['collision_source']='predicted actual motion, not a planned path state'
            elif stop and (not grid.collision_free(self.current,self.footprints) or
                           (confirmation or {}).get('source')=='current_clearance'):
                result=blocked_evidence(grid,self.execution_grid,self.current[None,:],
                    self.footprints,self.current,0,self.target)
                result['first_blocked_path_state_index']=None
                result['path_index']=int(start_index)
                result['collision_source']='current actual footprint, planned path can remain free'
            stamp=grid.map_stamp_ns
            result.update(timestamp_ns=self.get_clock().now().nanoseconds,
                map_timestamp_ns=stamp,map_age_s=self.get_clock().now().nanoseconds*1e-9-stamp*1e-9,
                map_received_age_s=time.monotonic()-self.map_wall,
                planning_map_timestamp_ns=self.planning_map_stamp_ns,reason=reason,stage=self.stage,
                replan_count_before_event=self.replans,map_sequence=grid.map_sequence,
                consecutive_blocked_observations=(confirmation or {}).get('consecutive_block_count',1),
                block_persistence_s=(confirmation or {}).get('block_persistence_s',0.),
                observation_count_note='Clearance/path blocks require distinct-map count and persistence; actual current overlap is immediate.',
                bay_center=self.target.center.tolist(),bay_heading=self.target.heading,
                bay_length=self.target.length,bay_width=self.target.width)
            result['decision']=decision if stop else 'STATIC_BOUNDARY_REFERENCE_WITH_RAW_NEAR_SAFETY'
            if confirmation is not None:result['block_confirmation']=confirmation
            for cell in result['blocked_cells']:
                i,j=cell['ij'];first=int(self.occupied_first_stamp[j,i])
                cell.update(first_occupied_stamp_ns=first,first_occupied_stage=self.occupied_first_stage.get(first),
                    first_appeared_after_execution_started=self.occupied_first_stage.get(first)=='EXECUTE')
            self.blocked_debug_count=getattr(self,'blocked_debug_count',0)+1
            save_blocked(directory,self.blocked_debug_count,result,grid,self.execution_grid,
                self.path.states,self.footprints,self.current,self.target)
            self.get_logger().info('[BlockedEvidence] '+json.dumps(result))
        except Exception as exc:self.get_logger().error(f'Blocked evidence capture failed: {exc}')

    def log_block_events(self,events):
        for event,record in events:
            record.update(clearance_threshold_m=LIVE_BLOCK_CLEARANCE_M,
                confirm_count=LIVE_BLOCK_CONFIRM_COUNT,confirm_sec=LIVE_BLOCK_CONFIRM_SEC)
            self.get_logger().info(f'[{event}] '+json.dumps(record))

    def confirm_live_block(self,grid,observation,now):
        if observation is not None:
            value=None
            if observation['blocking_cell_xy'] is not None:
                i,j=self.execution_grid.world_to_cell(observation['blocking_cell_xy'])[0]
                if 0<=i<self.execution_grid.width and 0<=j<self.execution_grid.height:
                    value=int(self.execution_grid.data[j,i])
            observation.update(planning_occupancy=value,
                existed_in_planning_snapshot=(value>=50 or value<0) if value is not None else None,
                planning_map_sequence=self.execution_grid.map_sequence,
                planning_map_stamp_ns=self.execution_grid.map_stamp_ns)
        self.log_block_events(self.live_block.update(
            observation,grid.map_sequence,grid.map_stamp_ns,now))
        return self.live_block.confirmed

    def reject_ready_candidate(self,reason):
        """Resume the existing finite candidate search without counting a replan."""
        self.send(0,0)
        self.get_logger().info('[PRE_EXECUTE_REJECTED] '+reason)
        self.plan_context['failures'].append(reason)
        self.path=None;self.controller=None;self.live_block.reset()
        self.transition(self.plan_context['stage'],reason)
        self.submit_plan_candidates()

    def on_local_status(self,msg):
        self.local_status_wall=time.monotonic()
        try:
            status=json.loads(msg.data)
            self.local_status=status
            if not status.get('initialized',False):self.local_status_wall=0.
        except (ValueError,TypeError):self.local_status_wall=0.

    def detect_gap(self,road_pose,length,width):
        if self.detection_msg is None:raise RuntimeError('Cartographer detection grid not received')
        m=self.detection_msg;o=m.info.origin
        grid=planning_grid(np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,[o.position.x,o.position.y,yaw(o.orientation)],simulation_clearance=True)
        obs=observe_gap(grid,road_pose,length,width)
        directory=self.get_parameter('blocked_debug_directory').value
        if directory:
            record={'sim_ns':self.get_clock().now().nanoseconds,'map_stamp_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,'source':'Cartographer /map: geometry identical, probabilities thresholded50, unknown preserved, no free-space injection','first':obs.first_obstacle,'gap':obs.gap_seen,'second':obs.second_obstacle,'reason':obs.reason,'center':obs.target.center.tolist() if obs.target else None,'yaw':obs.target.heading if obs.target else None,'length':obs.target.length if obs.target else None,'width':obs.target.width if obs.target else None}
            from pathlib import Path
            with (Path(directory)/'gap_observations.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
        return obs

    def stable_observation(self,target):
        stamp=(self.detection_msg.header.stamp.sec,self.detection_msg.header.stamp.nanosec)
        if target is None:
            if self.stop_policy=='observed_safe':self.observation_window=[];self.observation_hits=0
            return False
        if stamp!=self.observation_stamp:
            self.observation_stamp=stamp
            stable=all(np.linalg.norm(target.center-t.center)<.15 and
                abs(wrap(target.heading-t.heading))<math.radians(2) and
                abs(target.length-t.length)<.20 and abs(target.width-t.width)<.15
                for t in self.observation_window)
            if not stable:self.observation_window=[]
            if not stable:self.legacy_observation_stamps=[]
            self.legacy_observation_stamps=(getattr(self,'legacy_observation_stamps',[])+[stamp])[-3:]
            self.observation_window.append(target)
            self.observation_window=self.observation_window[-3:]
            self.observation_hits=len(self.observation_window)
            self.last_observation=target
        return self.observation_hits>=3
    def on_polygon(self,name,msg):
        self.polygons[name]=[(p.x,p.y) for p in msg.points]
        if len(self.polygons)==2 and self.footprints is None:
            self.footprints=Footprints(self.polygons['tractor'],self.polygons['trailer'],self.geometry)
            self.get_logger().info(f'Chassis+wheel envelope size={self.footprints.size}, center={self.footprints.center_offset}')
    def on_collision(self,name,msg):
        if msg.data:
            self.collision_count+=1;self.send(0,0)
            self.transition('EMERGENCY_STOP',f'{name} collision reported')

    def send(self,v,delta):
        self.v=float(v);self.delta=float(delta)
        self.articulation_safety.record(self.get_clock().now().nanoseconds*1e-9,v,delta)
        if not self.auto:return
        m=Twist();m.linear.x=float(v);m.angular.z=float(v/self.geometry.wheelbase*math.tan(delta))
        self.cmd.publish(m)

    def grid(self,fill=False):
        m,sequence=self.map_snapshot;o=m.info.origin
        if not fill and self.raw_grid_cache is not None:
            cached_sequence,cached_grid=self.raw_grid_cache
            if cached_sequence==sequence:return cached_grid
        grid=planning_grid(np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,
                             [o.position.x,o.position.y,yaw(o.orientation)],fill,simulation_clearance=True)
        grid.map_stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        grid.map_sequence=sequence
        grid.frame_id=m.header.frame_id
        if not fill:self.raw_grid_cache=(sequence,grid)
        return grid

    def select_goal(self):
        """Choose the nearest trailer-safe pose with a feasible reverse entry.

        Test a forward departure from each bay-contained pose and reverse that
        collision-checked path during parking. The gap center stays measured.
        """
        grid=self.grid(False)
        heading=self.target.heading
        along=np.array([math.cos(heading),math.sin(heading)])
        normal=np.array([-along[1],along[0]])
        # Extra longitudinal room at the tractor nose helps a U-bay shunt.
        # This offset is within the measured rectangle, never a world pose.
        for shift in (.4,0.,.2,.6):
            center=self.target.center-shift*along
            goal=self.footprints.goal(center,heading)
            if not self.target.contains(self.footprints.world(goal)) or not grid.check(goal,self.footprints):
                continue
            offset=float((self.current[:2]-goal[:2])@normal)
            staging=goal.copy();staging[:2]+=7.*along+offset*normal
            # A single smooth forward connector is not a feasibility oracle
            # for a U cavity. The 4D planner performs the full departure audit.
            if self.stop_policy=='observed_safe':
                from observed_stop import observed_staging_distance
                ahead=observed_staging_distance(grid,goal,float(self.road_pose[:2]@normal),self.footprints)
                staging_safe=ahead is not None
            else:staging_safe=grid.check(staging,self.footprints)
            if staging_safe:
                self.goal_shift=shift
                return goal
        raise RuntimeError('no bay-contained goal has a safe reverse entry')

    def launch_plan(self):
        self.raw_diagnostic_episode+=1
        self.send(0,0)
        self.live_block.reset()
        start=self.current.copy();goal=self.goal.copy();strict=self.grid(False)
        self.planning_map_stamp_ns=strict.map_stamp_ns
        directory=self.get_parameter('blocked_debug_directory').value
        if directory:
            from pathlib import Path
            out=Path(directory);out.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(out/f'planning_input_{self.replans+1:03d}.npz',
                data=strict.data,resolution=strict.resolution,origin=strict.origin,
                start=start,goal=goal,tractor=self.footprints.polygons[0],
                trailer=self.footprints.polygons[1],map_stamp_ns=strict.map_stamp_ns)
        # Keep the original full collision envelope; clearance is a separate
        # hard planning constraint and soft preference, not a cost redesign.
        timeout=float(self.get_parameter('planner_timeout').value)
        survey=not self.surveyed and bool(self.get_parameter('survey_before_parking').value)
        if survey:
            # A map-validated observation pose on the road, ahead of the gap.
            local=transform([self.target.center-self.road_pose[:2]],[0,0],-self.road_pose[2])[0]
            xy=transform([[local[0]+3.,0.]],self.road_pose[:2],self.road_pose[2])[0]
            goal=np.r_[xy,self.road_pose[2],self.road_pose[2]]
        self.survey_plan=survey
        filled=self.grid(True) if self.get_parameter('beam_gap_filling').value else None
        goal_candidates=[(goal,self.goal_shift,0.)]
        if not survey:
            along=np.array([math.cos(self.target.heading),math.sin(self.target.heading)])
            normal=np.array([-along[1],along[0]])
            for shift,lateral in ((.4,.1),(0.,0.),(.2,0.),(.4,-.1),(.6,0.)):
                candidate=self.footprints.goal(self.target.center-shift*along+lateral*normal,self.target.heading)
                if self.target.contains(self.footprints.world(candidate)) and strict.check(candidate,self.footprints):
                    if not any(np.linalg.norm(candidate-g)<.01 for g,_,_ in goal_candidates):
                        goal_candidates.append((candidate,shift,lateral))
        # Preserve the original candidate order/count and snapshots. A path
        # rejected just before EXECUTE resumes AFTER its existing attempt;
        # it is neither executed nor charged to the execution replan budget.
        self.plan_context=dict(start=start,survey=survey,timeout=timeout,filled=filled,
            normal=normal if not survey else None,stage=self.stage,next_index=0,failures=[],
            attempts=[(grid,candidate,shift,lateral) for grid in [strict,filled]
                      if grid is not None for candidate,shift,lateral in goal_candidates])
        self.submit_plan_candidates()
        self.get_logger().info(f'4D Hybrid A* start={np.round(start,3)} goal={np.round(goal,3)} survey={survey}')

    def submit_plan_candidates(self):
        context=self.plan_context
        def work():
            attempts=context['attempts']
            while context['next_index']<len(attempts):
                grid,candidate,shift,lateral=attempts[context['next_index']]
                context['next_index']+=1
                planner=HybridAStar(self.geometry,self.footprints,grid,timeout=context['timeout'])
                try:
                    if context['survey']:
                        path=planner.plan(context['start'],candidate,gears=(1,))
                    else:
                        # Preserve observed-road staging during recovery.
                        road_y=float((self.road_pose[:2]-candidate[:2])@context['normal'])
                        path=planner.parking_plan(context['start'],candidate,road_y=road_y,staging_distances=(7.,))
                    raw=path_metrics(self.grid(False),path.states,self.footprints)
                    if not raw['valid']:
                        raise RuntimeError(f"latest RAW candidate precheck rejected: minimum={raw['minimum_clearance_m']:.6f}m, hard={PLANNER_HARD_CLEARANCE_M:.2f}m (or footprint overlap)")
                    return path,grid,grid is context['filled'],candidate,shift,lateral
                except RuntimeError as exc:
                    detail=f'goal shift={shift:.2f}, lateral={lateral:.2f}: {exc}'
                    context['failures'].append(detail);self.get_logger().info(detail)
            raise RuntimeError('candidate search exhausted: '+' | '.join(context['failures']))
        self.future=self.planner_pool.submit(work)

    def tick(self):
        try:self.tick_checked()
        except Exception as exc:
            self.send(0,0);self.transition('FAILED',f'{type(exc).__name__}: {exc}')
            self.get_logger().error(str(exc))

    def tick_checked(self):
        if self.stage in ('FAILED','EMERGENCY_STOP','PARKED'):
            self.send(0,0);self.publish_status();return
        now=time.monotonic()
        if self.stage=='WAIT_FOR_LOCALIZATION':
            if (self.map_msg is None or self.map_msg.header.frame_id!='local_map' or
                self.footprints is None or self.est.pose() is None or
                now-self.scan_wall>1. or
                now-self.local_status_wall>1.5):
                self.publish_status();return
            self.transition('CALIBRATE_TRAILER')
        s=self.est.update(calibrate=self.stage=='CALIBRATE_TRAILER')
        self.observe_pose_age()
        if s is None:
            self.send(0,0)
            if self.stage in ('SEARCH_FORWARD','FIRST_OBSTACLE_SEEN','GAP_SEEN',
                              'SECOND_OBSTACLE_SEEN','EXECUTE'):
                self.transition('EMERGENCY_STOP',self.est.last_error)
            self.publish_status();return
        self.current=s
        phi=abs(wrap(s[3]-s[2]));self.max_phi=max(self.max_phi,phi)
        if phi>=math.radians(float(self.get_parameter('emergency_angle_deg').value)):
            self.send(0,0);self.transition('EMERGENCY_STOP','articulation hard safety limit');return
        if self.last_state_for_speed is not None:
            old,stamp=self.last_state_for_speed
            if now-stamp>.4:
                self.measured_speed=float(np.linalg.norm(s[:2]-old[:2])/(now-stamp));self.last_state_for_speed=(s.copy(),now)
        else:self.last_state_for_speed=(s.copy(),now)
        if self.stage=='CALIBRATE_TRAILER':
            self.road_pose=self.est.initial_pose.copy();self.transition('SEARCH_FORWARD')
        if self.stage in ('SEARCH_FORWARD','FIRST_OBSTACLE_SEEN','GAP_SEEN','SECOND_OBSTACLE_SEEN'):
            if (now-self.map_wall>1.5 or now-self.scan_wall>1. or now-self.local_status_wall>1.5):
                self.send(0,0);self.transition('EMERGENCY_STOP','localization/map stream stale');return
            road_error=transform([s[:2]-self.road_pose[:2]],[0,0],-self.road_pose[2])[0]
            self.search_distance=float(road_error[0])
            heading_error=wrap(s[2]-self.road_pose[2])
            if abs(road_error[1])>1.0 or abs(heading_error)>math.radians(15):
                self.send(0,0);self.transition('EMERGENCY_STOP','forward search left road corridor');return
            if self.search_started is None:self.search_started=now
            if road_error[0]>18.0 or now-self.search_started>160.:
                self.send(0,0);self.transition('FAILED','automatic gap search limit reached; no fixed target fallback');return
            if not self.grid(False).collision_free(s,self.footprints):
                self.send(0,0);self.transition('EMERGENCY_STOP','search footprint collides with observed map');return
            if now-self.last_detect>.5:
                self.last_detect=now
                obs=self.detect_gap(self.road_pose,
                    float(self.get_parameter('slot_length').value),
                    float(self.get_parameter('slot_width').value))
                self.last_gap_observation=obs
                self.first_seen |= obs.first_obstacle
                self.gap_seen |= obs.gap_seen
                self.second_seen |= obs.second_obstacle
                self.stable_observation(obs.target)
                if self.first_seen and self.stage=='SEARCH_FORWARD':self.transition('FIRST_OBSTACLE_SEEN')
                if self.gap_seen and self.stage=='FIRST_OBSTACLE_SEEN':self.transition('GAP_SEEN')
                if self.second_seen and self.stage=='GAP_SEEN':self.transition('SECOND_OBSTACLE_SEEN')
                target_u=transform([obs.target.center-self.road_pose[:2]],[0,0],-self.road_pose[2])[0,0] if obs.target is not None else math.inf
                stop_allowed=self.observation_hits>=3 and road_error[0]>target_u+3.0 if self.stop_policy=='legacy' else False
                stop_evidence={}
                if self.stop_policy=='observed_safe':
                    from observed_stop import observed_interior,safe_stop_sweep
                    m=self.detection_msg;o=m.info.origin
                    detection_grid=planning_grid(np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,[o.position.x,o.position.y,yaw(o.orientation)],simulation_clearance=True)
                    interior=observed_interior(detection_grid,obs.target) if obs.target is not None else {'complete':False,'reason':'no full current geometry'}
                    stamp=(m.header.stamp.sec,m.header.stamp.nanosec)
                    if not interior['complete']:
                        self.observation_window=[];self.observation_hits=0;self.stop_observations=[]
                    elif not self.stop_observations or self.stop_observations[-1]['stamp']!=list(stamp):
                        self.stop_observations.append({'stamp':list(stamp),'free_fraction':interior['free_fraction']})
                        self.stop_observations=self.stop_observations[-3:]
                    stable=self.observation_hits>=3 and len(self.stop_observations)>=3 and interior['complete']
                    sweep=safe_stop_sweep(self.grid(False),s,self.footprints,self.geometry,max(abs(self.v),abs(self.measured_speed)),self.delta) if stable else {'safe':False,'reason':'awaiting three complete stable maps'}
                    stop_allowed=stable and sweep['safe'] and self.collision_count==0
                    stop_evidence={'sim_ns':self.get_clock().now().nanoseconds,'mono_ns':time.monotonic_ns(),'state':s.tolist(),'target_center':obs.target.center.tolist() if obs.target else None,'map_stamp':list(stamp),'observation_hits':self.observation_hits,'complete_map_count':len(self.stop_observations),'interior':interior,'sweep':sweep,'stop_allowed':stop_allowed,'road_coordinate':float(road_error[0]),'target_road_coordinate':float(target_u) if np.isfinite(target_u) else None,'pose_stamp_s':self.est.pose_stamp,'source':'actual Cartographer+gyro state; no GT'}
                    directory=self.get_parameter('blocked_debug_directory').value
                    if directory:
                        from pathlib import Path
                        with (Path(directory)/'observed_stop_audit.jsonl').open('a') as f:f.write(json.dumps(stop_evidence)+'\n')
                if stop_allowed:
                    if self.stop_policy=='legacy':
                        self.legacy_stop_evidence={'sim_ns':self.get_clock().now().nanoseconds,'state':s.tolist(),'road_pose':self.road_pose.tolist(),'road_coordinate':float(road_error[0]),'target_road_coordinate':float(target_u),'center_offset_m':float(road_error[0]-target_u),'observation_hits':self.observation_hits,'map_stamps':getattr(self,'legacy_observation_stamps',[]),'condition':'observation_hits>=3 and road_error[0]>target_u+3.0','target_center':obs.target.center.tolist()}
                    # Preserve the proven10s map settling and post-stop3-map
                    # revalidation. No fixed extra forward displacement.
                    self.send(0,0);self.settle_until=now+10.;self.stop_started=stop_evidence or {'sim_ns':self.get_clock().now().nanoseconds,'mono_ns':time.monotonic_ns(),'state':s.tolist()}
                    self.observation_window=[];self.observation_hits=0;self.observation_stamp=None
                    self.transition('STOP_AND_STABILIZE','three full stable observed maps and safe swept stop' if self.stop_policy=='observed_safe' else 'three independent stable U-bay map observations');return
            pose_age=self.get_clock().now().nanoseconds*1e-9-self.est.pose_stamp
            search_speed=float(self.get_parameter('search_speed').value)
            search_steer=float(np.clip(-.25*road_error[1]-.8*heading_error,
                                       -math.radians(8),math.radians(8)))
            self.send(min(search_speed,.08) if pose_age>1.0 else search_speed,
                      search_steer)
            self.visualize();self.publish_status();return
        if self.stage=='STOP_AND_STABILIZE':
            self.send(0,0)
            if self.stop_policy=='legacy' and not hasattr(self,'legacy_stop_completed') and abs(self.measured_speed)<.06 and now-self.stop_started['mono_ns']/1e9>=2.:
                self.legacy_stop_completed={'sim_ns':self.get_clock().now().nanoseconds,'state':s.tolist(),'measured_speed_m_s':self.measured_speed,'zero_command':[self.v,self.delta]}
            if self.stop_policy=='observed_safe':
                if not self.grid(False).collision_free(s,self.footprints) or self.collision_count:
                    self.transition('EMERGENCY_STOP','unsafe observed-stop current footprint');return
                stationary=abs(self.measured_speed)<.06 and self.stop_started is not None and now-self.stop_started['mono_ns']/1e9>=2.
                if stationary and self.stop_completed is None:
                    self.stop_completed={'sim_ns':self.get_clock().now().nanoseconds,'mono_ns':time.monotonic_ns(),'state':s.tolist(),'measured_speed_m_s':self.measured_speed,'zero_command':[self.v,self.delta]}
                    directory=self.get_parameter('blocked_debug_directory').value
                    if directory:
                        from pathlib import Path
                        (Path(directory)/'observed_stop_completed.json').write_text(json.dumps({'start':self.stop_started,'completed':self.stop_completed},indent=2))
                if now>=self.settle_until and not stationary:
                    self.transition('FAILED','observed stop not stationary after existing stabilization interval');return
            if now>=self.settle_until:self.transition('DETECT_TARGET')
            else:self.publish_status();return
        if self.stage=='DETECT_TARGET':
            self.send(0,0)
            if now<self.settle_until:return
            self.last_detect=now
            if self.detect_started is None:self.detect_started=now
            if now-self.detect_started>30.:
                self.send(0,0);self.transition('FAILED','gap revalidation timed out; no fixed target fallback');return
            try:
                obs=self.detect_gap(self.road_pose,
                    float(self.get_parameter('slot_length').value),
                    float(self.get_parameter('slot_width').value))
                if obs.target is None:raise RuntimeError(obs.reason)
                if not self.stable_observation(obs.target):
                    self.publish_status();return
                self.target=obs.target
                self.goal=self.select_goal()
                self.get_logger().info(f'Target center={np.round(self.target.center,3)} goal={np.round(self.goal,3)} shift={self.goal_shift:.2f}m {self.target.confidence}')
                if self.get_parameter('detection_only').value:self.transition('READY','observed target confirmed; detection-only gate');return
                self.transition('PLAN');self.launch_plan()
            except RuntimeError as exc:
                self.last_plan_error=str(exc);self.get_logger().info(f'Waiting for map target: {exc}')
            self.visualize();self.publish_status();return
        if self.stage in ('PLAN','REPLAN'):
            self.send(0,0)
            if self.future is not None and self.future.done():
                try:
                    self.path,self.execution_grid,filled,self.goal,self.goal_shift,self.goal_lateral_shift=self.future.result()
                    self.planning_time+=self.path.elapsed;self.total_planned_length+=self.path.length
                    self.gear_changes+=self.path.gear_changes
                    if not self.execution_grid.path_free(self.path.states,self.footprints):
                        raise RuntimeError('planned swept path failed collision/articulation audit')
                    if np.max(np.abs(wrap(self.path.states[:,3]-self.path.states[:,2]))) > math.radians(PLANNING_ARTICULATION_LIMIT_DEG):
                        raise RuntimeError('planned path exceeds 30-degree articulation safety limit')
                    planner_metrics=path_metrics(self.execution_grid,self.path.states,self.footprints)
                    latest=self.grid(False);raw_metrics=path_metrics(latest,self.path.states,self.footprints)
                    if not raw_metrics['valid']:
                        self.reject_ready_candidate('latest RAW map changed after candidate precheck');return
                    self.clearance_metrics=dict(path_length_m=self.path.length,
                        planner_min_clearance_m=planner_metrics['minimum_clearance_m'],
                        min_swept_tractor_clearance_m=planner_metrics['minimum_tractor_clearance_m'],
                        min_swept_trailer_clearance_m=planner_metrics['minimum_trailer_clearance_m'],
                        raw_min_clearance_m=raw_metrics['minimum_clearance_m'],
                        difference_m=raw_metrics['minimum_clearance_m']-planner_metrics['minimum_clearance_m'],
                        preferred_clearance_m=PLANNER_PREFERRED_CLEARANCE_M,hard_clearance_m=PLANNER_HARD_CLEARANCE_M,
                        clearance_penalty=planner_metrics['soft_penalty'],
                        near_wall_exposure_distance_m=planner_metrics['near_wall_exposure_distance_m'],
                        near_wall_exposure_cost=planner_metrics['near_wall_exposure_cost'],
                        minimum_clearance_quality_cost=planner_metrics['minimum_clearance_quality_cost'],
                        distance_cost_m=self.path.length,
                        total_path_quality_score=planner_metrics['total_path_quality_score'],
                        planner_expansions=self.path.expanded,planner_time_s=self.path.elapsed,
                        planning_map_stamp_ns=self.execution_grid.map_stamp_ns,raw_map_stamp_ns=latest.map_stamp_ns,
                        beam_gap_fill=filled,
                        metric_note='Same full footprint and nearest-cell EDT for both hard checks; bilinear sampling only for soft cost. Differences reflect map snapshots or optional gap filling.')
                    self.controller=Controller(self.path,self.geometry,speed_factor=float(self.get_parameter("parking_speed_factor").value),completion_hinge_deg=float(self.get_parameter("completion_hinge_deg").value))
                    self.live_block.reset()
                    self.get_logger().info('[CLEARANCE PATH] '+json.dumps(self.clearance_metrics))
                    self.get_logger().info('[PATH QUALITY] '+json.dumps({
                        'path_length_m':self.path.length,
                        'min_swept_tractor_clearance_m':planner_metrics['minimum_tractor_clearance_m'],
                        'min_swept_trailer_clearance_m':planner_metrics['minimum_trailer_clearance_m'],
                        'combined_min_clearance_m':planner_metrics['minimum_clearance_m'],
                        'near_wall_exposure_distance_m':planner_metrics['near_wall_exposure_distance_m'],
                        'near_wall_exposure_cost':planner_metrics['near_wall_exposure_cost'],
                        'minimum_clearance_quality_cost':planner_metrics['minimum_clearance_quality_cost'],
                        'distance_cost_m':self.path.length,
                        'total_path_quality_score':self.clearance_metrics['total_path_quality_score'],
                        'planner_expansions':self.path.expanded,'planner_time_s':self.path.elapsed}))
                    self.get_logger().info(f'Plan found: {len(self.path.states)} states, {self.path.expanded} expansions, {self.path.elapsed:.2f}s, length={self.path.length:.2f}m, beam_gap_fill={filled}')
                    self.publish_paths();self.transition('READY','survey path' if self.survey_plan else 'parking path')
                except Exception as exc:self.transition('FAILED',str(exc))
            self.publish_status();return
        if self.stage=='READY':
            self.send(0,0);self.visualize()
            if self.auto and self.get_parameter('execute_parking').value:
                if (self.pose_freshness_stop_enabled and
                    self.get_clock().now().nanoseconds*1e-9-self.est.pose_stamp>=.6):
                    self.publish_status();return
                if (now-self.map_wall>1.5 or now-self.scan_wall>1 or
                    now-self.local_status_wall>1.5):
                    self.transition('EMERGENCY_STOP','streams stale before execution');return
                ready_grid=self.grid(False)
                if not ready_grid.collision_free(s,self.footprints):
                    self.send(0,0);self.transition('EMERGENCY_STOP','current actual footprint overlaps latest RAW map before execution')
                    self.record_blocked(ready_grid,0,self.failure,decision='EMERGENCY_STOP');return
                if not self.target.contains(self.footprints.world(self.path.states[-1])):
                    self.transition('FAILED','final planned footprints outside measured target');return
                # Audit the WHOLE selected path immediately before EXECUTE on
                # the newest available RAW snapshot, never the planning grid.
                validation_invocation_ns=self.get_clock().now().nanoseconds
                ready_metrics=path_metrics(ready_grid,self.path.states,self.footprints)
                if not ready_metrics['valid'] or not ready_grid.check(s,self.footprints):
                    # Capture only on a real rejection. No decision, threshold, or command changes.
                    self.raw_reject_count+=1
                    context=self.plan_context
                    attempt_index=max(0,min(context['next_index']-1,len(context['attempts'])-1))
                    used_grid,candidate,shift,lateral=context['attempts'][attempt_index]
                    candidate_index=sum(1 for a in context['attempts'][:attempt_index+1] if a[0] is used_grid)
                    map_wall_age=time.monotonic()-self.map_wall
                    identity={'attempt_id':self.raw_diagnostic_episode,'candidate_id':candidate_index,
                        'map_variant':'filled' if context['filled'] is not None and used_grid is context['filled'] else 'strict',
                        'goal_shift_m':float(shift),'lateral_shift_m':float(lateral),
                        'goal_pose':[float(v) for v in candidate],
                        'candidate_start_pose':[float(v) for v in context['start']]}
                    timestamps={'validator_invocation_ns':validation_invocation_ns,
                        'capture_ns':self.get_clock().now().nanoseconds,
                        'pose_stamp_ns':self.est.pose_stamp or 0,
                        'scan_stamp_ns':self.latest_scan_stamp_ns,
                        'map_received_wall_age_s':map_wall_age,
                        'map_received_wall':self.map_wall}
                    previous=self.raw_map_history[-2] if len(self.raw_map_history)>=2 and self.raw_map_history[-1]['sequence']==ready_grid.map_sequence else (self.raw_map_history[-1] if self.raw_map_history else None)
                    event_id=f'event_{self.raw_reject_count:03d}'
                    try:
                        self.pending_raw_reject,overhead_ms=capture_reject(
                            self.get_parameter('blocked_debug_directory').value,event_id,
                            ready_grid,previous,s,self.path.states,self.footprints,identity,timestamps,
                            {'path_metrics_valid':bool(ready_metrics['valid']),
                             'current_pose_check':bool(ready_grid.check(s,self.footprints)),
                             'logged_path_minimum_m':ready_metrics['minimum_clearance_m']})
                        self.get_logger().info(f'[RAW_REJECT_CAPTURED] event={event_id} overhead_ms={overhead_ms:.3f}')
                    except Exception as exc:
                        self.get_logger().error(f'[RAW_REJECT_CAPTURE_FAILED] event={event_id}: {exc}')
                    self.reject_ready_candidate(
                        f"latest RAW full-path/current clearance audit failed: map_sequence={ready_grid.map_sequence}, "
                        f"minimum={ready_metrics['minimum_clearance_m']:.6f}m, hard={PLANNER_HARD_CLEARANCE_M:.2f}m (or footprint overlap)");return
                self.clearance_metrics.update(pre_execute_map_sequence=ready_grid.map_sequence,
                    pre_execute_map_stamp_ns=ready_grid.map_stamp_ns,
                    pre_execute_min_clearance_m=ready_metrics['minimum_clearance_m'])
                self.get_logger().info('[PRE_EXECUTE_VALIDATED] '+json.dumps(ready_metrics)+
                    f' map_sequence={ready_grid.map_sequence} map_stamp_ns={ready_grid.map_stamp_ns}')
                self.transition('EXECUTE')
            else:self.publish_status();return
        if self.stage=='EXECUTE':
            if self.pose_freshness_stop_required(
                self.get_clock().now().nanoseconds*1e-9-self.est.pose_stamp):
                self.send(0,0);self.transition('EMERGENCY_STOP','pose too old for closed-loop parking');return
            if (now-self.map_wall>1.5 or now-self.scan_wall>1 or
                now-self.local_status_wall>1.5):
                self.send(0,0);self.transition('EMERGENCY_STOP','localization/map stream stale');return
            live_grid=self.grid(False)
            # Actual current tractor/trailer overlap is NEVER debounced. The
            # zero command and emergency transition precede optional evidence.
            if not live_grid.collision_free(s,self.footprints):
                self.send(0,0);self.transition('EMERGENCY_STOP','current actual footprint overlaps latest RAW occupied/unknown cell')
                self.record_blocked(live_grid,self.controller.index,self.failure,decision='EMERGENCY_STOP');return
            v,delta,done=self.controller.command(s)
            sim_now=self.get_clock().now().nanoseconds*1e-9
            v,delta,guard=self.articulation_safety.project(s,v,delta,self.est.pose_stamp,sim_now)
            self.controller.last_delta=delta
            directory=self.get_parameter('blocked_debug_directory').value
            if directory:
                from pathlib import Path
                with (Path(directory)/'control_trace.jsonl').open('a') as f:
                    f.write(json.dumps(dict(time=sim_now,pose_stamp=self.est.pose_stamp,
                        pose_age_s=sim_now-self.est.pose_stamp,
                        pose_freshness_stop_enabled=self.pose_freshness_stop_enabled,
                        state=s.tolist(),reference=self.controller.last_reference.tolist(),
                        index=self.controller.index,v=v,delta=delta,
                        terms=self.controller.terms,guard=guard))+'\n')
            if guard['stop']:
                self.replan('recovery-safe articulation: '+guard['reason']);return
            self.max_tracking_error=max(self.max_tracking_error,self.controller.deviation)
            err=s-self.controller.last_reference;err[2:]=wrap(err[2:])
            deviation=self.controller.deviation>.7 or max(abs(err[2:]))>math.radians(25)
            if deviation:
                self.deviation_since=self.deviation_since or now
                if now-self.deviation_since>1.2:self.replan('tracking deviation');return
            else:self.deviation_since=None
            if done:
                self.send(0,0)
                if self.final_wait is None:self.final_wait=now
                if now-self.final_wait<2:return
                if self.survey_plan:
                    self.surveyed=True;self.settle_until=now+3;self.transition('DETECT_TARGET','survey complete; refine map target');return
                error=s-self.goal;error[2:]=wrap(error[2:])
                valid=(np.linalg.norm(error[:2])<.35 and max(abs(error[2:]))<math.radians(5)
                       and phi<math.radians(float(self.get_parameter("completion_hinge_deg").value)) and self.measured_speed<.06 and self.collision_count==0
                       and self.target.contains(self.footprints.world(s)))
                if valid:self.transition('PARKED','both bodies inside target, headings aligned, stationary');return
                # Keep zero commands while the original unchanged completion
                # predicates settle. Never move/replan from one noisy stopped
                # pose sample; retain a bounded8-second verification deadline.
                if now-self.final_wait<float(self.get_parameter('completion_settle_max_s').value):
                    self.publish_status();return
                self.final_wait=None
                self.replan(f'final tolerances not met: error={error}, speed={self.measured_speed:.3f}');return
            self.final_wait=None
            # Future path/prediction clearance is evidence, not an immediate
            # stop. Only a persistent block consumes the existing replan budget.
            if now-getattr(self,'last_safety',0)>.2:
                self.last_safety=now
                grid=live_grid;p=s.copy();predicted=[]
                for _ in range(15):
                    p=step(p,v,delta,.1,self.geometry)
                    predicted.append(p.copy())
                    if abs(wrap(p[3]-p[2]))>math.radians(35):
                        self.record_blocked(grid,self.controller.index,'predicted articulation exceeds existing safety check',predicted_states=predicted)
                        self.replan('predicted articulation exceeds existing safety check');return
                index=self.controller.index
                current_block=path_block_observation(grid,s[None,:],self.footprints,index,'current_clearance')
                remaining_block=path_block_observation(grid,self.path.states[index:],self.footprints,index)
                predicted_block=path_block_observation(grid,np.asarray(predicted),self.footprints,index,'predicted_footprint')
                observation=current_block or remaining_block or predicted_block
                if observation is not None:
                    observation.update(current_clearance_blocked=current_block is not None,
                        remaining_path_blocked=remaining_block is not None,
                        predicted_footprint_blocked=predicted_block is not None)
                checked_now=time.monotonic()
                if self.confirm_live_block(grid,observation,checked_now):
                    confirmation=self.live_block.snapshot(checked_now)
                    reason='CONFIRMED_PATH_BLOCK: '+observation['source']
                    self.send(0,0)
                    self.record_blocked(grid,index,reason,confirmation=confirmation,
                        predicted_states=predicted if observation['source']=='predicted_footprint' else None)
                    self.replan(reason);return
            self.send(v,delta)
        self.visualize();self.publish_status()

    def replan(self,reason):
        self.send(0,0);self.replans+=1;self.deviation_since=None
        if self.replans>int(self.get_parameter('max_replans').value):self.transition('FAILED',f'replan budget exhausted: {reason}');return
        self.transition('REPLAN',reason);self.launch_plan()

    def pose_msg(self,s):
        m=PoseStamped();m.header.frame_id='local_map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.position.x=float(s[0]);m.pose.position.y=float(s[1]);m.pose.orientation.z=math.sin(s[2]/2);m.pose.orientation.w=math.cos(s[2]/2)
        return m

    def publish_paths(self):
        directory=self.get_parameter('blocked_debug_directory').value
        if directory:
            from pathlib import Path
            out=Path(directory);out.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(out/f'planned_path_{self.replans+1:03d}.npz',states=self.path.states,
                gears=self.path.gears,steering=self.path.steering,goal=self.goal,
                planning_map_stamp_ns=self.planning_map_stamp_ns)
            (out/f'clearance_path_{self.replans+1:03d}.json').write_text(json.dumps(self.clearance_metrics,indent=2))
            latest=self.grid(False)
            np.savez_compressed(out/f'accepted_raw_grid_{self.replans+1:03d}.npz',
                data=latest.data,resolution=latest.resolution,origin=latest.origin,
                map_stamp_ns=latest.map_stamp_ns,current=self.current)
        for pub,trailer in [(self.tractor_path_pub,False),(self.trailer_path_pub,True)]:
            m=RosPath();m.header.frame_id='local_map';m.header.stamp=self.get_clock().now().to_msg()
            for s in self.path.states:
                xy=hitch(s,self.geometry) if trailer else s[:2];heading=s[3] if trailer else s[2]
                m.poses.append(self.pose_msg([*xy,heading]))
            pub.publish(m)

    def visualize(self):
        if self.current is None or time.monotonic()-getattr(self,'last_viz',0)<.4:return
        self.last_viz=time.monotonic();markers=[]
        def line(points,color,ns,number):
            m=Marker();m.header.frame_id='local_map';m.header.stamp=self.get_clock().now().to_msg();m.ns=ns;m.id=number;m.type=Marker.LINE_STRIP;m.action=Marker.ADD
            m.pose.orientation.w=1.;m.scale.x=.04;m.color.r,m.color.g,m.color.b=color;m.color.a=.85
            for p in np.vstack([points,points[0]]):m.points.append(Point(x=float(p[0]),y=float(p[1]),z=.08))
            markers.append(m)
        current_footprints=self.footprints.world(self.current)
        for i,p in enumerate(current_footprints):line(p,(0.,1.,.7),'current_body',i)
        if np.isfinite(self.current).all():
            # Reuse the estimated current footprint placement, not a planned
            # state or simulator pose. The vertex centroid stays inside the body.
            center=np.mean(current_footprints[1],axis=0)
            pose=self.pose_msg([*center,self.current[3]])
            m=Marker();m.header=pose.header;m.ns='trailer_pose_arrow';m.id=0;m.type=Marker.ARROW;m.action=Marker.ADD
            m.pose=pose.pose;m.pose.position.z=.15
            m.scale.x=.8;m.scale.y=.08;m.scale.z=.18
            m.color.r=1.;m.color.g=0.;m.color.b=0.;m.color.a=1.;markers.append(m)
        candidate=getattr(self,'last_gap_observation',None)
        if candidate is not None and candidate.target is not None:line(candidate.target.polygon(),(0.,1.,1.),'observed_candidate',0)
        if self.target is not None:
            line(self.target.polygon(),(1.,.8,0.),'slot',0)
            if self.goal is not None:self.target_pub.publish(self.pose_msg(self.goal))
        if self.path is not None:
            for j,s in enumerate(self.path.states[::15]):
                for i,p in enumerate(self.footprints.world(s)):line(p,(.3,.5,1.),'planned_footprints',2*j+i)
        if self.controller is not None:
            m=Marker();m.header.frame_id='local_map';m.ns='control_target';m.id=0;m.type=Marker.ARROW;m.pose=self.pose_msg(self.controller.last_reference).pose;m.scale.x=.6;m.scale.y=.1;m.scale.z=.1;m.color.r=1.;m.color.a=1.;markers.append(m)
        text=Marker();text.header.frame_id='local_map';text.ns='articulation';text.id=0;text.type=Marker.TEXT_VIEW_FACING
        text.pose.position.x=float(self.current[0]);text.pose.position.y=float(self.current[1]+1);text.pose.position.z=1.;text.pose.orientation.w=1.;text.scale.z=.4;text.color.r=text.color.g=text.color.b=text.color.a=1.
        text.text=f'{self.stage} phi={math.degrees(wrap(self.current[3]-self.current[2])):.1f} deg';markers.append(text)
        self.markers_pub.publish(MarkerArray(markers=markers))

    def publish_status(self,force=False):
        now=time.monotonic()
        if not force and now-self.last_log<2:return
        self.last_log=now
        info={'stamp_ns':self.get_clock().now().nanoseconds,'physical_parking_speed_factor':float(self.get_parameter('parking_speed_factor').value),'state':self.stage,'pose':self.current.tolist() if self.current is not None else None,
              'goal':self.goal.tolist() if self.goal is not None else None,'v':round(self.v,3),'steering_deg':round(math.degrees(self.delta),2),
              'omega':self.v/self.geometry.wheelbase*math.tan(self.delta),'progress':self.controller.index if self.controller else 0,
              'path_states':len(self.path.states) if self.path is not None else 0,'replans':self.replans,
              'max_articulation_deg':math.degrees(self.max_phi),'collisions':self.collision_count,
              'planning_seconds':self.planning_time,'planned_length':self.total_planned_length,'gear_changes':self.gear_changes,
              'planned_max_articulation_deg':float(np.rad2deg(np.max(np.abs(wrap(self.path.states[:,3]-self.path.states[:,2]))))) if self.path is not None else None,
              'planning_articulation_limit_deg':PLANNING_ARTICULATION_LIMIT_DEG,
              'speed_from_slam':self.measured_speed,'failure':self.failure,'estimator':self.est.last_error}
        info.update(first_obstacle=self.first_seen,gap_seen=self.gap_seen,
                    second_obstacle=self.second_seen,search_distance=self.search_distance,
                    max_tracking_error=self.max_tracking_error,
                    gap_center=self.target.center.tolist() if self.target is not None else None,
                    gap_confidence=self.target.confidence if self.target is not None else None,
                    gap_yaw=self.target.heading if self.target else None,gap_length=self.target.length if self.target else None,gap_width=self.target.width if self.target else None,target_source='observe_gap(Cartographer OccupancyGrid)')
        info['goal_shift_m']=self.goal_shift
        info['parking_stop_policy']=self.stop_policy
        info['legacy_stop_evidence']=getattr(self,'legacy_stop_evidence',None)
        info['legacy_stop_completed']=getattr(self,'legacy_stop_completed',None)
        info['goal_lateral_shift_m']=self.goal_lateral_shift
        info['planning_map_stamp_ns']=getattr(self,'planning_map_stamp_ns',None)
        info.update(map_age_s=now-self.map_wall,lidar_age_s=now-self.scan_wall,
                    tractor_imu_age_s=None,
                    trailer_imu_age_s=now-self.est.imu_wall,
                    localization_status_age_s=now-self.local_status_wall,
                    independent_observation_hits=self.observation_hits,
                    reference_state=self.controller.last_reference.tolist() if self.controller else None)
        info.update(pose_freshness_stop_enabled=self.pose_freshness_stop_enabled,
                    pose_freshness_threshold_s=POSE_FRESHNESS_THRESHOLD,
                    max_pose_age_s=self.max_pose_age,
                    pose_freshness_exceed_count=self.pose_freshness_exceed_count,
                    pose_age_samples=self.pose_age_samples)
        if self.target is not None:
            info.update(target_length=self.target.length,target_width=self.target.width,
                        target_heading=self.target.heading,
                        tractor_trailer_inside_target=self.target.contains(self.footprints.world(self.current)) if self.current is not None else False)
        if self.est.pose_stamp is not None:
            info['pose_age_s']=self.get_clock().now().nanoseconds*1e-9-self.est.pose_stamp
        obs=getattr(self,'last_gap_observation',None)
        if obs is not None:
            info['observed_intervals']=obs.obstacle_intervals
            info['gap_reason']=obs.reason
            info['candidate_center']=obs.target.center.tolist() if obs.target is not None else None
        if self.current is not None and self.goal is not None:
            info['position_error']=float(np.linalg.norm(self.current[:2]-self.goal[:2]));info['heading_errors_deg']=np.rad2deg(wrap(self.current[2:]-self.goal[2:])).tolist()
        msg=String();msg.data=json.dumps(info);self.status_pub.publish(msg);self.get_logger().info('[Parking] '+msg.data)


def main():
    rclpy.init();node=AutoParking()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.send(0,0)
        node.planner_pool.shutdown(wait=False,cancel_futures=True);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
