"""4D Hybrid A* with an off-axle numerical goal connection.

Concepts studied in HybridAStarTrailer (Atsushi Sakai, MIT) and Tracailer
(BSD-3-Clause). This is an independent implementation; no Julia/ROS1 code is
ported. Both headings are propagated and checked during every expansion,
including the numerical connector. No tractor-only Reeds-Shepp correction.
"""
from dataclasses import dataclass
import heapq
import json
import math
import time
import numpy as np
from scipy.optimize import least_squares
from tractor_trailer_model import wrap,step,propagate
from tractor_parking_clearance import (footprint_clearances,integrated_penalty,penalty_density,
                                      integrated_near_wall_cost,
                                      near_wall_density,minimum_clearance_density,
                                      PLANNER_HARD_CLEARANCE_M,
                                      CLEARANCE_EPS_M,NEAR_WALL_EXPOSURE_WEIGHT)


PLANNING_ARTICULATION_LIMIT_DEG = 30.0
CONNECTOR_PENALTY_START_DEG = 27.0

# Cost units are metres of forward travel. Physical/safety limits are separate.
W_DISTANCE = 1.0
W_STEER = .45
W_STEER_CHANGE = .40
W_STEER_REVERSAL = .25
W_ARTICULATION = .35
W_ARTICULATION_CHANGE = .10
STEERING_DEADBAND_RAD = math.radians(2.)


def preference_costs(states, steering, max_steer, articulation_limit,
                     previous_steering=0., last_active_steering=0.):
    """Incoming-edge controls; parent included, angles in radians.

    Variation is independent of sample count. Active sign survives zero steer.
    Future articulation comes entirely from the propagated four-dimensional path.
    """
    ds=np.linalg.norm(np.diff(states[:,:2],axis=0),axis=1)
    delta=np.asarray(steering)[1:]
    phi=wrap(states[:,2]-states[:,3])
    change=float(np.abs(np.diff(np.r_[previous_steering,delta])).sum())/max_steer
    active=delta[np.abs(delta)>STEERING_DEADBAND_RAD]
    if abs(last_active_steering)>STEERING_DEADBAND_RAD:active=np.r_[last_active_steering,active]
    reversals=int(np.count_nonzero(np.diff(np.sign(active))))
    return dict(distance=W_DISTANCE*float(ds.sum()),
        steering_magnitude=W_STEER*float(np.sum((delta/max_steer)**2*ds)),
        steering_change=W_STEER_CHANGE*change,
        steering_reversals=W_STEER_REVERSAL*reversals,
        articulation=W_ARTICULATION*float(np.sum(.5*((phi[:-1]/articulation_limit)**2+
                                                     (phi[1:]/articulation_limit)**2)*ds)),
        articulation_change=W_ARTICULATION_CHANGE*float(np.abs(wrap(np.diff(phi))).sum())/articulation_limit)


@dataclass
class Path:
    states: np.ndarray
    gears: np.ndarray
    steering: np.ndarray
    elapsed: float = 0.
    expanded: int = 0

    @property
    def length(self):
        return float(np.linalg.norm(np.diff(self.states[:,:2],axis=0),axis=1).sum())

    @property
    def gear_changes(self):
        return int(np.count_nonzero(np.diff(self.gears)))

    def reversed(self):
        # Gear and steering label the incoming edge of each state. Reversing
        # state order shifts both controls by one, including at gear changes.
        return Path(self.states[::-1].copy(),np.r_[-self.gears[-1],-self.gears[:0:-1]],
                    np.r_[self.steering[-1],self.steering[:0:-1]],self.elapsed,self.expanded)


def combine(a,b):
    gears=np.r_[a.gears,b.gears[1:]]
    steering=np.r_[a.steering,b.steering[1:]]
    if len(a.states)==1:
        # A zero-length root inherits the first moving segment's gear.
        gears[0]=b.gears[0];steering[0]=b.steering[0]
    return Path(np.vstack([a.states,b.states[1:]]),gears,steering,
                a.elapsed+b.elapsed,a.expanded+b.expanded)


class HybridAStar:
    def __init__(self,geometry,footprints,grid,xy_resolution=.25,yaw_resolution=math.radians(7.5),timeout=45.,
                 articulation_limit_deg=PLANNING_ARTICULATION_LIMIT_DEG,
                 connector_penalty_start_deg=CONNECTOR_PENALTY_START_DEG,
                 connector_penalty_weight=20.,primitive_length=.6):
        self.geometry=geometry;self.footprints=footprints;self.grid=grid
        self.xy_resolution=xy_resolution;self.yaw_resolution=yaw_resolution;self.timeout=timeout
        self.steers=np.deg2rad([-35,-25,-15,0,15,25,35])
        self.steers=np.clip(self.steers,-geometry.max_steer,geometry.max_steer)
        self.articulation_limit=math.radians(articulation_limit_deg)
        self.connector_penalty_start=math.radians(connector_penalty_start_deg)
        self.connector_penalty_weight=connector_penalty_weight
        self.expanded=0
        self.primitive_length=float(primitive_length)

    def path_free(self, states):
        # Apply the planning margin before the occupancy check. Grid's 35 deg
        # validity remains the independent runtime emergency boundary.
        if np.any(np.abs(wrap(states[:,3]-states[:,2]))>self.articulation_limit):
            return False
        if hasattr(self.grid,'validate_path'):
            valid,self._motion_clearances=self.grid.validate_path(states,self.footprints)
        else:
            valid=self.grid.path_free(states,self.footprints)
            if valid:self._motion_clearances=footprint_clearances(self.grid,states,self.footprints)
        return bool(valid and np.all(self._motion_clearances>
                                    PLANNER_HARD_CLEARANCE_M+CLEARANCE_EPS_M))

    def key(self,s,gear):
        # Four physical dimensions, plus gear for a well-defined switching cost.
        return (round(s[0]/self.xy_resolution),round(s[1]/self.xy_resolution),
                round(wrap(s[2])/self.yaw_resolution),round(wrap(s[3])/self.yaw_resolution),gear)

    def heuristic(self,s,goal):
        d=np.linalg.norm(s[:2]-goal[:2])
        # Weighted, non-admissible heuristic. No optimality claim.
        return d+1.8*abs(wrap(s[2]-goal[2]))+1.3*abs(wrap(s[3]-goal[3]))

    def rollout_controls(self,start,steers,length,gear,steps_per_segment=5):
        s=start.copy();out=[]
        ds=length/len(steers)/steps_per_segment
        for delta in steers:
            for _ in range(steps_per_segment):
                s=step(s,gear,float(delta),ds,self.geometry);out.append(s)
        return np.asarray(out)

    def connector(self,start,goal,gear,tries=2):
        dist=float(np.linalg.norm(start[:2]-goal[:2]))
        if dist<.05:return None
        direction=math.atan2(goal[1]-start[1],goal[0]-start[0])
        if abs(wrap(direction-start[2]-(math.pi if gear<0 else 0)))>1.5:
            return None
        n=10;limit=self.geometry.max_steer
        # Quintic lane-change seed; the optimizer constrains both final headings.
        lateral=-math.sin(start[2])*(goal[0]-start[0])+math.cos(start[2])*(goal[1]-start[1])
        seed=np.sin(np.linspace(0,2*math.pi,n))*np.clip(lateral/dist*.8,-.45,.45)
        for attempt in range(tries):
            u0=np.r_[seed*(1 if attempt==0 else 1.8),dist*(1.08 if attempt==0 else 1.25)]
            u0[:-1]=np.clip(u0[:-1],-limit+.001,limit-.001)
            def residual(u):
                p=self.rollout_controls(start,u[:-1],u[-1],gear)
                e=p[-1]-goal;e[2:]=wrap(e[2:])
                phi=np.abs(wrap(p[:,3]-p[:,2]))
                clearance=footprint_clearances(self.grid,p,self.footprints,smooth=True)
                ds=np.linalg.norm(np.diff(np.vstack([start[:2],p[:,:2]]),axis=0),axis=1)
                # Extend the existing solve's objective, preserving its seeds,
                # bounds, iteration budget and endpoint/feasibility checks.
                controls=np.repeat(u[:-1],5)
                signed_phi=wrap(np.r_[start[2]-start[3],p[:,2]-p[:,3]])
                changes=np.diff(np.r_[0.,u[:-1]])/limit
                phi_changes=wrap(np.diff(signed_phi))/self.articulation_limit
                # Smooth absolute-value surrogate for least_squares only.
                # Exact absolute variation and deadband reversals are used in g.
                eps=1e-4
                reversal=np.maximum(0.,-u[:-2]*u[1:-1]/limit**2)
                return np.r_[e*[12,12,18,18],.04*u[:-1],.10*np.diff(u[:-1]),
                             self.connector_penalty_weight*np.maximum(0,phi-self.connector_penalty_start),
                             np.sqrt(ds*penalty_density(clearance,optimization=True)),
                             np.sqrt(ds*NEAR_WALL_EXPOSURE_WEIGHT*
                                     near_wall_density(clearance)),
                             np.sqrt(ds*minimum_clearance_density(clearance)),
                             math.sqrt(float(minimum_clearance_density(clearance).max())*dist),
                             np.sqrt(W_STEER*ds)*controls/limit,
                             np.sqrt(W_STEER_CHANGE*(np.sqrt(changes**2+eps**2)-eps)),
                             np.sqrt(W_STEER_REVERSAL)*reversal,
                             np.sqrt(W_ARTICULATION*ds*.5)*signed_phi[:-1]/self.articulation_limit,
                             np.sqrt(W_ARTICULATION*ds*.5)*signed_phi[1:]/self.articulation_limit,
                             np.sqrt(W_ARTICULATION_CHANGE*(np.sqrt(phi_changes**2+eps**2)-eps)),
                             math.sqrt(W_DISTANCE*u[-1])]
            sol=least_squares(residual,u0,bounds=(np.r_[np.full(n,-limit),max(.02,dist*.95)],
                             np.r_[np.full(n,limit),max(dist*1.7,dist+2)]),max_nfev=65,ftol=.002,xtol=.002,gtol=.002)
            states=[start.copy()];angles=[sol.x[0]]
            for d in sol.x[:-1]:
                p=propagate(states[-1],gear*sol.x[-1]/n,d,self.geometry)
                states.extend(p);angles.extend([d]*len(p))
            p=np.asarray(states);err=p[-1]-goal;err[2:]=wrap(err[2:])
            if np.linalg.norm(err[:2])<.10 and max(abs(err[2:]))<math.radians(2) and self.path_free(p):
                return Path(p,np.full(len(p),gear),np.asarray(angles))
        return None

    def plan(self,start,goal,gears=(1,-1)):
        started=time.monotonic();start=np.asarray(start);goal=np.asarray(goal)
        if not self.path_free(start[None,:]):raise RuntimeError('Start footprint/articulation invalid')
        if not self.path_free(goal[None,:]):raise RuntimeError('Goal footprint/articulation invalid')
        # Last active steering is metadata only; the duplicate key is unchanged.
        # parent, state, incoming swept segment, gear, steer, g, last active steer
        nodes=[(-1,start,np.array([start]),gears[0],0.,0.,0.)]
        heap=[(self.heuristic(start,goal)*2.5,0)];best={self.key(start,gears[0]):0.}
        self.expanded=0
        while heap and time.monotonic()-started<self.timeout:
            _,idx=heapq.heappop(heap);parent,s,segment,gear,delta,cost,last_active=nodes[idx]
            if cost>best.get(self.key(s,gear),math.inf)+1e-6:continue
            self.expanded+=1
            if self.expanded==1 or (self.expanded%60==0 and np.linalg.norm(s[:2]-goal[:2])<9):
                for g in gears:
                    tail=self.connector(s,goal,g,tries=2 if self.expanded==1 else 1)
                    if tail:
                        result=combine(self.reconstruct(nodes,idx),tail)
                        result.elapsed=time.monotonic()-started;result.expanded=self.expanded;return result
            error=s-goal;error[2:]=wrap(error[2:])
            if np.linalg.norm(error[:2])<.15 and np.max(np.abs(error[2:]))<math.radians(2.5):
                result=self.reconstruct(nodes,idx);result.elapsed=time.monotonic()-started;result.expanded=self.expanded;return result
            for g in gears:
                for d in self.steers:
                    p=propagate(s,g*self.primitive_length,float(d),self.geometry)
                    if not self.path_free(p):continue
                    phi=abs(wrap(p[-1,3]-p[-1,2]))
                    clearance=float(self.grid.distances(p[-1,:2])[0])
                    extra=self.primitive_length*(1 if g>0 else 1.25)+(.8 if g!=gear else 0)+.12*abs(d)+.15*abs(d-delta)+.35*phi+.5*max(0,phi-math.radians(25))**2+.025/max(clearance,.1)
                    extra+=integrated_penalty(p,self._motion_clearances,start=s)
                    extra+=NEAR_WALL_EXPOSURE_WEIGHT*integrated_near_wall_cost(
                        p,self._motion_clearances,start=s)
                    extra+=self.primitive_length*float(
                        minimum_clearance_density(self._motion_clearances).max())
                    terms=preference_costs(np.vstack([s,p]),np.full(len(p)+1,d),
                        self.geometry.max_steer,self.articulation_limit,delta,last_active)
                    # Distance already occurs in the original forward/reverse
                    # term; scale it once, rather than adding a second copy.
                    extra+=(W_DISTANCE-1.)*self.primitive_length
                    extra+=sum(v for name,v in terms.items() if name!='distance')
                    value=cost+extra;k=self.key(p[-1],g)
                    if value>=best.get(k,math.inf):continue
                    active=float(d) if abs(d)>STEERING_DEADBAND_RAD else last_active
                    best[k]=value;nodes.append((idx,p[-1],p,g,float(d),value,active))
                    heapq.heappush(heap,(value+2.5*self.heuristic(p[-1],goal),len(nodes)-1))
        raise RuntimeError(f'4D Hybrid A* no solution: {self.expanded} expansions in {time.monotonic()-started:.1f}s')

    @staticmethod
    def reconstruct(nodes,idx):
        chunks=[];gears=[];angles=[]
        while idx>=0:
            parent,s,p,g,d,*_=nodes[idx]
            chunks.append(p);gears.append(np.full(len(p),g));angles.append(np.full(len(p),d));idx=parent
        return Path(np.vstack(chunks[::-1]),np.concatenate(gears[::-1]),np.concatenate(angles[::-1]))

    def parking_plan(self,start,goal,road_y=None,staging_distances=(8.,10.,12.)):
        """Plan a forward departure from the slot, reverse it, then plan approach.

        This is a reversible 4D boundary-value problem, not an open-loop
        manoeuvre script. All departure/approach states use both footprints and
        the same OccupancyGrid. The reverse segment necessarily occurs last.
        """
        c,s=math.cos(goal[2]),math.sin(goal[2]);along=np.array([c,s]);normal=np.array([-s,c])
        offset=float((start[:2]-goal[:2])@normal) if road_y is None else road_y
        errors=[]
        for ahead in staging_distances:
            staging=goal.copy();staging[:2]+=ahead*along+offset*normal
            try:
                # A narrow U cavity may require shunting before either body
                # clears a side solid. Keep both headings and the same 30 deg
                # envelope through every forward/reverse departure primitive.
                departure_planner=HybridAStar(self.geometry,self.footprints,self.grid,
                    xy_resolution=.10,yaw_resolution=math.radians(3),timeout=self.timeout,
                    articulation_limit_deg=math.degrees(self.articulation_limit),
                    primitive_length=.20)
                departure=departure_planner.plan(goal,staging,gears=(1,-1))
                approach=self.plan(start,departure.states[-1])
                result=combine(approach,departure.reversed())
                print('[PATH COST BREAKDOWN] '+json.dumps(self.cost_breakdown(result)),flush=True)
                return result
            except RuntimeError as exc:errors.append(str(exc))
        raise RuntimeError('; '.join(errors))

    def cost_breakdown(self,path):
        """Common sampled-path score, not a claim of exact lattice g + tail.

        Original search has endpoint/primitive costs and an unscored connector.
        Report its terms as densities at this planner's primitive length so
        baseline and modified returned paths can be compared on one scale.
        """
        result=preference_costs(path.states,path.steering,self.geometry.max_steer,self.articulation_limit)
        ds=np.linalg.norm(np.diff(path.states[:,:2],axis=0),axis=1)
        delta=path.steering[1:];phi=np.abs(wrap(path.states[1:,2]-path.states[1:,3]))
        gears=path.gears[1:];clearance=footprint_clearances(self.grid,path.states,self.footprints)
        center=self.grid.distances(path.states[1:,:2])
        result['existing_reverse_direction_cost']=float(.25*np.sum(ds[gears<0])+.8*np.count_nonzero(np.diff(gears)))
        result['existing_steering_articulation_cost']=float(np.sum(ds/self.primitive_length*(.12*np.abs(delta)+.35*phi+
            .5*np.maximum(0.,phi-math.radians(25))**2))+.15*np.abs(np.diff(np.r_[0.,delta])).sum())
        result['existing_clearance_cost']=float(np.sum(ds/self.primitive_length*.025/np.maximum(center,.1))+
            integrated_penalty(path.states,clearance)+NEAR_WALL_EXPOSURE_WEIGHT*integrated_near_wall_cost(path.states,clearance)+
            np.sum(ds*minimum_clearance_density(clearance[1:])))
        result['total']=sum(result.values())
        return result
