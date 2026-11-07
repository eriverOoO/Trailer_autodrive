"""B-only lexicographic path objective; unchanged dynamics, primitives and hard checks."""
import math,time,heapq,numpy as np
LOOKAHEAD_EXPANSIONS=60  # One existing connector cadence; bounded by original internal90s.
DEADBAND=math.radians(2.)
# Sustained large articulation/steering: quadratic in excess, linear in distance.
ARTICULATION_SOFT=math.radians(20.);ARTICULATION_REF=math.radians(30.)
REVERSE_ARTICULATION_WEIGHT=2.  # Execution-direction reverse; the trailer leads.
FLIP_WEIGHT=.25  # Same scale as the original W_STEER_REVERSAL.
# v: 0 peak steer,1 peak change,2 total variation,3 flips,4 curvature TV,5 |steer| integral,
#    6 -min clearance,7 near-wall distance,8 length,9 gear changes,10 articulation excess,11 steer^2 integral
ZERO=(0.,0.,0.,0.,0.,0.,-math.inf,0.,0.,0.,0.,0.)

def extend(v,states,steers,gears,geo,clearance,previous=0.,last_active=0.,previous_gear=1,execution_sign=1):
 ds=np.linalg.norm(np.diff(states[:,:2],axis=0),axis=1);u=np.asarray(steers)[1:];limit=geo.max_steer;ref=geo.wheelbase+geo.hitch_offset+geo.trailer_length
 change=np.abs(np.diff(np.r_[previous,u]));k=np.tan(np.r_[previous,u])/geo.wheelbase;active=u[np.abs(u)>DEADBAND]
 signs=np.r_[last_active,active] if abs(last_active)>DEADBAND else active
 flips=int(np.count_nonzero(np.diff(np.sign(signs))))
 gs=np.r_[previous_gear,np.asarray(gears)[1:]]
 # A departure search runs opposite to execution; execution_sign restores the real gear.
 phi=np.abs(np.arctan2(np.sin(states[1:,2]-states[1:,3]),np.cos(states[1:,2]-states[1:,3])))
 excess=np.maximum(0.,(phi-ARTICULATION_SOFT)/(ARTICULATION_REF-ARTICULATION_SOFT))**2
 weight=np.where(execution_sign*np.asarray(gears)[1:]<0,REVERSE_ARTICULATION_WEIGHT,1.)
 return (max(v[0],float(np.max(abs(u),initial=0.))/limit),max(v[1],float(np.max(change,initial=0.))/(2*limit)),v[2]+float(change.sum())/(2*limit),v[3]+FLIP_WEIGHT*flips,v[4]+float(np.abs(np.diff(k)).sum())/(2*math.tan(limit)/geo.wheelbase),v[5]+float(np.sum(abs(u)*ds))/(limit*ref),max(v[6],-float(np.min(clearance))),v[7]+float(np.sum(ds*(np.asarray(clearance)[1:]<.30))),v[8]+float(ds.sum()),v[9]+int(np.count_nonzero(np.diff(gs))),v[10]+float(np.sum(weight*excess*ds))/ref,v[11]+float(np.sum((u/limit)**2*ds))/ref)

def smoothness(v):return round(sum(v[:6])+v[10]+v[11],12)

# Safety margin, then gear changes, ahead of near-wall distance and length.
def key(v):return (smoothness(v),v[6],v[9],v[7],v[8])

def vector_for_path(path,planner):
 c=planner._priority_mod.footprint_clearances(planner.grid,path.states,planner.footprints)
 return extend(ZERO,path.states,path.steering,path.gears,planner.geometry,c,previous=0.,previous_gear=int(path.gears[0]),execution_sign=getattr(planner,'_execution_gear_sign',1))

def rank(path,planner):return key(vector_for_path(path,planner))

def lower_bound(v,s,goal,geo):
 # From |delta| >= cos(maxdelta)^2 |tan(delta)| and |dyaw|<=integral|kappa|ds.
 # Goal tolerances match original plan() and are subtracted for a valid bound.
 angular=max(0.,abs(math.atan2(math.sin(s[2]-goal[2]),math.cos(s[2]-goal[2])))-math.radians(2.5))
 h=geo.wheelbase/(geo.wheelbase+geo.hitch_offset+geo.trailer_length)*math.cos(geo.max_steer)**2/geo.max_steer*angular
 # Added terms are nonnegative, so h stays a valid bound on the steering sum.
 return (round(smoothness(v)+h,12),v[6],v[9],v[7],v[8]+max(0.,float(np.linalg.norm(s[:2]-goal[:2]))-.15))

def install(mod):
 cls=mod.HybridAStar;cls._priority_mod=mod
 def plan_steps(self,start,goal,gears=(1,-1)):
  self.active_s=0.;self.paused_s=0.;started=time.monotonic();start=np.asarray(start);goal=np.asarray(goal)
  if not self.path_free(start[None,:]):raise RuntimeError('Start footprint/articulation invalid')
  root_c=float(mod.footprint_clearances(self.grid,start[None,:],self.footprints).min())
  if not self.path_free(goal[None,:]):raise RuntimeError('Goal footprint/articulation invalid')
  root=ZERO[:6]+(-root_c,)+ZERO[7:];nodes=[(-1,start,np.array([start]),gears[0],0.,root,0.)];self.expanded=0
  def statekey(s,g,d,active):return (*self.key(s,g),round(float(d),8),int(np.sign(active)))
  labels={statekey(start,gears[0],0.,0.):[root]};heap=[(*lower_bound(root,start,goal,self.geometry),0)]
  incumbent=None;inc_key=None;finish_at=None
  while heap and time.monotonic()-started-self.paused_s<self.timeout:
   self.active_s=time.monotonic()-started-self.paused_s;pause=time.monotonic();yield None;self.paused_s+=time.monotonic()-pause
   *_,idx=heapq.heappop(heap);parent,s,segment,gear,delta,v,last_active=nodes[idx]
   if v not in labels.get(statekey(s,gear,delta,last_active),[]):continue
   self.expanded+=1
   if self.expanded==1 or (self.expanded%60==0 and np.linalg.norm(s[:2]-goal[:2])<9):
    for g in gears:
     tail=self.connector(s,goal,g,tries=2 if self.expanded==1 else 1)
     if tail:
      candidate=mod.combine(self.reconstruct(nodes,idx),tail);score=rank(candidate,self)
      if incumbent is None or score<inc_key:incumbent=candidate;inc_key=score
      if finish_at is None:finish_at=self.expanded+LOOKAHEAD_EXPANSIONS
   error=s-goal;error[2:]=mod.wrap(error[2:])
   if np.linalg.norm(error[:2])<.15 and np.max(abs(error[2:]))<math.radians(2.5):
    candidate=self.reconstruct(nodes,idx);score=rank(candidate,self)
    if incumbent is None or score<inc_key:incumbent=candidate;inc_key=score
    if finish_at is None:finish_at=self.expanded+LOOKAHEAD_EXPANSIONS
   if incumbent is not None and (self.expanded>=finish_at or (heap and tuple(heap[0][:-1])>=inc_key)):
    incumbent.elapsed=time.monotonic()-started-self.paused_s;incumbent.expanded=self.expanded;return incumbent
   for g in gears:
    for d in self.steers:
     p=mod.propagate(s,g*self.primitive_length,float(d),self.geometry)
     if not self.path_free(p):continue
     states=np.vstack([s,p]);value=extend(v,states,np.full(len(states),d),np.full(len(states),g),self.geometry,np.r_[ -v[6],self._motion_clearances],delta,last_active,gear,getattr(self,'_execution_gear_sign',1))
     active=float(d) if abs(d)>DEADBAND else last_active;k=statekey(p[-1],g,d,active);old=labels.get(k,[])
     # Componentwise resource labels: do not discard lower-peak/higher-TV history
     # merely on scalar g; future max/min extension could otherwise reverse rank.
     if any(all(a<=b+1e-12 for a,b in zip(q,value)) for q in old):continue
     labels[k]=[q for q in old if not all(a<=b+1e-12 for a,b in zip(value,q))]+[value]
     nodes.append((idx,p[-1],p,g,float(d),value,active));heapq.heappush(heap,(*lower_bound(value,p[-1],goal,self.geometry),len(nodes)-1))
  if incumbent is not None:
   incumbent.elapsed=time.monotonic()-started-self.paused_s;incumbent.expanded=self.expanded;return incumbent
  raise RuntimeError(f'4D steering-priority no solution: {self.expanded} expansions in {time.monotonic()-started-self.paused_s:.1f}s')
 cls.plan_steps=plan_steps
 cls.priority_rank=lambda self,path:rank(path,self)
 mod._steering_priority=True
