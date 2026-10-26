"""Sensor-only stop qualification. Never reads World bay coordinates or GT."""
import math
import numpy as np
from tractor_trailer_model import transform,step,wrap
from tractor_parking_clearance import path_metrics

def observed_interior(grid,target):
 # Same .18m interior sampling offset as observe_gap; require every sampled
 # interior cell to be observed free, without converting unknown to free.
 u=np.arange(-target.length/2+.18,target.length/2-.15,grid.resolution)
 v=np.arange(-target.width/2+.18,target.width/2-.15,grid.resolution)
 if not len(u) or not len(v):return {'complete':False,'reason':'empty interior'}
 uu,vv=np.meshgrid(u,v);xy=transform(np.c_[uu.ravel(),vv.ravel()],target.center,target.heading);ij=grid.world_to_cell(xy)
 inside=(ij[:,0]>=0)&(ij[:,1]>=0)&(ij[:,0]<grid.width)&(ij[:,1]<grid.height);values=np.full(len(ij),-1,dtype=int);values[inside]=grid.data[ij[inside,1],ij[inside,0]]
 free=(values>=0)&(values<50)
 return {'complete':bool(np.all(free)),'sampled_cells':len(values),'free_fraction':float(np.mean(free)),'unknown_cells':int(np.count_nonzero(values<0)),'occupied_cells':int(np.count_nonzero(values>=50)),'boundary_basis':'current observe_gap validated both inner end faces and rear wall, lengths/width; this additionally requires full sampled interior observed free'}

def safe_stop_sweep(grid,state,footprints,geometry,speed,steer):
 # Existing runtime prediction horizon1.5s; conservatively keep the full speed
 # over the horizon instead of assuming immediate braking. This is a spatial
 # safety envelope, never an instruction to drive this distance.
 speed=abs(float(speed));states=[np.asarray(state,dtype=float).copy()]
 if abs(wrap(state[3]-state[2]))>=math.radians(30):return {'safe':False,'reason':'planning articulation bound','speed_bound_m_s':speed}
 for delta in sorted(set([float(steer),0.,-math.radians(8),math.radians(8)])):
  p=np.asarray(state,dtype=float).copy()
  for _ in range(30):
   p=step(p,speed,delta,.05,geometry);states.append(p.copy())
 states=np.asarray(states)
 articulation=np.max(abs(wrap(states[:,3]-states[:,2])))
 metrics=path_metrics(grid,states,footprints)
 safe=articulation<math.radians(30) and grid.path_free(states,footprints) and metrics['valid']
 return {'safe':bool(safe),'horizon_s':1.5,'speed_bound_m_s':speed,'distance_envelope_m':speed*1.5,'max_predicted_articulation_deg':float(np.degrees(articulation)),'minimum_clearance_m':float(metrics['minimum_clearance_m']),'swept_samples':len(states),'reason':'full tractor/trailer swept footprint and unchanged hard clearance' if safe else 'unsafe or unknown swept footprint'}

def observed_staging_distance(grid,goal,road_coordinate,footprints,max_ahead=7.):
 """Farthest observed-safe endpoint within existing staging extent.
 A geometric endpoint candidate, not an instruction to drive a fixed distance.
 The 4D departure/approach searches still validate every intervening state.
 """
 along=np.array([math.cos(goal[2]),math.sin(goal[2])]);normal=np.r_[-along[1],along[0]]
 offset=road_coordinate-float(np.asarray(goal[:2])@normal)
 spacing=max(float(grid.resolution),.1)
 for ahead in np.arange(max_ahead,0.,-spacing):
  p=np.asarray(goal,dtype=float).copy();p[:2]+=float(ahead)*along+offset*normal
  if grid.check(p,footprints):return float(ahead)
 return None
