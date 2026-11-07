"""Independent V2 variants. Search state, primitive spacing and safety constants unchanged."""
import numpy as np,math

def install(mod,variant):
 names=variant.split('+')
 if 'motion' in names:
  from native import propagate
  mod.propagate=propagate
 if 'clearance' in names:
  from native import clearances
  import tractor_parking_clearance as clear
  clear.footprint_clearances=clearances;mod.footprint_clearances=clearances
 if 'shot' in names:
  install_shot(mod)
 if 'resume' in names or 'resume20' in names:
  mod._resume_checkpoints=(5.,20.,185.) if 'resume20' in names else (5.,40.,185.)
  from resumable import install_resume
  install_resume(mod)
 unknown=set(names)-{'motion','clearance','shot','resume','resume20'}
 if unknown:raise ValueError(unknown)

def install_shot(mod):
 original=mod.HybridAStar.connector
 def connector(self,start,goal,gear,tries=2):
  dist=float(np.linalg.norm(start[:2]-goal[:2]));direction=math.atan2(goal[1]-start[1],goal[0]-start[0])
  if dist<.05 or abs(mod.wrap(direction-start[2]-(math.pi if gear<0 else 0)))>1.5:return None
  # Fast 4D boundary-value shot. Same ten controls, bounds, RK4 and full final checks.
  # A valid shortcut is scored independently; an invalid shortcut retains the original solver fallback.
  n=10;limit=self.geometry.max_steer;lateral=-math.sin(start[2])*(goal[0]-start[0])+math.cos(start[2])*(goal[1]-start[1])
  seed=np.sin(np.linspace(0,2*math.pi,n))*np.clip(lateral/dist*.8,-.45,.45);u0=np.r_[np.clip(seed,-limit+.001,limit-.001),dist*1.08]
  def residual(u):
   p=self.rollout_controls(start,u[:-1],u[-1],gear);e=p[-1]-goal;e[2:]=mod.wrap(e[2:]);phi=np.abs(mod.wrap(p[:,3]-p[:,2]))
   return np.r_[e*[12,12,18,18],.04*u[:-1],.10*np.diff(u[:-1]),self.connector_penalty_weight*np.maximum(0,phi-self.connector_penalty_start)]
  sol=mod.least_squares(residual,u0,bounds=(np.r_[np.full(n,-limit),max(.02,dist*.95)],np.r_[np.full(n,limit),max(dist*1.7,dist+2)]),max_nfev=25,ftol=.002,xtol=.002,gtol=.002)
  states=[start.copy()];angles=[sol.x[0]]
  for d in sol.x[:-1]:
   p=mod.propagate(states[-1],gear*sol.x[-1]/n,d,self.geometry);states.extend(p);angles.extend([d]*len(p))
  p=np.asarray(states);err=p[-1]-goal;err[2:]=mod.wrap(err[2:])
  if np.linalg.norm(err[:2])<.10 and max(abs(err[2:]))<math.radians(2) and self.path_free(p):
   self.shot_successes=getattr(self,'shot_successes',0)+1
   return mod.Path(p,np.full(len(p),gear),np.asarray(angles))
  self.shot_failures=getattr(self,'shot_failures',0)+1
  return original(self,start,goal,gear,tries)
 mod.HybridAStar.connector=connector
