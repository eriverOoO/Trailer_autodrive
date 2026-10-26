"""Offline independent convex-versus-rectangle full body clearance, physical GT only."""
import sys,math,pandas as pd,numpy as np
from common import *
sys.path.insert(0,str(ROOT/'vendor'))
from tractor_trailer_model import Geometry,Footprints

def clearance(polys,lo,hi):
 edges=np.roll(polys,-1,axis=1)-polys;normal=np.stack((-edges[:,:,1],edges[:,:,0]),axis=2);rect=np.array([[lo[0],lo[1]],[hi[0],lo[1]],hi,[lo[0],hi[1]]])
 pp=np.einsum('nvi,nei->nev',polys,normal);rp=np.einsum('vi,nei->nev',rect,normal);separated=((pp.max(2)<rp.min(2)-1e-9)|(rp.max(2)<pp.min(2)-1e-9)).any(1);separated|=((polys.max(1)<lo)|(polys.min(1)>hi)).any(1)
 a=np.linalg.norm(polys-np.clip(polys,lo,hi),axis=2).min(1);q=rect[None,None,:,:]-polys[:,:,None,:];v=edges[:,:,None,:];u=np.clip((q*v).sum(3)/np.maximum((v*v).sum(3),1e-20),0,1);b=np.linalg.norm(q-u[:,:,:,None]*v,axis=3).min((1,2));return np.where(separated,np.minimum(a,b),0.)

def evaluate(run):
 if not (run/'run_status.json').exists():raise RuntimeError('Offline only')
 gt=pd.read_csv(run/'physics_gt.csv').dropna().iloc[::10];g=read(ROOT/'active_world_geometry.json');geo=Geometry.from_world(ROOT/'config/rect.world.xml');shapes={v['name']:np.array(v['vertices']) for v in g['fixtures']};fp=Footprints(shapes['tractor/chassis']-[-2,0],shapes['trailer/chassis']-[-2.524,0],geo);minima={};maxmove=0.
 for name,poly in zip(['tractor','trailer'],fp.polygons):
  yaw=gt['yaw' if name=='tractor' else 'trailer_yaw'].values;origin=gt[['x','y'] if name=='tractor' else ['trailer_x','trailer_y']].values;c=np.cos(yaw)[:,None];s=np.sin(yaw)[:,None];p=np.stack((c*poly[:,0]-s*poly[:,1]+origin[:,None,0],s*poly[:,0]+c*poly[:,1]+origin[:,None,1]),axis=2);maxmove=max(maxmove,float(np.linalg.norm(np.diff(p,axis=0),axis=2).max()))
  for wall in [v for v in read(run/'initial_geometry.json')['height_filtered_shapes'] if not v['name'].startswith(('tractor/','trailer/'))]:
   vertices=np.array(wall['vertices']);d=clearance(p,vertices.min(0),vertices.max(0));minima[name+'/'+wall['name']]=float(d.min())
 raw=min(minima.values());conservative=raw-maxmove;result={'status':'PASS' if conservative>=.18 else 'FAIL','sampled_minimum_clearance_m':raw,'maximum_surface_point_step_m':maxmove,'conservative_clearance_after_step_allowance_m':conservative,'hard_clearance_m':.18,'per_body_wall_minimum_m':minima,'samples':len(gt),'sample_dt_s':.05,'method':'Independent SAT and vertex/edge distances using conservative full chassis+XML steering wheel envelopes at actual GT poses; native5ms contact log remains primary collision evidence. subtract maximum sampled full-body surface displacement as interpolation allowance. GT never used online.'};write(run/'offline_physical_clearance.json',result);return result
if __name__=='__main__':print(evaluate(Path(sys.argv[1])))
