import math,sys
import numpy as np
from common import *
def intersects(a,b):
 for p in [a,b]:
  e=np.roll(p,-1,axis=0)-p;norm=np.column_stack((-e[:,1],e[:,0]))
  for n in norm:
   pa=a@n;pb=b@n
   if pa.max()<pb.min()+1e-7 or pb.max()<pa.min()+1e-7:return False
 return True

def distance(a,b):
 if intersects(a,b):return 0.
 vals=[]
 for p,q in [(a,b),(b,a)]:
  starts=q;ends=np.roll(q,-1,axis=0);v=ends-starts
  for point in p:
   u=np.clip(((point-starts)*v).sum(1)/(v*v).sum(1),0,1);vals.extend(np.linalg.norm(starts+u[:,None]*v-point,axis=1))
 return min(vals)

def validate(run):
 g=read(ROOT/'active_world_geometry.json');path=np.load(run/'planned_path_001.npz')['states'];dense=[]
 for a,b in zip(path[:-1],path[1:]):
  d=b-a;d[2:]=np.arctan2(np.sin(d[2:]),np.cos(d[2:]));n=max(1,int(np.ceil(max(np.linalg.norm(d[:2])/.02,np.max(abs(d[2:]))/math.radians(.5)))))
  dense.extend(a+d*t/n for t in range(n))
 dense.append(path[-1]);walls=[v for v in g['fixtures'] if v['body_type']==0];fixtures=[v for v in g['fixtures'] if v['body_type']==2];mins={w['name']:999. for w in walls};coll=[]
 for i,state in enumerate(dense):
  x,y,a,ta=state;h=[x-.524*math.cos(a),y-.524*math.sin(a)]
  for v in fixtures:
   trailer=v['name'].startswith('trailer/');origin=[-2.524,0] if trailer else [-2,0];angle=ta if trailer else a;target=h if trailer else [x,y];c,s=math.cos(angle),math.sin(angle);poly=(np.array(v['vertices'])-origin)@np.array([[c,s],[-s,c]])+target
   for w in walls:
    d=distance(poly,np.array(w['vertices'])+[2,0]);mins[w['name']]=min(mins[w['name']],d)
    if d<1e-7:coll.append([i,v['name'],w['name']])
 hinge=float(np.max(abs(np.arctan2(np.sin(path[:,2]-path[:,3]),np.cos(path[:,2]-path[:,3]))))*180/np.pi)
 result={'status':'PASS' if not coll and hinge<=30.001 and min(mins.values())>=.18 else 'FAIL','path_nodes':len(path),'dense_poses':len(dense),'translation_step_max_m':.02,'angle_step_max_deg':.5,'minimum_clearance_m':mins,'collisions':coll,'maximum_hinge_deg':hinge,'world_to_map_fixed_translation':[2,0],'geometry_source':'loaded Box2D fixtures; chassis and all wheels; initial XML reference only','run':str(run),'configuration':read(ROOT/'evidence/validated_configuration.json')}
 write(ROOT/'parking_path_validation.json',result);write(ROOT/'evidence/gate5.json',result);print(result)
if __name__=='__main__':validate(Path(sys.argv[1]))
