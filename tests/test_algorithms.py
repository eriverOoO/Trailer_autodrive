from pathlib import Path
import math,json,time
import numpy as np
import pytest
from real_parking.bootstrap import install_b
mod=install_b()
from real_vehicle_bridge.geometry import VehicleGeometry
from tractor_trailer_model import Footprints,propagate,Grid
from tractor_parking_perception import planning_grid
from tractor_local_parking_gap_detector import observe_gap
from tractor_parking_controller import Controller
from tractor_parking_clearance import path_metrics,footprint_clearances
from native import clearances,propagate as native_propagate
from priority_schedule import schedule
W=Path(__file__).resolve().parents[1]

def rig():
    v=VehicleGeometry(W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml');g=v.model();return g,Footprints(*v.polygons(),g)
def open_grid():
    a=np.zeros((300,400),dtype=int);a[:2,:]=100;a[-2:,:]=100;a[:,:2]=100;a[:,-2:]=100
    return planning_grid(a,.05,[-5,-5,0],simulation_clearance=True)

def test_native_kinematics_and_clearance_match_python():
    g,fp=rig();start=np.array([0.,0.,.1,.05]);a=propagate(start,-.8,.2,g);b=native_propagate(start,-.8,.2,g)
    assert np.allclose(a,b,atol=1e-10);grid=open_grid()
    # Compare the native wrapper against the original function captured before patching.
    from native import original_clearances
    assert np.allclose(clearances(grid,a,fp),original_clearances(grid,a,fp),atol=1e-10)

def test_gap_boundary_measurement_and_internal_obstacle_reject():
    res=.05;a=np.zeros((200,300),dtype=int);origin=[-2.,-7.,0.]
    def rect(x0,x1,y0,y1):
        i0,i1=int((x0+2)/res),int((x1+2)/res);j0,j1=int((y0+7)/res),int((y1+7)/res);a[j0:j1,i0:i1]=100
    rect(0,2,-2.15,-2);rect(8,10,-2.15,-2);rect(1.85,2,-4.2,-2);rect(8,8.15,-4.2,-2);rect(0,10,-4.4,-4.2)
    obs=observe_gap(planning_grid(a,res,origin),np.array([0.,0.,0.]),6.2,2.1)
    assert obs.target is not None,obs.reason
    assert 5.7<obs.target.length<6.2 and 2.0<obs.target.width<2.4
    rect(4,5,-3.5,-2.8)
    assert observe_gap(planning_grid(a,res,origin),np.array([0.,0.,0.]),6.2,2.1).target is None

def test_b_planner_forward_reverse_and_invalid_start():
    g,fp=rig();grid=open_grid();s=np.array([0.,0.,0.,0.])
    for goal,gears in [(np.array([.8,0.,0.,0.]),(1,)),(np.array([-.8,0.,0.,0.]),(-1,))]:
        p=mod.HybridAStar(g,fp,grid,timeout=4.).plan(s,goal,gears)
        assert np.linalg.norm(p.states[-1,:2]-goal[:2])<.15
        assert np.max(abs(p.states[-1,2:]))<math.radians(2.5)
        assert path_metrics(grid,p.states,fp)['valid']
        assert np.isin(p.gears,gears).all()
    with pytest.raises(RuntimeError):mod.HybridAStar(g,fp,grid,timeout=1.).plan(np.array([100.,100.,0.,0.]),s)

def test_lqr_both_gears_and_completion():
    g,fp=rig();s=np.array([0.,0.,0.,0.])
    for gear in [1,-1]:
        states=np.vstack([s,propagate(s,gear*1.,0.,g)]);p=mod.Path(states,np.full(len(states),gear),np.zeros(len(states)));c=Controller(p,g,speed_factor=1.755)
        shifted=s.copy();shifted[1]=.01;v,d,done=c.command(shifted)
        assert np.isfinite([v,d]).all() and v*gear>0 and abs(d)<=g.max_steer and not done
        assert c.command(states[-1])[2]

def test_priority_rank_prefers_straight_and_preserved_cde_schedule(tmp_path,monkeypatch):
    g,fp=rig();grid=open_grid();s=np.array([0.,0.,0.,0.]);end=np.array([.8,0.,0.,0.]);states=np.vstack([s,propagate(s,.8,0.,g)])
    planner=mod.HybridAStar(g,fp,grid,timeout=4.);a=mod.Path(states,np.ones(len(states)),np.zeros(len(states)));b=mod.Path(states,np.ones(len(states)),np.full(len(states),.2))
    assert planner.priority_rank(a)<planner.priority_rank(b)
    budget=tmp_path/'budget.json';budget.write_text(json.dumps({'deadline_monotonic':time.monotonic()+600}));monkeypatch.setenv('RESEARCH_BUDGET_FILE',str(budget))
    # Small straight departure/approach fixture exercises actual C,D,E generator jobs.
    queue=[{'index':i,'map_variant':'raw','goal':[0.,0.,0.,0.],'staging_distances':[.8]} for i in range(3)]
    winner,jobs,events=schedule(mod,end,queue,{'raw':grid},g,fp,np.array([0.,1.]),0.,timeout=4.)
    assert winner is not None and len(jobs)==3 and all(j['status']=='FOUND' for j in jobs)
    assert {e['index'] for e in events}=={0,1,2}
