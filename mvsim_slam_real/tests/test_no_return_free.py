import math
import numpy as np
import pytest
from real_vehicle_bridge.free_space import NoReturnFreeLayer

INF=float('inf');NAN=float('nan');RES=.1

def grid(w=60,h=60,value=-1):return np.full((h,w),value,dtype=np.int16)
def free_cells(data):return {(int(i),int(j)) for j,i in zip(*np.nonzero(data==0))}

def test_free_only_up_to_configured_range():
    # One +inf beam along +x from (0.02, 0.05) in a grid whose origin is (-1, -1): x 0.07..0.97.
    layer=NoReturnFreeLayer(RES,1.);layer.add_scan([INF],0.,.01,0.,[.02,.05,0.])
    data=layer.apply(grid(),[-1.,-1.,0.]);cells=free_cells(data)
    assert cells=={(i,10) for i in range(10,20)}
    assert data[10,20]==-1 and data[10,25]==-1

def test_nan_beams_finite_returns_and_range_min_ignored():
    layer=NoReturnFreeLayer(RES,2.)
    assert layer.add_scan([NAN,1.5,NAN],0.,.5,0.,[0.,0.,0.])==0 and len(layer)==0
    layer.add_scan([INF],0.,.01,.6,[.02,.05,0.]);data=layer.apply(grid(),[-1.,-1.,0.])
    xs=sorted(i for i,_ in free_cells(data))
    assert xs[0]==16 and xs[-1]==29 and (data[10,10:16]==-1).all()

def test_occupied_cells_kept_and_cells_behind_stay_unknown():
    data=grid();data[10,15]=100;data[10,13]=30
    occupied=data>=50;origin=[-1.,-1.,0.]
    layer=NoReturnFreeLayer(RES,2.);layer.add_scan([INF],0.,.01,0.,[.02,.05,0.],occupied,origin)
    out=layer.apply(data.copy(),origin)
    assert out[10,15]==100 and out[10,13]==30
    assert (out[10,10:13]==0).all() and out[10,14]==0
    assert (out[10,16:]==-1).all()
    # Even a cell accumulated earlier without the occupied grid is never overwritten if now occupied.
    free=NoReturnFreeLayer(RES,2.);free.add_scan([INF],0.,.01,0.,[.02,.05,0.])
    assert free.apply(data.copy(),origin)[10,15]==100

def test_accumulates_across_poses_and_survives_map_origin_change():
    layer=NoReturnFreeLayer(RES,1.)
    layer.add_scan([INF],0.,.01,0.,[.02,.05,0.])
    layer.add_scan([INF],0.,.01,0.,[.02,1.05,0.])
    a=free_cells(layer.apply(grid(),[-1.,-1.,0.]))
    assert {j for _,j in a}=={10,20}
    # Map grew: new origin (-2, -3) and bigger grid. Same world cells must be free.
    b=free_cells(layer.apply(grid(100,100),[-2.,-3.,0.]))
    assert b=={(i+10,j+20) for i,j in a}
    # Rotated grid origin (1, -1, 90 deg): grid i = world y + 1, grid j = 1 - world x.
    c=layer.apply(grid(),[1.,-1.,math.pi/2])
    assert free_cells(c)=={(10,j) for j in range(10)}|{(20,j) for j in range(10)}

def test_sensor_yaw_is_applied():
    layer=NoReturnFreeLayer(RES,1.);layer.add_scan([INF],0.,.01,0.,[.05,.02,math.pi/2])
    cells=free_cells(layer.apply(grid(),[-1.,-1.,0.]))
    assert cells=={(10,j) for j in range(10,20)}
    # angle_min/increment also rotate: beam index 1 of [nan, inf] at angle_min=-pi/2 with step pi/2 → +x.
    layer=NoReturnFreeLayer(RES,1.);layer.add_scan([NAN,INF],-math.pi/2,math.pi/2,0.,[.02,.05,0.])
    assert {j for _,j in free_cells(layer.apply(grid(),[-1.,-1.,0.]))}=={10}

def test_zero_range_disables_layer():
    layer=NoReturnFreeLayer(RES,0.)
    assert layer.add_scan([INF]*10,0.,.1,0.,[0.,0.,0.])==0 and len(layer)==0
    assert (layer.apply(grid(),[-1.,-1.,0.])==-1).all()

@pytest.mark.parametrize('resolution,free_range',[(0.,1.),(-.1,1.),(NAN,1.),(.1,-1.),(.1,INF)])
def test_invalid_layer_parameters_rejected(resolution,free_range):
    with pytest.raises(ValueError):NoReturnFreeLayer(resolution,free_range)

@pytest.mark.parametrize('pose',[[NAN,0.,0.],[0.,INF,0.],[0.,0.,NAN],[0.,0.]])
def test_invalid_pose_rejected(pose):
    with pytest.raises(ValueError):NoReturnFreeLayer(RES,1.).add_scan([INF],0.,.1,0.,pose)

def test_invalid_origin_rejected():
    layer=NoReturnFreeLayer(RES,1.);layer.add_scan([INF],0.,.1,0.,[0.,0.,0.])
    with pytest.raises(ValueError):layer.apply(grid(),[NAN,0.,0.])
