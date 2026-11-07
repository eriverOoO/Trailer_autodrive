"""Software-only tests. All inputs here are synthetic, never hardware evidence."""
import math
import pytest
from tractor_trailer_rviz.core import YawTrack,Alignment,vehicle_poses,rpy_quat,wrap

G={'slam_to_rear_axle_xy':[0.,0.],'rear_axle_to_hitch_xy':[-.524,0.],
   'trailer_origin_to_hitch_xy':[0.,0.],'trailer_origin_to_axle_xy':[-1.33,0.]}


def push(t,n,angle):
    stamp=int(n*1e9)
    assert t.push(stamp,n,stamp,q=rpy_quat(0.,0.,math.radians(angle)))


def pair():
    a,b=YawTrack(),YawTrack()
    # Different absolute sensor heading must cancel under straight alignment.
    push(a,1,120);push(b,1,-40)
    align=Alignment();align.align(a,b,10**9,.7)
    return a,b,align


def test_independent_yaws_and_hitch():
    a,b,s=pair();push(a,1.1,150);push(b,1.1,-40)
    v=s.evaluate(a,b,1_100_000_000)
    assert v['tractor_delta']==pytest.approx(math.radians(30))
    assert v['trailer_yaw']==pytest.approx(.7)
    assert v['articulation']==pytest.approx(math.radians(30))
    p=vehicle_poses((3.,4.),1.8,v['tractor_yaw'],v['trailer_yaw'],G)
    assert p['hitch']==pytest.approx(p['trailer'])
    push(a,1.2,150);push(b,1.2,-20)
    v2=s.evaluate(a,b,1_200_000_000)
    assert v2['tractor_yaw']==pytest.approx(v['tractor_yaw'])
    assert v2['trailer_delta']==pytest.approx(math.radians(20))


def test_wrap_and_same_angle():
    a,b=YawTrack(),YawTrack();push(a,1,179);push(b,1,-179)
    s=Alignment();s.align(a,b,10**9,0)
    push(a,1.1,-179);push(b,1.1,-177)
    v=s.evaluate(a,b,1_100_000_000)
    assert math.degrees(v['tractor_delta'])==pytest.approx(2)
    assert v['articulation']==pytest.approx(0,abs=1e-10)


def test_timestamp_interpolation_not_latest_difference():
    a,b,s=pair();push(a,1.1,130);push(a,1.2,140);push(b,1.15,-25)
    v=s.evaluate(a,b,1_150_000_000)
    assert v['tractor_delta']==pytest.approx(math.radians(15))
    assert v['trailer_delta']==pytest.approx(math.radians(15))
    with pytest.raises(ValueError):a.at(1_250_000_000)


def test_stale_invalid_gap_requires_new_alignment():
    a,b,s=pair()
    assert a.state(1.6,1_600_000_000)=='STALE'
    push(a,1.7,130);push(b,1.7,-30)
    with pytest.raises(ValueError,match='ALIGNMENT LOST'):s.evaluate(a,b,1_700_000_000)
    assert not a.push(1_800_000_000,1.8,1_800_000_000,q=(0,0,0,0))
    assert a.state(1.8,1_800_000_000)=='INVALID'


def test_no_cartographer_yaw_replacement_and_lever_arm():
    a,b,s=pair();push(a,1.1,120);push(b,1.1,-40)
    v=s.evaluate(a,b,1_100_000_000)
    p1=vehicle_poses((0,0),0,v['tractor_yaw'],v['trailer_yaw'],G)
    p2=vehicle_poses((0,0),2.,v['tractor_yaw'],v['trailer_yaw'],G)
    assert p1==p2
    g=dict(G,slam_to_rear_axle_xy=[-.3,0.])
    assert vehicle_poses((0,0),math.pi/2,v['tractor_yaw'],v['trailer_yaw'],g)['tractor']==pytest.approx((0,-.3))


def test_general_trailer_hitch_offset():
    g=dict(G,trailer_origin_to_hitch_xy=[.4,.1])
    p=vehicle_poses((1,2),0,.4,-.2,g)
    from tractor_trailer_rviz.core import add,rot2
    assert add(p['trailer'],rot2(-.2,[.4,.1]))==pytest.approx(p['hitch'])


def test_mounted_sensor_and_explicit_gyro_relative_mode():
    t=YawTrack(mounting=(0,0,math.pi/2),sign=-1)
    push(t,1,100);push(t,1.1,120)
    assert t.at(1_100_000_000)-t.at(1_000_000_000)==pytest.approx(math.radians(-20))
    g=YawTrack(source='gyro_z',bias=.1)
    assert g.push(10**9,1.,10**9,gyro=(0,0,1.1))
    assert g.push(1_200_000_000,1.2,1_200_000_000,gyro=(0,0,1.1))
    assert g.value==pytest.approx(.2)
    assert not g.push(1_100_000_000,1.3,1_300_000_000,gyro=(0,0,1.1))


def test_orientation_unavailable_never_falls_back_to_gyro():
    t=YawTrack(source='orientation')
    assert not t.push(10**9,1.,10**9,q=None,gyro=(0,0,1))
    assert t.state(1,10**9)=='INVALID'
