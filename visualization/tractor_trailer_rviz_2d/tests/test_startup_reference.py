"""Synthetic startup/motion cases; these do not establish physical angle accuracy."""
import math
import pytest
from tractor_trailer_rviz_2d.core import StationaryReference,Alignment,YawTrack,rpy_quat


def sample(ref,i,a=2.,b=-1.,x=0.,stationary=True):
    t=100.+i*.05
    return ref.push(round(t*1e9),t,a,b,x,0.,.4,stationary)


def test_five_second_average_and_independent_initial_headings():
    ref=StationaryReference()
    for i in range(100):
        assert sample(ref,i,a=2.+(.001 if i%2 else -.001)) is None
    result=sample(ref,100)
    assert result==pytest.approx((2.,-1.,.4),abs=1e-5)
    assert ref.progress==1.


def test_motion_and_missing_input_restart_full_window():
    ref=StationaryReference()
    for i in range(80):sample(ref,i)
    assert sample(ref,80,stationary=False) is None
    assert ref.progress==0
    for i in range(81,160):assert sample(ref,i) is None
    assert sample(ref,170) is None  # receive gap exceeds max_gap
    assert ref.progress==0
    assert len(ref.window)==1


@pytest.mark.parametrize('kwargs',[{'a':2.1},{'b':-1.1},{'x':.03}])
def test_slow_drift_restarts_window(kwargs):
    ref=StationaryReference()
    for i in range(60):sample(ref,i)
    assert sample(ref,60,**kwargs) is None
    assert ref.progress==0


def test_repeated_timer_tick_does_not_erase_stationary_samples():
    ref=StationaryReference()
    for i in range(80):
        sample(ref,i)
        sample(ref,i)
    assert len(ref.window)==80
    for i in range(80,101):result=sample(ref,i)
    assert result is not None


def test_reversed_mount_and_wrap_only_changes_are_used():
    a=YawTrack(sign=-1)
    b=YawTrack(sign=-1,mounting=(0,0,math.pi))
    for t,aa,bb in [(1.,179.,-21.),(1.1,-171.,-11.)]:
        stamp=round(t*1e9)
        assert a.push(stamp,t,stamp,q=rpy_quat(0,0,math.radians(aa)))
        assert b.push(stamp,t,stamp,q=rpy_quat(0,0,math.radians(bb)))
        if t==1.:
            alignment=Alignment();alignment.align(a,b,stamp,.7)
    result=alignment.evaluate(a,b,1_100_000_000)
    assert result['tractor_delta']==pytest.approx(math.radians(-10))
    assert result['trailer_delta']==pytest.approx(math.radians(-10))
    assert result['phi']==pytest.approx(0.,abs=1e-9)


def test_unphysical_orientation_jump_loses_reference():
    a=YawTrack(max_yaw_rate=math.radians(120))
    b=YawTrack()
    for track in (a,b):assert track.push(10**9,1.,10**9,q=rpy_quat(0,0,0))
    alignment=Alignment();alignment.align(a,b,10**9,0)
    assert not a.push(1_010_000_000,1.01,1_010_000_000,q=rpy_quat(0,0,1.))
    assert a.push(1_020_000_000,1.02,1_020_000_000,q=rpy_quat(0,0,1.))
    assert b.push(1_020_000_000,1.02,1_020_000_000,q=rpy_quat(0,0,0))
    with pytest.raises(ValueError,match='ALIGNMENT LOST'):
        alignment.evaluate(a,b,1_020_000_000)


def test_live_input_gate_and_locked_epoch_require_explicit_reset():
    from types import SimpleNamespace as NS
    from tractor_trailer_rviz_2d.visualizer import Vehicle2D
    from rclpy.time import Time
    a,b=YawTrack(),YawTrack()
    now=10**9
    for t in (a,b):assert t.push(now,1.,now,q=rpy_quat(0,0,0))
    obj=NS(scan_valid=False,scan_wall=1.,scan_stamp=now,scan_times=[1.]*5,
           map_valid=True,map_wall=1.,reference_locked=True,reference_epochs=(0,0),
           tracks={'tractor':a,'trailer':b},
           buffer=NS(lookup_transform=lambda *args:None),
           get_clock=lambda:NS(now=lambda:Time(nanoseconds=now)),frame='map')
    with pytest.raises(ValueError,match='LiDAR'):
        Vehicle2D.automatic_alignment(obj,now,None,1.)
    obj.scan_valid=True;obj.map_valid=False
    with pytest.raises(ValueError,match='SLAM map'):
        Vehicle2D.automatic_alignment(obj,now,None,1.)
    obj.map_valid=True
    Vehicle2D.automatic_alignment(obj,now,None,1.)
    a.reject('sensor reset')
    with pytest.raises(ValueError,match='REFERENCE LOST'):
        Vehicle2D.automatic_alignment(obj,now,None,1.)
    assert obj.reference_locked  # No silent zero while the vehicle might be moving.


def test_first_ready_sample_is_exactly_parallel_despite_averaging_noise():
    from types import SimpleNamespace as NS
    from tractor_trailer_rviz_2d.visualizer import Vehicle2D
    from rclpy.time import Time
    a,b=YawTrack(),YawTrack()
    stamp=10**9
    for track,yaw in ((a,.601),(b,-1.099)):
        assert track.push(stamp,1.,stamp,q=rpy_quat(0,0,yaw))
    alignment=Alignment()
    obj=NS(scan_valid=True,scan_wall=1.,scan_stamp=stamp,scan_times=[1.]*5,
           map_valid=True,map_wall=1.,reference_locked=False,reference_epochs=None,
           tracks={'tractor':a,'trailer':b},gyro_norms={'tractor':0.,'trailer':0.},gyro_limit=.1,
           buffer=NS(lookup_transform=lambda *args:None),
           get_clock=lambda:NS(now=lambda:Time(nanoseconds=stamp)),frame='map',
           reference=NS(push=lambda *args:(.600,-1.100,.0001)),
           align=alignment,geom={'slam_to_body_yaw_rad':0.},renderer=NS(last_viz=1.))
    pose=NS(transform=NS(translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    Vehicle2D.automatic_alignment(obj,stamp,pose,1.)
    result=alignment.evaluate(a,b,stamp)
    assert result['tractor_delta']==result['trailer_delta']==result['phi']==0.
    assert result['tractor_yaw']==result['trailer_yaw']


def test_first_markers_are_straight_preview_without_sensor_data():
    """SYNTHETIC_ONLY: actual renderer, no sensor or device process started."""
    from pathlib import Path
    import numpy as np
    import rclpy
    from rclpy.parameter import Parameter
    from tractor_trailer_rviz_2d.visualizer import Vehicle2D
    from std_msgs.msg import String, Bool
    from visualization_msgs.msg import MarkerArray
    config=Path(__file__).parents[1]/'config/relative_yaw.yaml'
    rclpy.init()
    node=Vehicle2D(parameter_overrides=[Parameter('config_file',value=str(config))])
    listener=rclpy.create_node('synthetic_only_startup_marker_observer')
    received={}
    listener.create_subscription(MarkerArray,'/vehicle_viz_2d/model',lambda msg:received.update(markers=msg),1)
    listener.create_subscription(Bool,'/vehicle_viz_2d/ready',lambda msg:received.update(ready=msg.data),1)
    listener.create_subscription(String,'/vehicle_viz_2d/status',lambda msg:received.update(status=msg.data),1)
    import time,json
    end=time.monotonic()+2
    try:
        while time.monotonic()<end and len(received)<3:
            rclpy.spin_once(node,timeout_sec=.01)
            rclpy.spin_once(listener,timeout_sec=.01)
        assert received['ready'] is False
        status=json.loads(received['status'])
        assert status['startup_pose_assumed'] is True
        assert status['valid'] is False
        bodies=[m for m in received['markers'].markers if m.ns=='current_body']
        assert len(bodies)==2
        centres=[]
        for marker in bodies:
            points=np.array([(p.x,p.y) for p in marker.points[:-1]])
            # Both unrotated measured bodies have these exact XY spans. The
            # legacy chamfer edges themselves are not all axis aligned.
            np.testing.assert_allclose(np.ptp(points,axis=0),[.99,.55],atol=1e-9)
            centres.append((points.min(axis=0)+points.max(axis=0))/2)
        assert centres[0][1]==pytest.approx(centres[1][1])
        arrow=next(m for m in received['markers'].markers if m.ns=='trailer_pose_arrow')
        assert arrow.pose.orientation.z==0.
        assert arrow.pose.orientation.w==1.
    finally:
        node.destroy_node();listener.destroy_node();rclpy.shutdown()
