"""Display validity and unknown-extrinsics regressions; publishes nothing."""
from types import SimpleNamespace as NS
import pytest
from tractor_trailer_rviz_2d.core import YawTrack,rpy_quat
from tractor_trailer_rviz_2d.visualizer import Vehicle2D


def case():
    track=YawTrack()
    track.push(10**9,1.,10**9,q=rpy_quat(0,0,.5))
    track.push(1_100_000_000,1.1,1_100_000_000,q=rpy_quat(0,0,.6))
    p=NS(header=NS(stamp=NS(sec=1,nanosec=100_000_000)),
         transform=NS(translation=NS(x=3.,y=4.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    obj=NS(tracks={'tractor':track},source_fault={'tractor':''},get_slam=lambda:p,
           buffer=NS(lookup_transform=lambda *args:p),frame='map',slam_frame='laser_frame',
           cfg={'imu_timeout_s':.5,'pose_timeout_s':.5,
                'tractor_diagnostic':{'allow_sensor_anchor':True,'extrinsics_confirmed':False},
                'tractor':{'mounting_and_sign_confirmed':False}},
           diagnostic_anchor=None,visualization_mode='TRACTOR_ONLY_DIAGNOSTIC',
           renderer=NS(render_tractor=lambda *args:NS(markers=[])))
    return obj,{'imu':{'tractor':'LIVE','trailer':'NO DATA'}}


def test_diagnostic_needs_no_trailer_and_never_claims_full_vehicle_validity():
    obj,info=case()
    Vehicle2D.tractor_diagnostic(obj,info,1_100_000_000,1.1)
    assert info['diagnostic_valid'] and not info['valid'] and not info['full_vehicle_valid']
    assert not info['vehicle_pose_valid'] and not info['heading_alignment_verified']
    assert info['position_reference']=='UNMEASURED_SENSOR_ANCHOR'
    assert info['display_anchor_xy']==(3.,4.)
    assert info['values']['tractor_yaw'] is None
    assert info['values']['trailer_yaw'] is None and info['values']['phi'] is None


def test_unknown_extrinsics_require_explicit_diagnostic_opt_in():
    obj,info=case();obj.cfg['tractor_diagnostic']['allow_sensor_anchor']=False
    with pytest.raises(ValueError,match='measure tractor extrinsics'):
        Vehicle2D.tractor_diagnostic(obj,info,1_100_000_000,1.1)


def test_stale_tractor_does_not_create_live_display():
    obj,info=case()
    with pytest.raises(ValueError,match='tractor IMU must be LIVE'):
        Vehicle2D.tractor_diagnostic(obj,info,1_700_000_000,1.7)


def test_full_mode_alignment_still_requires_measured_geometry_and_both_mounts():
    obj=NS(geometry_error='unmeasured geometry')
    with pytest.raises(ValueError,match='unmeasured geometry'):Vehicle2D.require_alignment(obj)
    obj.geometry_error=''
    obj.cfg={'tractor':{'mounting_and_sign_confirmed':True,'source':'orientation'},
             'trailer':{'mounting_and_sign_confirmed':False,'source':'orientation'}}
    with pytest.raises(ValueError,match='trailer mounting/sign unverified'):
        Vehicle2D.require_alignment(obj)
