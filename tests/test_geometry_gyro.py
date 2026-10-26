from pathlib import Path
import math
import numpy as np
import pytest
from real_vehicle_bridge.geometry import VehicleGeometry
from real_vehicle_bridge.yaw_estimator import GyroYaw
W=Path(__file__).resolve().parents[1]

def test_reference_points_and_equal_footprints():
    g=VehicleGeometry(W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml');p0,p1=g.polygons()
    assert np.allclose(np.ptp(p0,axis=0),[.99,.55]);assert np.allclose(np.ptp(p1,axis=0),[.99,.55])
    assert np.allclose(g.tractor_axle([0,0],math.pi/2),[0,-.305])
    a=g.trailer_center([-.305,0,0,0]);b=g.trailer_center([-.305,0,0,math.pi/2])
    assert np.allclose(a,[-1.2,0]);assert np.allclose(b,[-.645,-.555])

def test_measured_hinge_gap_and_overall_length():
    g=VehicleGeometry(W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml')
    p0,p1=g.polygons()
    # Both rectangles in tractor-centre coordinates at straight alignment.
    p0=p0+[-.305,0]
    p1=p1+[g.values['hitch_position'][0],0]
    hinge=g.values['hitch_position'][0]
    assert g.values['tractor_rear_axle_position'][0]-p0[:,0].min() == pytest.approx(.19)
    assert g.values['tractor_steering_axis'][0]-g.values['tractor_rear_axle_position'][0] == pytest.approx(.55)
    assert g.hitch_offset == pytest.approx(.34)
    assert g.values['trailer_axle_position'][0] + .99/2 == pytest.approx(.19)
    assert g.values['hitch_to_axle_distance'] == pytest.approx(.86)
    assert p0[:,0].min()-hinge == pytest.approx(.15)
    assert hinge-p1[:,0].max() == pytest.approx(.06)
    assert p0[:,0].min()-p1[:,0].max() == pytest.approx(.21)
    assert p0[:,0].max()-p1[:,0].min() == pytest.approx(2.19)
    import yaml
    mount=yaml.safe_load((W/'src/mvsim_slam_real_bringup/config/g6_mount.yaml').read_text())
    assert mount['xyz'] == pytest.approx(g.values['lidar_mounting_offset'][:3])
    for role, sensor, front_distance in [('tractor','lidar_mounting_offset',.895),
                                          ('tractor','tractor_imu_offset',.223),
                                          ('trailer','trailer_imu_offset',.215)]:
        assert g.values[role+'_length']/2-g.values[sensor][0] == pytest.approx(front_distance)

@pytest.mark.parametrize('updates', [
    {'tractor_rear_to_hitch_distance': .105, 'trailer_front_to_hitch_distance': .105},
    {'trailer_front_to_hitch_distance': -.06},
    {'connection_length': float('nan')},
    {'trailer_front_to_hitch_distance': .07},
])
def test_inconsistent_hitch_geometry_rejected(tmp_path, updates):
    import yaml
    values=yaml.safe_load((W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml').read_text())
    values.update(updates)
    path=tmp_path/'geometry.yaml';path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError):VehicleGeometry(path)

def test_legacy_symmetric_hitch_file(tmp_path):
    import yaml
    values=yaml.safe_load((W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml').read_text())
    for key in ('tractor_rear_to_hitch_distance','trailer_front_to_hitch_distance'):
        del values[key]
    values.update(tractor_length=.97,trailer_length=.97,connection_length=.12,
                  hitch_position=[-.545,0.,0.],hitch_to_axle_distance=.835,
                  trailer_axle_position=[-.29,0.,0.])
    path=tmp_path/'legacy.yaml';path.write_text(yaml.safe_dump(values))
    assert VehicleGeometry(path).trailer_hitch_x == pytest.approx(.545)

def test_gyro_real_epoch_bias_wrap_and_no_orientation_use():
    g=GyroYaw();g.bias=.01;base=1_790_000_000*10**9
    for i in range(20):g.push(base+i*10_000_000,.11,-1)
    assert g.initialize(base+10_000_000,math.pi-.005)
    assert abs(g.at(base+110_000_000)-(-math.pi+.005))<1e-8
    assert abs(g.integral-.019)<1e-9 and not g.orientation_used

def test_gyro_gap_backward_and_nonfinite_stop():
    for second,value in [(40_000_000,.1),(0,.1),(10_000_000,float('nan'))]:
        g=GyroYaw();g.bias=0.;g.push(1_000_000_000,.1,-1);g.push(1_000_000_000+second,value,-1)
        assert g.fault and g.at(1_000_000_000) is None
