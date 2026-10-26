"""Compare the port with the actual final-B source/vertices, not a new design."""
import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from rclpy.time import Time
from visualization_msgs.msg import Marker
from tractor_trailer_rviz_2d.reference_renderer import ReferenceRenderer
from tractor_trailer_rviz_2d.render_adapter import Renderer,WorldFootprints,unavailable
from tractor_trailer_rviz_2d.reference_geometry import Geometry,Footprints

ROOT=Path(__file__).resolve().parents[1]
NODE=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=1_000_000_000)))


def reference():
    g=Geometry.from_world(str(ROOT/'models/original_rect.world.xml'))
    raw=json.loads((ROOT/'models/active_world_geometry.json').read_text())
    polys=[]
    for name,origin in [('tractor',[-2.,0.]),('trailer',[-2.524,0.])]:
        polys.append(np.asarray(next(f for f in raw['fixtures'] if f['name']==name+'/chassis')['vertices'])-origin)
    return Footprints(*polys,g)


def test_exact_visualize_method_preserved():
    assert inspect.getsource(ReferenceRenderer.visualize)==(ROOT/'models/original_visualize_method.txt').read_text()


def test_exact_original_hull_vertices():
    design=json.loads((ROOT/'models/reference_footprints.json').read_text())
    f=reference()
    assert [len(p) for p in f.polygons]==[10,9]
    for original,copied in zip(f.polygons,design['polygons']):np.testing.assert_array_equal(original,copied)


def test_exact_marker_style_positions_and_phi_sign():
    f=reference();s=np.array([2.,3.,.4,.1])
    r=Renderer(NODE,[p.tolist() for p in f.polygons],[-.524,0.])
    arr=r.render(s,'SEARCH_FORWARD','map')
    assert len(arr.markers)==4
    for i,m in enumerate(arr.markers[:2]):
        assert m.type==Marker.LINE_STRIP and m.ns=='current_body' and m.id==i
        assert (m.color.r,m.color.g,m.color.b,m.color.a)==pytest.approx((0,1,.7,.85))
        assert m.scale.x==pytest.approx(.04)
        expected=f.world(s)[i]
        np.testing.assert_allclose([[p.x,p.y] for p in m.points],np.vstack([expected,expected[0]]),atol=1e-12)
        assert all(p.z==pytest.approx(.08) for p in m.points)
        assert m.header.frame_id=='map'
    arrow,text=arr.markers[2:]
    assert arrow.ns=='trailer_pose_arrow' and arrow.type==Marker.ARROW
    center=f.world(s)[1].mean(axis=0)
    assert [arrow.pose.position.x,arrow.pose.position.y,arrow.pose.position.z]==pytest.approx([*center,.15])
    assert [arrow.scale.x,arrow.scale.y,arrow.scale.z]==pytest.approx([.8,.08,.18])
    assert (arrow.color.r,arrow.color.g,arrow.color.b,arrow.color.a)==(1.,0.,0.,1.)
    assert text.type==Marker.TEXT_VIEW_FACING and text.scale.z==pytest.approx(.4)
    assert (text.pose.position.x,text.pose.position.y,text.pose.position.z)==pytest.approx((2.,4.,1.))
    assert text.text=='SEARCH_FORWARD phi=-17.2 deg'
    assert not any(m.type in (Marker.MESH_RESOURCE,Marker.CUBE) for m in arr.markers)


def test_missing_data_hides_phi_and_keeps_explicit_stale_label():
    f=reference();r=Renderer(NODE,[p.tolist() for p in f.polygons],[-.524,0.])
    held=r.render([0,0,.1,.2],'SEARCH_FORWARD','map')
    a=unavailable(NODE,'map','T:STALE R:LIVE',held=held)
    text=next(m for m in a.markers if m.ns=='articulation')
    assert 'phi=' not in text.text and 'LAST_POSE_NOT_LIVE' in text.text
    assert 'SEARCH_FORWARD' not in text.text
    assert len(unavailable(NODE,'map','NO_DATA').markers)==1


def test_default_rviz_exact_map_scan_camera_and_no_3d_topic():
    import yaml
    original=yaml.safe_load((ROOT/'models/original_slam.rviz').read_text())['Visualization Manager']
    live=yaml.safe_load((ROOT/'rviz/live.rviz').read_text())['Visualization Manager']
    assert live['Views']==original['Views']
    assert live['Global Options']==original['Global Options']
    for kind in ('Grid','Map','LaserScan','TF'):
        def display(v):return next(d for d in v['Displays'] if d['Class']=='rviz_default_plugins/'+kind)
        assert display(live)==display(original)
    topics=[d.get('Topic',{}).get('Value') for d in live['Displays']]
    assert '/vehicle_viz/model' not in topics
    assert '/vehicle_viz_2d/model' in topics
    assert not any(d['Class']=='rviz_default_plugins/RobotModel' for d in live['Displays'])
