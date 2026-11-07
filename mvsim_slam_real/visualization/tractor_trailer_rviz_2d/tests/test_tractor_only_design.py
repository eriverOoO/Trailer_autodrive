"""Rendering-only regression checks. No ROS inputs are published."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from rclpy.time import Time
from visualization_msgs.msg import Marker
from tractor_trailer_rviz_2d.render_adapter import Renderer


@pytest.mark.parametrize('xy,yaw',[([0.,0.],0.),([2.,-3.],1.2)])
def test_original_tractor_vertices_and_style_without_any_trailer(xy,yaw):
    root=Path(__file__).resolve().parents[1]
    design=json.loads((root/'models/reference_footprints.json').read_text())
    node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=10**9)))
    renderer=Renderer(node,design['polygons'],[None,None])
    markers=renderer.render_tractor(xy,yaw,'DIAG phi=N/A\nSENSOR ANCHOR','map').markers
    assert len(markers)==3
    body=next(m for m in markers if m.ns=='current_body')
    assert body.id==0 and len(body.points)==11 and body.type==Marker.LINE_STRIP
    c,s=math.cos(yaw),math.sin(yaw)
    expected=np.asarray(design['polygons'][0])@np.array([[c,s],[-s,c]])+xy
    np.testing.assert_allclose([[p.x,p.y] for p in body.points],np.vstack([expected,expected[0]]))
    assert [body.color.r,body.color.g,body.color.b,body.color.a]==pytest.approx([0,1,.7,.85])
    assert body.scale.x==pytest.approx(.04)
    assert all(p.z==pytest.approx(.08) for p in body.points)
    arrow=next(m for m in markers if m.ns=='tractor_pose_arrow')
    assert arrow.type==Marker.ARROW
    assert [arrow.scale.x,arrow.scale.y,arrow.scale.z]==pytest.approx([.8,.08,.18])
    assert [arrow.color.r,arrow.color.g,arrow.color.b,arrow.color.a]==[1.,0.,0.,1.]
    assert [arrow.pose.position.x,arrow.pose.position.y]==pytest.approx(expected.mean(axis=0))
    assert all(m.header.frame_id=='map' for m in markers)
    assert not any('trailer' in m.ns or (m.ns=='current_body' and m.id==1) for m in markers)
    assert all(math.isfinite(v) for m in markers for v in [m.pose.position.x,m.pose.position.y,m.pose.orientation.z,m.pose.orientation.w])
    text=next(m for m in markers if m.type==Marker.TEXT_VIEW_FACING)
    assert 'phi=N/A' in text.text and 'nan' not in text.text.lower()
