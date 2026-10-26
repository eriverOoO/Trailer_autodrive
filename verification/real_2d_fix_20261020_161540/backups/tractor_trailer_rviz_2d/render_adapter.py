"""Wrap the unchanged final-B Marker method; no planner, controller or TF writer."""
import copy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from .reference_renderer import ReferenceRenderer


class WorldFootprints:
    def __init__(self,polygons,rear_axle_to_hitch):
        self.polygons=[np.asarray(p,dtype=float) for p in polygons]
        self.offset=rear_axle_to_hitch

    def world(self,state):
        # Same final-B transform formula; actual hitch may have lateral offset.
        def transform(points,xy,yaw):
            c,s=math.cos(yaw),math.sin(yaw)
            return np.asarray(points)@np.array([[c,s],[-s,c]])+xy
        h=transform([self.offset],state[:2],state[2])[0]
        return [transform(self.polygons[0],state[:2],state[2]),
                transform(self.polygons[1],h,state[3])]


class Capture:
    def __init__(self):self.last=None
    def publish(self,msg):self.last=msg


class Renderer(ReferenceRenderer):
    def __init__(self,node,polygons,offset):
        self.node=node
        self.current=None
        self.footprints=WorldFootprints(polygons,offset)
        self.target=self.goal=self.path=self.controller=None
        self.markers_pub=Capture()
        self.stage='STATE_UNKNOWN'

    def get_clock(self):return self.node.get_clock()

    def pose_msg(self,s):
        # Same final-B pose_msg formula. Frames are adapted after rendering.
        m=PoseStamped();m.header.frame_id='local_map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.position.x=float(s[0]);m.pose.position.y=float(s[1])
        m.pose.orientation.z=math.sin(s[2]/2);m.pose.orientation.w=math.cos(s[2]/2)
        return m

    def render(self,state,stage,frame):
        self.current=np.asarray(state,dtype=float)
        self.stage=stage
        self.visualize()
        batch=copy.deepcopy(self.markers_pub.last)
        if batch is not None:
            for m in batch.markers:
                # Original local_map is an identity alias of map. Writing directly
                # in map avoids publishing even that extra identity TF.
                m.header.frame_id=frame
                m.lifetime.sec=0;m.lifetime.nanosec=800_000_000
        return batch


def unavailable(node,frame,reason,xy=(10.,0.),held=None,test=False):
    markers=[]
    if held:
        for m in copy.deepcopy(held.markers):
            if m.ns!='articulation':markers.append(m)
    text=Marker();text.header.frame_id=frame;text.header.stamp=node.get_clock().now().to_msg()
    text.ns='articulation';text.id=0;text.type=Marker.TEXT_VIEW_FACING
    text.pose.position.x=float(xy[0]);text.pose.position.y=float(xy[1]+1.)
    text.pose.position.z=1.;text.pose.orientation.w=1.;text.scale.z=.4
    text.color.r=text.color.g=text.color.b=text.color.a=1.
    text.text=('TEST/SIMULATED ' if test else '')+reason
    if held:text.text+=' LAST_POSE_NOT_LIVE'
    markers.append(text)
    for m in markers:
        m.header.frame_id=frame;m.lifetime.sec=0;m.lifetime.nanosec=800_000_000
    return MarkerArray(markers=markers)
