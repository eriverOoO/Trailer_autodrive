"""Exact AutoParking.visualize() method from final B, without the state machine.
Only geometry, messages and time are imported; no controller is constructed.
"""
import math,time
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from .reference_math import wrap

class ReferenceRenderer:
    def visualize(self):
        if self.current is None or time.monotonic()-getattr(self,'last_viz',0)<.4:return
        self.last_viz=time.monotonic();markers=[]
        def line(points,color,ns,number):
            m=Marker();m.header.frame_id='local_map';m.header.stamp=self.get_clock().now().to_msg();m.ns=ns;m.id=number;m.type=Marker.LINE_STRIP;m.action=Marker.ADD
            m.pose.orientation.w=1.;m.scale.x=.04;m.color.r,m.color.g,m.color.b=color;m.color.a=.85
            for p in np.vstack([points,points[0]]):m.points.append(Point(x=float(p[0]),y=float(p[1]),z=.08))
            markers.append(m)
        current_footprints=self.footprints.world(self.current)
        for i,p in enumerate(current_footprints):line(p,(0.,1.,.7),'current_body',i)
        if np.isfinite(self.current).all():
            # Reuse the estimated current footprint placement, not a planned
            # state or simulator pose. The vertex centroid stays inside the body.
            center=np.mean(current_footprints[1],axis=0)
            pose=self.pose_msg([*center,self.current[3]])
            m=Marker();m.header=pose.header;m.ns='trailer_pose_arrow';m.id=0;m.type=Marker.ARROW;m.action=Marker.ADD
            m.pose=pose.pose;m.pose.position.z=.15
            m.scale.x=.8;m.scale.y=.08;m.scale.z=.18
            m.color.r=1.;m.color.g=0.;m.color.b=0.;m.color.a=1.;markers.append(m)
        candidate=getattr(self,'last_gap_observation',None)
        if candidate is not None and candidate.target is not None:line(candidate.target.polygon(),(0.,1.,1.),'observed_candidate',0)
        if self.target is not None:
            line(self.target.polygon(),(1.,.8,0.),'slot',0)
            if self.goal is not None:self.target_pub.publish(self.pose_msg(self.goal))
        if self.path is not None:
            for j,s in enumerate(self.path.states[::15]):
                for i,p in enumerate(self.footprints.world(s)):line(p,(.3,.5,1.),'planned_footprints',2*j+i)
        if self.controller is not None:
            m=Marker();m.header.frame_id='local_map';m.ns='control_target';m.id=0;m.type=Marker.ARROW;m.pose=self.pose_msg(self.controller.last_reference).pose;m.scale.x=.6;m.scale.y=.1;m.scale.z=.1;m.color.r=1.;m.color.a=1.;markers.append(m)
        text=Marker();text.header.frame_id='local_map';text.ns='articulation';text.id=0;text.type=Marker.TEXT_VIEW_FACING
        text.pose.position.x=float(self.current[0]);text.pose.position.y=float(self.current[1]+1);text.pose.position.z=1.;text.pose.orientation.w=1.;text.scale.z=.4;text.color.r=text.color.g=text.color.b=text.color.a=1.
        text.text=f'{self.stage} phi={math.degrees(wrap(self.current[3]-self.current[2])):.1f} deg';markers.append(text)
        self.markers_pub.publish(MarkerArray(markers=markers))
