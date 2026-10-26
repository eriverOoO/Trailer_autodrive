"""Geometry-only source extraction from final B; no dynamics/controller import."""
from dataclasses import dataclass
import math,re
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull

def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

@dataclass(frozen=True)
class Geometry:
    wheelbase: float = 1.13
    hitch_offset: float = 0.524
    trailer_length: float = 1.33
    max_steer: float = math.radians(35)
    tractor_wheels: tuple = ()
    trailer_wheels: tuple = ()

    @classmethod
    def from_world(cls, filename):
        # MVSim uses literal colon names without XML namespaces.
        text = open(filename, encoding='utf8').read()
        root = ET.fromstring(re.sub(r'(<\/?)(block|vehicle):class', r'\1\2_class', text))
        tractor = root.find("vehicle_class[@name='tractor_class']/dynamics")
        trailer = root.find("vehicle_class[@name='trailer_class']/dynamics")
        joint = root.find("joint[@body_a='tractor'][@body_b='trailer']")
        rear_x = float(tractor.find('rl_wheel').get('pos').split()[0])
        anchor = [float(x) for x in joint.get('anchor_a').split()]
        anchor_b = [float(x) for x in joint.get('anchor_b').split()]
        if abs(rear_x) > 1e-6 or anchor[1] != 0 or anchor_b != [0., 0.]:
            raise ValueError('This controller requires the verified rear-axle/hitch frame convention')
        wheel_points=[]
        for dyn,names in [(tractor,['rl_wheel','rr_wheel','fl_wheel','fr_wheel']),
                          (trailer,['l_wheel','r_wheel'])]:
            pts=[]
            for name in names:
                w=dyn.find(name)
                if w.get('pos'):
                    x,y=map(float,w.get('pos').split())
                else:
                    x=float(dyn.findtext('f_wheels_x'));y=float(dyn.findtext('f_wheels_d'))/2*(1 if name=='fl_wheel' else -1)
                dx=float(w.get('diameter'))/2;dy=float(w.get('width'))/2
                corners=np.array([[-dx,-dy],[-dx,dy],[dx,dy],[dx,-dy]])
                for angle in ([-.9,0,.9] if name.startswith(('fl','fr')) else [0]):
                    pts.extend(transform(corners,[x,y],angle).tolist())
            wheel_points.append(tuple(map(tuple,pts)))
        return cls(float(tractor.findtext('f_wheels_x')) - rear_x,
                   -anchor[0], -float(trailer.find('l_wheel').get('pos').split()[0]),
                   math.radians(float(tractor.findtext('max_steer_ang_deg'))),*wheel_points)

def transform(points, xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + xy

def hitch(state, geometry=Geometry()):
    return np.asarray(state[:2]) - geometry.hitch_offset*np.array([math.cos(state[2]), math.sin(state[2])])

def inside(points, polygon):
    """Ray crossing including a small edge tolerance is handled by grid inflation."""
    pts=np.asarray(points); p=np.asarray(polygon); mask=np.zeros(len(pts),dtype=bool)
    x,y=pts.T
    for a,b in zip(p,np.roll(p,-1,axis=0)):
        if abs(b[1]-a[1]) < 1e-12:
            continue
        mask ^= ((a[1]>y)!=(b[1]>y)) & (x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])
    return mask

class Footprints:
    """Use actual latched chassis polygons, conservatively including wheel envelope."""
    def __init__(self, tractor, trailer, geometry=Geometry()):
        self.geometry=geometry
        self.polygons=[np.asarray(tractor,dtype=float),np.asarray(trailer,dtype=float)]
        if any(len(p)<3 or not np.isfinite(p).all() for p in self.polygons):
            raise ValueError('Invalid chassis polygons')
        # The chassis topic omits wheels: include their XML dimensions and the
        # front steering sweep. A convex hull is a conservative collision body.
        for i,wheels in enumerate([geometry.tractor_wheels,geometry.trailer_wheels]):
            if wheels:
                p=np.vstack([self.polygons[i],wheels])
                self.polygons[i]=p[ConvexHull(p).vertices]
        self.samples=[]
        for p in self.polygons:
            p=p[ConvexHull(p).vertices]
            lo=p.min(axis=0);hi=p.max(axis=0)
            xx,yy=np.meshgrid(np.arange(lo[0],hi[0]+.08,.08),np.arange(lo[1],hi[1]+.08,.08))
            xy=np.c_[xx.ravel(),yy.ravel()]
            edges=[]
            for a,b in zip(p,np.roll(p,-1,axis=0)):
                edges.extend(np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.07)+1)))
            self.samples.append(np.vstack([xy[inside(xy,p)],edges]))
        allp=np.vstack([self.polygons[0],self.polygons[1]-[geometry.hitch_offset,0]])
        self.center_offset=(allp.max(axis=0)+allp.min(axis=0))/2
        self.size=allp.max(axis=0)-allp.min(axis=0)

    def world(self,state):
        return [transform(self.polygons[0],state[:2],state[2]),
                transform(self.polygons[1],hitch(state,self.geometry),state[3])]

    def goal(self,center,heading):
        xy=np.asarray(center)-transform([self.center_offset],[0,0],heading)[0]
        return np.r_[xy,heading,heading]
