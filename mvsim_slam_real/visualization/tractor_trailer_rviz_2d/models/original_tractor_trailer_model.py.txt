"""Off-axle tractor/trailer model and occupancy collision geometry.

Original Python implementation. Algorithm references (read-only local copies):
Atsushi Sakai, HybridAStarTrailer (MIT); Tracailer (BSD-3-Clause).
The latter's Ltail/Lhead recursion agrees with the hitch velocity derivation.
State is [rear_axle_x, rear_axle_y, tractor_yaw, trailer_yaw], metres/radians.
"""
from dataclasses import dataclass
import math
import re
import xml.etree.ElementTree as ET
import numpy as np
from scipy.ndimage import distance_transform_edt
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


def derivative(state, v, delta, geometry=Geometry()):
    """Ph=P0-a e0; projecting Ph_dot on trailer normal gives yaw1_dot.

    Ph_dot = v e0 - a yaw0_dot n0. Therefore
    yaw1_dot = (v sin(yaw0-yaw1) - a yaw0_dot cos(yaw0-yaw1))/L1.
    Signed v handles reversing; steering has the same convention in both gears.
    """
    t0, t1 = state[2], state[3]
    w = v * math.tan(delta) / geometry.wheelbase
    b = t0 - t1
    return np.array([v * math.cos(t0), v * math.sin(t0), w,
                     (v * math.sin(b) - geometry.hitch_offset * w * math.cos(b)) /
                     geometry.trailer_length])


def step(state, v, delta, dt, geometry=Geometry()):
    s = np.asarray(state, dtype=float)
    k1 = derivative(s, v, delta, geometry)
    k2 = derivative(s + dt*k1/2, v, delta, geometry)
    k3 = derivative(s + dt*k2/2, v, delta, geometry)
    k4 = derivative(s + dt*k3, v, delta, geometry)
    out = s + dt*(k1 + 2*k2 + 2*k3 + k4)/6
    out[2:] = wrap(out[2:])
    return out


def propagate(state, distance, delta, geometry=Geometry(), ds=0.075):
    n = max(1, int(math.ceil(abs(distance)/ds)))
    s = np.asarray(state, dtype=float)
    path = []
    for _ in range(n):
        s = step(s, math.copysign(1., distance), delta, abs(distance)/n, geometry)
        path.append(s)
    return np.asarray(path)


def transform(points, xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + xy


def hitch(state, geometry=Geometry()):
    return np.asarray(state[:2]) - geometry.hitch_offset*np.array([math.cos(state[2]), math.sin(state[2])])


def trailer_axle(state, geometry=Geometry()):
    return hitch(state, geometry) - geometry.trailer_length*np.array([math.cos(state[3]), math.sin(state[3])])


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


class Grid:
    """Full OccupancyGrid origin rotation; outside/unknown are occupied by default."""
    def __init__(self, data, resolution, origin, threshold=50, unknown_occupied=True, margin=.12):
        self.data=np.asarray(data,dtype=np.int16);self.resolution=float(resolution)
        self.origin=np.asarray(origin,dtype=float);self.margin=margin
        self.occupied=self.data>=threshold
        self.blocked=self.occupied | ((self.data<0) if unknown_occupied else False)
        self.clearance=distance_transform_edt(~self.blocked)*self.resolution
        self.height,self.width=self.data.shape

    def world_to_cell(self, xy):
        p=transform(np.atleast_2d(xy)-self.origin[:2],[0,0],-self.origin[2])
        return np.floor(p/self.resolution).astype(int)

    def cell_to_world(self, ij):
        return transform((np.atleast_2d(ij)+.5)*self.resolution,self.origin[:2],self.origin[2])

    def distances(self, xy):
        ij=self.world_to_cell(xy);i,j=ij.T
        valid=(i>=0)&(j>=0)&(i<self.width)&(j<self.height)
        d=np.zeros(len(ij));d[valid]=self.clearance[j[valid],i[valid]]
        return d

    def check(self,state,footprints,soft_limit=math.radians(35)):
        if abs(wrap(state[3]-state[2]))>soft_limit:
            return False
        xy=np.vstack([transform(footprints.samples[0],state[:2],state[2]),
                      transform(footprints.samples[1],hitch(state,footprints.geometry),state[3])])
        # Half raster/sample spacing bounds cells missed between sample points.
        return bool(np.all(self.distances(xy)>self.margin+.06))

    def path_free(self, states, footprints):
        return all(self.check(s,footprints) for s in states)
