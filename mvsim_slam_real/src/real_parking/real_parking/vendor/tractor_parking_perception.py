"""Parking-space geometry from Cartographer OccupancyGrid only.

White markings are unobservable. Slot width/depth and parked-vehicle size are
explicit scene priors, not obstacle poses read from the world. No image/ML input.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import binary_closing,binary_dilation,label
from tractor_trailer_model import Grid,transform


@dataclass
class Target:
    center: np.ndarray
    heading: float
    length: float = 5.8
    width: float = 2.0
    confidence: str = 'fixed debug target'

    def polygon(self):
        l,w=self.length/2,self.width/2
        return transform([[-l,-w],[l,-w],[l,w],[-l,w]],self.center,self.heading)

    def contains(self,polygons,tolerance=0.):
        for p in polygons:
            local=transform(p-self.center,[0,0],-self.heading)
            if np.any(abs(local[:,0])>self.length/2+tolerance) or np.any(abs(local[:,1])>self.width/2+tolerance):return False
        return True


def planning_grid(data,resolution,origin,bridge_gaps=False,simulation_clearance=False):
    data=np.asarray(data).copy()
    if bridge_gaps:
        # Only close narrow holes BETWEEN observed free beams; broad occlusion
        # shadows and the outside remain blocked. Never overwrite occupied cells.
        radius=max(1,int(math.ceil(.20/resolution)))
        yy,xx=np.mgrid[-radius:radius+1,-radius:radius+1]
        closed=binary_closing((data>=0)&(data<50),structure=xx*xx+yy*yy<=radius*radius)
        data[(data<0)&closed]=49
    if simulation_clearance:
        from tractor_parking_clearance import SimulationParkingGrid
        return SimulationParkingGrid(data,resolution,origin,threshold=50,unknown_occupied=True)
    return Grid(data,resolution,origin,threshold=50,unknown_occupied=True)


def detect_target(grid,road_pose,footprints,slot_length=5.8,slot_width=2.,car_length=3.2,car_width=1.7):
    """Cluster occupied evidence in a configurable road-relative parking side ROI.

    Partial L-shaped returns are sufficient: estimate the visible near and end
    faces using the vehicle-size prior, and use the midpoint of the two inferred
    centres. Reject pairs inconsistent in direction, size, gap and lateral row.
    """
    ij=np.argwhere(grid.data>=52)[:,::-1]
    if len(ij)<10:raise RuntimeError('too little occupied map evidence')
    xy=grid.cell_to_world(ij)
    local=transform(xy-road_pose[:2],[0,0],-road_pose[2])
    keep=(local[:,0]>3)&(local[:,0]<38)&(local[:,1]>-8)&(local[:,1]<-1.4)
    mask=np.zeros_like(grid.data,dtype=bool);ii=ij[keep];mask[ii[:,1],ii[:,0]]=True
    labels,n=label(binary_dilation(mask,iterations=max(1,int(.38/grid.resolution))))
    components=[]
    for k in range(1,n+1):
        ji=np.argwhere(mask&(labels==k))
        if len(ji)<6:continue
        pts=transform(grid.cell_to_world(ji[:,::-1])-road_pose[:2],[0,0],-road_pose[2])
        lo=np.quantile(pts,.04,axis=0);hi=np.quantile(pts,.96,axis=0);extent=hi-lo
        if not (.35<extent[1]<3.5 and .05<extent[0]<4.8):continue
        # Sensor initially approaches from the low-x, road side. Both end and
        # near faces are observed; a partial far car is completed by size prior.
        cx=(lo[0]+hi[0])/2 if extent[0]>car_length*.75 else lo[0]+car_length/2
        cy=hi[1]-car_width/2
        components.append((np.array([cx,cy]),len(pts),extent,pts))
    candidates=[]
    for i,(a,na,ea,pa) in enumerate(components):
        for b,nb,eb,pb in components[i+1:]:
            left,right=(a,b) if a[0]<b[0] else (b,a)
            separation=right[0]-left[0];gap=separation-car_length
            if not (footprints.size[0]+.6<gap<13 and abs(right[1]-left[1])<1.0):continue
            center=(left+right)/2
            # Heading from centres averages the row orientation, constrained to
            # the road prior to reject unrelated walls/landmark pairs.
            angle=math.atan2(right[1]-left[1],separation)
            if abs(angle)>math.radians(8):continue
            heading=road_pose[2]+angle
            world=transform([center],road_pose[:2],road_pose[2])[0]
            score=na+nb-20*abs(left[1]-right[1])
            candidates.append((score,Target(world,heading,slot_length,slot_width,
                                           f'two map clusters ({na},{nb} cells), gap={gap:.2f}m')))
    if not candidates:raise RuntimeError('two consistent parked-car clusters not yet observed')
    return max(candidates,key=lambda x:x[0])[1]
