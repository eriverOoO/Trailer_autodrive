"""Accumulated short-range free space from LiDAR beams that returned no echo (+inf).

Cartographer only clears cells along beams that hit something, so an open area without
walls stays unobserved (-1) and planning/detection grids treat it as blocked. This layer
remembers the cells crossed by no-return beams up to free_range in map coordinates and
marks only still-unobserved cells free. Occupied cells are never changed.
numpy only (no rclpy) so it can be tested offline.
"""
import math
import numpy as np

# Cell (ix, iy) packed into one int64; offset keeps (ix+offset)<<32 below the sign bit.
_OFFSET=2**30

def _encode(ix,iy):return ((ix.astype(np.int64)+_OFFSET)<<32)|(iy.astype(np.int64)+_OFFSET)
def _decode(keys):return (keys>>32)-_OFFSET,(keys&0xFFFFFFFF)-_OFFSET

def _origin(origin):
    o=np.asarray(origin,dtype=float)
    if o.shape!=(3,) or not np.isfinite(o).all():raise ValueError('invalid grid origin')
    return o

def _grid_index(x,y,origin,resolution):
    # Same map→grid cell convention as StateEstimator.collision_data (origin yaw supported).
    c,s=math.cos(origin[2]),math.sin(origin[2]);dx=x-origin[0];dy=y-origin[1]
    return np.floor((c*dx+s*dy)/resolution).astype(int),np.floor((-s*dx+c*dy)/resolution).astype(int)

class NoReturnFreeLayer:
    def __init__(self,resolution,free_range):
        self.resolution=float(resolution);self.free_range=float(free_range)
        if not math.isfinite(self.resolution) or self.resolution<=0:raise ValueError('resolution must be positive')
        if not math.isfinite(self.free_range) or self.free_range<0:raise ValueError('free_range must be >= 0')
        self.cells=set();self._array=None
    def __len__(self):return len(self.cells)
    def add_scan(self,ranges,angle_min,angle_increment,range_min,pose,occupied=None,origin=None):
        """Accumulate cells crossed by +inf beams from sensor pose [x, y, yaw] (map frame).
        With an occupied bool grid and its origin [x, y, yaw], each ray stops at its first occupied cell."""
        if self.free_range<=0:return 0
        p=np.asarray(pose,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('invalid sensor pose')
        if not all(math.isfinite(v) for v in (angle_min,angle_increment,range_min)) or range_min<0:raise ValueError('invalid scan geometry')
        rs=np.asarray(ranges,dtype=float);beams=np.flatnonzero(np.isposinf(rs))
        # Half-cell spacing; samples inside range_min are never trusted.
        step=self.resolution*.5;distances=np.arange(max(range_min,step),self.free_range,step);distances=distances[distances<self.free_range]
        if not beams.size or not distances.size:return 0
        angles=p[2]+angle_min+beams*angle_increment
        x=p[0]+distances[None,:]*np.cos(angles)[:,None];y=p[1]+distances[None,:]*np.sin(angles)[:,None]
        keep=np.ones(x.shape,dtype=bool)
        if occupied is not None and origin is not None:
            occ=np.asarray(occupied,dtype=bool);o=_origin(origin)
            if occ.ndim!=2:raise ValueError('occupied grid must be 2D')
            i,j=_grid_index(x,y,o,self.resolution);inside=(i>=0)&(j>=0)&(i<occ.shape[1])&(j<occ.shape[0])
            hit=np.zeros(x.shape,dtype=bool);hit[inside]=occ[j[inside],i[inside]]
            # The occupied cell and everything behind it stay as SLAM reported them.
            keep=np.cumsum(hit,axis=1)==0
        keys=np.unique(_encode(np.floor(x[keep]/self.resolution),np.floor(y[keep]/self.resolution)))
        before=len(self.cells);self.cells.update(keys.tolist())
        if len(self.cells)!=before:self._array=None
        return len(self.cells)-before
    def apply(self,data,origin):
        """Mark accumulated cells free where data < 0 (unobserved). data is (height, width), modified in place."""
        if self.free_range<=0 or not self.cells:return data
        o=_origin(origin)
        if self._array is None:self._array=np.fromiter(self.cells,dtype=np.int64,count=len(self.cells))
        ix,iy=_decode(self._array)
        # Cell centres in map coordinates → current grid, so origin/size changes keep the same world cells.
        i,j=_grid_index((ix+.5)*self.resolution,(iy+.5)*self.resolution,o,self.resolution)
        inside=(i>=0)&(j>=0)&(i<data.shape[1])&(j<data.shape[0]);i=i[inside];j=j[inside]
        unknown=data[j,i]<0;data[j[unknown],i[unknown]]=0
        return data
