import ctypes,math
from pathlib import Path
import numpy as np
from tractor_parking_clearance import footprint_clearances as original_clearances
LIB=ctypes.CDLL(str(Path(__file__).with_name('libnative.so')))
P=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
LIB.propagate_native.argtypes=[P,ctypes.c_double,ctypes.c_double,P,ctypes.c_int,P];LIB.propagate_native.restype=None
LIB.clearance_native.argtypes=[P,ctypes.c_int,P,ctypes.c_int,ctypes.c_int,ctypes.c_double,P,ctypes.c_int,ctypes.c_int,P,ctypes.c_double,ctypes.c_int,P];LIB.clearance_native.restype=None
_geometry={};_samples={}
def propagate(state,distance,delta,geometry,ds=.075):
 key=(geometry.wheelbase,geometry.hitch_offset,geometry.trailer_length)
 g=_geometry.setdefault(key,np.asarray(key,dtype=np.float64));n=max(1,int(math.ceil(abs(distance)/ds)));out=np.empty((n,4))
 LIB.propagate_native(np.ascontiguousarray(state,dtype=np.float64),distance,delta,g,n,out);return out
def clearances(grid,states,footprints,smooth=False):
 states=np.ascontiguousarray(np.atleast_2d(states),dtype=np.float64)
 k=id(footprints)
 if k not in _samples or _samples[k][0] is not footprints:_samples[k]=(footprints,np.ascontiguousarray(np.concatenate(footprints.samples),dtype=np.float64))
 points=_samples[k][1];out=np.empty(2*len(states))
 LIB.clearance_native(states,len(states),points,len(points),len(footprints.samples[0]),footprints.geometry.hitch_offset,np.ascontiguousarray(grid.clearance),grid.width,grid.height,np.ascontiguousarray(grid.origin),grid.resolution,int(smooth),out)
 values=out[:len(states)];ambiguous=out[len(states):]>0
 if not smooth and np.any(ambiguous):values[ambiguous]=original_clearances(grid,states[ambiguous],footprints,False)
 return values
