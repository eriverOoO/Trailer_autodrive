"""Separate planning clearance policy from live-map blockage confirmation.

Occupied/unknown cell overlap and out-of-map footprints remain invalid.
Generic Grid and MVSim physics collision detection remain unchanged.
"""
import math
import numpy as np
from scipy.ndimage import map_coordinates
from tractor_trailer_model import Grid, transform


PLANNER_HARD_CLEARANCE_M = .18
PLANNER_PREFERRED_CLEARANCE_M = .25
LIVE_BLOCK_CLEARANCE_M = .18
LIVE_BLOCK_CONFIRM_COUNT = 3
LIVE_BLOCK_CONFIRM_SEC = .5
LIVE_BLOCK_REGION_M = .20  # Associate nearby blocking cells, in world metres.
CLEARANCE_SOFT_WEIGHT = 2.0  # Added distance-cost units per metre at unit density.
NEAR_WALL_START_M = .30
NEAR_WALL_EXPOSURE_WEIGHT = 1.0
MIN_CLEARANCE_QUALITY_WEIGHT = .5
SAMPLE_ALLOWANCE_M = .06
CLEARANCE_EPS_M = 1e-9  # Equality tolerance only, not an extra safety reduction.


def penalty_density(clearance, optimization=False):
    """Continuous, zero-slope onset at preferred; steep near the hard minimum.

    Optimizer iterates may be invalid; final acceptance always uses hard checks.
    """
    d = np.asarray(clearance, dtype=float)
    deficit = np.maximum(0., (PLANNER_PREFERRED_CLEARANCE_M-d)/
                         (PLANNER_PREFERRED_CLEARANCE_M-PLANNER_HARD_CLEARANCE_M))
    value = CLEARANCE_SOFT_WEIGHT*deficit**2*(PLANNER_PREFERRED_CLEARANCE_M/
                    np.maximum(d, PLANNER_HARD_CLEARANCE_M/2))**2
    if not optimization:
        value = np.where(d <= PLANNER_HARD_CLEARANCE_M+CLEARANCE_EPS_M, np.inf, value)
    return value


def sample_cloud(states, footprints):
    states = np.atleast_2d(states)
    pieces=[]
    for body, samples in enumerate(footprints.samples):
        angle=states[:,2+body];c=np.cos(angle);s=np.sin(angle)
        xy=states[:,:2].copy()
        if body:
            xy-=footprints.geometry.hitch_offset*np.c_[np.cos(states[:,2]),np.sin(states[:,2])]
        points=np.asarray(samples)
        pieces.append(np.stack((c[:,None]*points[:,0]-s[:,None]*points[:,1],
                                s[:,None]*points[:,0]+c[:,None]*points[:,1]),axis=2)+xy[:,None,:])
    return np.concatenate(pieces,axis=1)


def footprint_clearances(grid, states, footprints, smooth=False):
    cloud=sample_cloud(states,footprints);flat=cloud.reshape(-1,2)
    if smooth:
        local=transform(flat-grid.origin[:2],[0,0],-grid.origin[2])/grid.resolution-.5
        distances=map_coordinates(grid.clearance,[local[:,1],local[:,0]],order=1,
                                  mode='constant',cval=0.,prefilter=False)
    else:
        distances=grid.distances(flat)
    return distances.reshape(cloud.shape[:2]).min(axis=1)


def footprint_clearance_components(grid, states, footprints, smooth=False):
    """Return per-state tractor and trailer minima from the same EDT query."""
    cloud=sample_cloud(states,footprints);flat=cloud.reshape(-1,2)
    if smooth:
        local=transform(flat-grid.origin[:2],[0,0],-grid.origin[2])/grid.resolution-.5
        distances=map_coordinates(grid.clearance,[local[:,1],local[:,0]],order=1,
                                  mode='constant',cval=0.,prefilter=False)
    else:
        distances=grid.distances(flat)
    values=distances.reshape(cloud.shape[:2])
    n_tractor=len(footprints.samples[0])
    return values[:,:n_tractor].min(axis=1),values[:,n_tractor:].min(axis=1)


def integrated_penalty(states, clearances, start=None, optimization=False):
    states=np.atleast_2d(states)
    if start is None:
        distances=np.r_[0.,np.linalg.norm(np.diff(states[:,:2],axis=0),axis=1)]
    else:
        distances=np.linalg.norm(np.diff(np.vstack([np.asarray(start)[:2],states[:,:2]]),axis=0),axis=1)
    density=penalty_density(clearances,optimization)
    if np.any(~np.isfinite(density)):return math.inf
    return float(np.sum(distances*density))


def near_wall_density(clearances):
    """Soft quality cost through 0.30 m; remains separate from hard validity."""
    c=np.asarray(clearances,dtype=float)
    out=np.zeros_like(c)
    mid=(c>=PLANNER_PREFERRED_CLEARANCE_M)&(c<NEAR_WALL_START_M)
    out[mid]=.25*((NEAR_WALL_START_M-c[mid])/
                  (NEAR_WALL_START_M-PLANNER_PREFERRED_CLEARANCE_M))**2
    near=(c>PLANNER_HARD_CLEARANCE_M+CLEARANCE_EPS_M)&(c<PLANNER_PREFERRED_CLEARANCE_M)
    out[near]=.25+1.75*((PLANNER_PREFERRED_CLEARANCE_M-c[near])/
                        (PLANNER_PREFERRED_CLEARANCE_M-PLANNER_HARD_CLEARANCE_M))**2
    return out


def integrated_near_wall_cost(states, clearances, start=None):
    states=np.atleast_2d(states)
    if start is None:
        ds=np.r_[0.,np.linalg.norm(np.diff(states[:,:2],axis=0),axis=1)]
    else:
        ds=np.linalg.norm(np.diff(np.vstack([np.asarray(start)[:2],states[:,:2]]),axis=0),axis=1)
    return float(np.sum(ds*near_wall_density(clearances)))


def minimum_clearance_density(clearances):
    c=np.asarray(clearances,dtype=float)
    return MIN_CLEARANCE_QUALITY_WEIGHT*np.maximum(0.,
        (NEAR_WALL_START_M-c)/(NEAR_WALL_START_M-PLANNER_HARD_CLEARANCE_M))**2


def integrated_minimum_clearance_cost(states,clearances,start=None):
    states=np.atleast_2d(states)
    if start is None:
        ds=np.r_[0.,np.linalg.norm(np.diff(states[:,:2],axis=0),axis=1)]
    else:
        ds=np.linalg.norm(np.diff(np.vstack([np.asarray(start)[:2],states[:,:2]]),axis=0),axis=1)
    return float(np.sum(ds*minimum_clearance_density(clearances)))


def clearance_quality_cost(states,clearances):
    """Additive clearance-only ranking term; it never participates in validity."""
    c=np.asarray(clearances,dtype=float)
    minimum=float(c.min()) if c.size else 0.
    minimum_quality=MIN_CLEARANCE_QUALITY_WEIGHT*max(0.,
        (NEAR_WALL_START_M-minimum)/
        (NEAR_WALL_START_M-PLANNER_HARD_CLEARANCE_M))**2
    return (integrated_penalty(states,c,optimization=True)+
            NEAR_WALL_EXPOSURE_WEIGHT*integrated_near_wall_cost(states,clearances)+
            integrated_minimum_clearance_cost(states,clearances)+minimum_quality)


def path_quality_score(length,states,clearances):
    return float(length)+clearance_quality_cost(states,clearances)


class SimulationParkingGrid(Grid):
    """Planner hard clearance plus an independent exact footprint overlap veto."""
    hard_clearance_m=PLANNER_HARD_CLEARANCE_M

    def __init__(self,data,resolution,origin,threshold=50,unknown_occupied=True):
        super().__init__(data,resolution,origin,threshold,unknown_occupied,
                         margin=PLANNER_HARD_CLEARANCE_M-SAMPLE_ALLOWANCE_M)

    def polygon_collision_free(self, polygon):
        """Separating-axis test against full blocked cell squares, not centers.

        Runtime chassis+wheel envelopes are convex. Touching a blocked cell is
        rejected conservatively. Unknown and outside remain collision obstacles.
        """
        p=transform(polygon-self.origin[:2],[0,0],-self.origin[2])/self.resolution
        lo=np.floor(p.min(axis=0)).astype(int);hi=np.floor(p.max(axis=0)).astype(int)
        if np.any(lo<0) or hi[0]>=self.width or hi[1]>=self.height:return False
        ji=np.argwhere(self.blocked[lo[1]:hi[1]+1,lo[0]:hi[0]+1])
        if not len(ji):return True
        centers=ji[:,::-1]+lo+.5
        edges=np.roll(p,-1,axis=0)-p
        axes=np.vstack((np.eye(2),np.c_[-edges[:,1],edges[:,0]]))
        axes=axes[np.linalg.norm(axes,axis=1)>1e-12]
        projections=p@axes.T;center=centers@axes.T
        radius=.5*np.abs(axes).sum(axis=1)
        overlap=np.all((center+radius>=projections.min(axis=0)-1e-12)&
                       (center-radius<=projections.max(axis=0)+1e-12),axis=1)
        return not bool(np.any(overlap))

    def collision_free(self,state,footprints):
        return all(self.polygon_collision_free(p) for p in footprints.world(state))

    def validate_path(self,states,footprints,soft_limit=math.radians(35)):
        states=np.atleast_2d(states)
        if not len(states):return False,np.empty(0)
        if np.any(np.abs((states[:,3]-states[:,2]+np.pi)%(2*np.pi)-np.pi)>soft_limit):
            return False,np.empty(0)
        clearances=footprint_clearances(self,states,footprints)
        if np.any(clearances<=PLANNER_HARD_CLEARANCE_M+CLEARANCE_EPS_M):return False,clearances
        return all(self.collision_free(s,footprints) for s in states),clearances

    def check(self,state,footprints,soft_limit=math.radians(35)):
        return self.validate_path(np.asarray(state)[None,:],footprints,soft_limit)[0]

    def path_free(self,states,footprints):
        return self.validate_path(states,footprints)[0]


def path_metrics(grid,states,footprints):
    valid,values=grid.validate_path(states,footprints)
    tractor,trailer=footprint_clearance_components(grid,states,footprints,smooth=False)
    smooth_tractor,smooth_trailer=footprint_clearance_components(
        grid,states,footprints,smooth=True)
    smooth=np.minimum(smooth_tractor,smooth_trailer)
    # Hard clearance is reported using exactly the same nearest-cell metric
    # as live safety. Bilinear sampling is only for continuous soft optimization.
    distances=np.r_[0.,np.linalg.norm(np.diff(np.atleast_2d(states)[:,:2],axis=0),axis=1)]
    combined_min=float(values.min()) if len(values) else 0.
    hard_minimum_quality=MIN_CLEARANCE_QUALITY_WEIGHT*max(0.,
        (NEAR_WALL_START_M-combined_min)/
        (NEAR_WALL_START_M-PLANNER_HARD_CLEARANCE_M))**2
    length=float(np.linalg.norm(np.diff(np.atleast_2d(states)[:,:2],axis=0),axis=1).sum())
    quality_score=(length+integrated_penalty(states,smooth,optimization=True)+
        NEAR_WALL_EXPOSURE_WEIGHT*integrated_near_wall_cost(states,smooth)+
        integrated_minimum_clearance_cost(states,smooth)+hard_minimum_quality)
    return dict(valid=valid,minimum_clearance_m=combined_min,
                minimum_tractor_clearance_m=float(tractor.min()) if len(tractor) else 0.,
                minimum_trailer_clearance_m=float(trailer.min()) if len(trailer) else 0.,
                near_wall_exposure_distance_m=float(np.sum(distances[smooth<NEAR_WALL_START_M])),
                near_wall_exposure_cost=integrated_near_wall_cost(states,smooth),
                minimum_clearance_quality_cost=hard_minimum_quality,
                soft_penalty=integrated_penalty(states,smooth,optimization=True),
                total_path_quality_score=quality_score)
