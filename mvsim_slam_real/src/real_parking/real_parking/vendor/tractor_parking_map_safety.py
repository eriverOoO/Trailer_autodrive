"""EXECUTE RAW-map footprint-overlap evidence and persistence."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from tractor_trailer_model import Grid, transform
from tractor_parking_clearance import (sample_cloud,
    LIVE_BLOCK_CONFIRM_COUNT,LIVE_BLOCK_CONFIRM_SEC,
    LIVE_BLOCK_REGION_M)


def path_block_observation(grid, states, footprints, start_index=0, source='remaining_path'):
    """First footprint overlap, with a local nearest blocking cell.

    This is evidence only. Current actual polygon overlap must be stopped by
    the caller immediately; future path/predicted footprint blocks are confirmed.
    Clearance is diagnostic only and never creates an EXECUTE block.
    Avoid an additional whole-map distance transform for diagnostic cell lookup.
    """
    states=np.atleast_2d(states)
    if not len(states):return None
    cloud=sample_cloud(states,footprints)
    distances=grid.distances(cloud.reshape(-1,2)).reshape(cloud.shape[:2])
    for index,state in enumerate(states):
        clearance=float(distances[index].min())
        overlaps=not grid.collision_free(state,footprints)
        if not overlaps:continue
        sample=cloud[index,int(np.argmin(distances[index]))]
        i,j=grid.world_to_cell(sample)[0]
        cell=None;cell_xy=None;occupancy=None
        if 0<=i<grid.width and 0<=j<grid.height:
            radius=int(math.ceil(clearance/grid.resolution))+1
            lo=np.maximum([i-radius,j-radius],0)
            hi=np.minimum([i+radius+1,j+radius+1],[grid.width,grid.height])
            ji=np.argwhere(grid.blocked[lo[1]:hi[1],lo[0]:hi[0]])
            if len(ji):
                ij=ji[:,::-1]+lo
                nearest=ij[int(np.argmin(np.sum((ij-[i,j])**2,axis=1)))]
                cell=nearest.tolist();cell_xy=grid.cell_to_world(nearest)[0].tolist()
                occupancy=int(grid.data[nearest[1],nearest[0]])
        path_index=start_index if source=='predicted_footprint' else start_index+index
        return dict(source=source,blocking_path_index=int(path_index),
            prediction_step=int(index) if source=='predicted_footprint' else None,
            blocking_pose=np.asarray(state).tolist(),measured_clearance_m=clearance,
            footprint_overlap=overlaps,blocking_cell_ij=cell,blocking_cell_xy=cell_xy,
            blocking_region_xy=cell_xy if cell_xy is not None else sample.tolist(),
            blocking_occupancy=occupancy)
    return None


class LiveBlockConfirmation:
    """Count distinct checked maps for one region AND require elapsed time.

    Rechecking an unchanged map never adds a hit or confirms a block. A free
    path, a free blocking cell (even between timer checks), or another region
    resets the streak. Events are emitted only at pending/cleared/confirmed.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.pending=None;self.anchor=None;self.since=None
        self.count=0;self.last_sequence=None;self.confirmed=False

    def snapshot(self,now):
        return dict(self.pending,consecutive_block_count=self.count,
                    block_persistence_s=max(0.,now-self.since))

    def clear(self,reason,now,sequence=None,stamp_ns=None):
        if self.pending is None:return []
        record=self.snapshot(now);record['reset_reason']=reason
        record['last_blocked_map_sequence']=record['map_sequence']
        record['last_blocked_map_stamp_ns']=record['map_stamp_ns']
        if sequence is not None:record['map_sequence']=sequence
        if stamp_ns is not None:record['map_stamp_ns']=stamp_ns
        self.reset()
        return [('BLOCK_CLEARED',record)]

    def update(self,observation,sequence,stamp_ns,now):
        if observation is None:return self.clear('relevant path checks free',now,sequence,stamp_ns)
        events=[];region=np.asarray(observation['blocking_region_xy'])
        if self.pending is not None and np.linalg.norm(region-self.anchor)>LIVE_BLOCK_REGION_M:
            events+=self.clear('blocking region changed',now,sequence,stamp_ns)
        first=self.pending is None
        if first:
            self.since=now;self.anchor=region.copy()
        fresh=sequence!=self.last_sequence
        self.pending=dict(observation,map_sequence=sequence,map_stamp_ns=stamp_ns,
                          map_checked_wall_s=now)
        if fresh:
            self.count+=1;self.last_sequence=sequence
        if first:events.append(('BLOCK_PENDING',self.snapshot(now)))
        if (fresh and not self.confirmed and self.count>=LIVE_BLOCK_CONFIRM_COUNT and
            now-self.since>=LIVE_BLOCK_CONFIRM_SEC):
            self.confirmed=True
            events.append(('BLOCK_CONFIRMED',self.snapshot(now)))
        return events

    def clear_if_cell_free(self,data,resolution,origin,sequence,stamp_ns,now):
        """Notice intervening free map updates without building a safety grid."""
        if self.pending is None or self.pending['blocking_cell_xy'] is None:return []
        xy=np.asarray(self.pending['blocking_cell_xy'])
        local=transform([xy-np.asarray(origin[:2])],[0,0],-origin[2])[0]
        i,j=np.floor(local/resolution).astype(int)
        if 0<=j<data.shape[0] and 0<=i<data.shape[1] and 0<=data[j,i]<50:
            events=self.clear('blocking cell free in newer RAW map',now,sequence,stamp_ns)
            events[0][1]['cleared_cell_occupancy']=int(data[j,i])
            return events
        return []


def boundary_reference(live, snapshot, target, tolerance=.25):
    """Only bounded reobservations of measured U walls use the frozen reference.

    Unknown changes and occupied cells away from BOTH previously occupied
    evidence and detected walls remain obstacles. This grid is for distant
    reference-path validity only; the original live grid must check near motion.
    """
    compatible=np.zeros(live.data.shape,dtype=bool)
    if live.data.shape!=snapshot.data.shape or not np.any(snapshot.occupied):
        return live,compatible
    if live.resolution!=snapshot.resolution or not np.allclose(live.origin,snapshot.origin):
        return live,compatible
    new=live.occupied & (snapshot.data>=0) & ~snapshot.occupied
    ji=np.argwhere(new)
    if not len(ji):return live,compatible
    xy=live.cell_to_world(ji[:,::-1])
    local=transform(xy-target.center,[0,0],-target.heading)
    l,w=target.length/2,target.width/2
    wall=np.minimum(np.minimum(abs(local[:,0]-l),abs(local[:,0]+l)),abs(local[:,1]+w))
    roi=(abs(local[:,0])<=l+tolerance)&(abs(local[:,1])<=w+tolerance)
    old_distance=distance_transform_edt(~snapshot.occupied)*live.resolution
    keep=roi & (wall<=tolerance) & (old_distance[ji[:,0],ji[:,1]]<=tolerance)
    ij=ji[keep];compatible[ij[:,0],ij[:,1]]=True
    data=live.data.copy();data[compatible]=snapshot.data[compatible]
    return Grid(data,live.resolution,live.origin,margin=live.margin),compatible


def near_path(states, index, current, horizon):
    """Raw-map check covers upcoming travel AND future states spatially nearby."""
    remaining=np.asarray(states[index:])
    if not len(remaining):return remaining
    travel=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(remaining[:,:2],axis=0),axis=1))]
    nearby=np.linalg.norm(remaining[:,:2]-current[:2],axis=1)<=horizon
    return remaining[(travel<=horizon)|nearby]
