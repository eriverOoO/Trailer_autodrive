"""Event-triggered, behavior-neutral evidence capture for latest RAW rejects."""
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
from tractor_trailer_model import transform, wrap, hitch, trailer_axle
from tractor_parking_clearance import (
    PLANNER_HARD_CLEARANCE_M, SimulationParkingGrid, footprint_clearance_components,
    footprint_clearances, path_metrics,
)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _map_snapshot(grid, received_wall=None):
    return {
        'data': np.asarray(grid.data, dtype=np.int16).copy(),
        'width': int(grid.width), 'height': int(grid.height),
        'resolution': float(grid.resolution), 'origin': np.asarray(grid.origin, dtype=float).copy(),
        'frame_id': getattr(grid, 'frame_id', 'local_map'),
        'stamp_ns': int(getattr(grid, 'map_stamp_ns', 0)),
        'sequence': int(getattr(grid, 'map_sequence', 0)),
        'received_wall': received_wall,
    }


def snapshot_from_msg(msg, sequence, received_wall):
    return {
        'data': np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width).copy(),
        'width': int(msg.info.width), 'height': int(msg.info.height),
        'resolution': float(msg.info.resolution),
        'origin': np.array([msg.info.origin.position.x, msg.info.origin.position.y,
                            math.atan2(2*(msg.info.origin.orientation.w*msg.info.origin.orientation.z + msg.info.origin.orientation.x*msg.info.origin.orientation.y),
                                       1-2*(msg.info.origin.orientation.y**2+msg.info.origin.orientation.z**2))]),
        'frame_id': str(msg.header.frame_id), 'stamp_ns': _stamp_ns(msg.header.stamp),
        'sequence': int(sequence), 'received_wall': float(received_wall),
    }


def _save_map(directory, label, snap):
    if snap is None:
        return None
    np.save(Path(directory) / f'map_{label}.npy', snap['data'], allow_pickle=False)
    result={k:v for k,v in snap.items() if k!='data'}
    if isinstance(result.get('origin'),np.ndarray):result['origin']=result['origin'].astype(float).tolist()
    return result


def _state_failure(grid, state, footprints):
    """Return failed body/reason/cell for the exact captured validator grid."""
    bodies = ('TRACTOR', 'TRAILER')
    polygons = footprints.world(state)
    vals = footprint_clearance_components(grid, np.asarray(state)[None, :], footprints)
    body_clearance = [float(vals[0][0]), float(vals[1][0])]
    # Out-of-grid evidence has precedence because Grid.distances returns zero there.
    for b, samples in enumerate(footprints.samples):
        xy = transform(samples, state[:2] if b == 0 else hitch(state, footprints.geometry),
                       state[2+b])
        ij = grid.world_to_cell(xy)
        outside = (ij[:, 0] < 0) | (ij[:, 1] < 0) | (ij[:, 0] >= grid.width) | (ij[:, 1] >= grid.height)
        if np.any(outside):
            k = int(np.flatnonzero(outside)[0])
            return bodies[b], 'OUT_OF_MAP', polygons[b], body_clearance[b], None, xy[k], None
    overall = min(body_clearance)
    if overall <= PLANNER_HARD_CLEARANCE_M + 1e-9:
        b = int(body_clearance[1] < body_clearance[0])
        body = bodies[b]
        # Closest blocked cell center to actual validator sample points, bounded to body ROI.
        samples = footprints.samples[b]
        origin_xy = state[:2] if b == 0 else hitch(state, footprints.geometry)
        cloud = transform(samples, origin_xy, state[2+b])
        ij = grid.world_to_cell(cloud)
        lo = np.maximum(np.min(ij, axis=0)-3, [0, 0]); hi = np.minimum(np.max(ij, axis=0)+4, [grid.width, grid.height])
        region = grid.blocked[lo[1]:hi[1], lo[0]:hi[0]]
        cells = np.argwhere(region)
        if len(cells):
            cells[:, 0] += lo[1]; cells[:, 1] += lo[0]
            # Match the validator's EDT nearest-blocked cell for the closest sampled point.
            from scipy.ndimage import distance_transform_edt
            nearest = distance_transform_edt(~grid.blocked, return_distances=False, return_indices=True)
            sample_ij = grid.world_to_cell(cloud)
            sample_dist = grid.distances(cloud)
            si = int(np.argmin(sample_dist)); sx, sy = sample_ij[si]
            if 0 <= sx < grid.width and 0 <= sy < grid.height:
                cy, cx = int(nearest[0, sy, sx]), int(nearest[1, sy, sx])
            else:
                cy, cx = int(cells[0,0]), int(cells[0,1])
            center = grid.cell_to_world([[cx, cy]])[0]
            distance = float(np.linalg.norm(cloud[si]-center))
            occ = int(grid.data[cy, cx])
            reason = 'UNKNOWN_CELL' if occ < 0 else 'HARD_CLEARANCE'
            return body, reason, polygons[b], body_clearance[b], (cx, cy, center, occ, distance), cloud[si], ('sample', si)
        return body, 'HARD_CLEARANCE', polygons[b], body_clearance[b], None, cloud[int(np.argmin(grid.distances(cloud)))], None
    for b, polygon in enumerate(polygons):
        if not grid.polygon_collision_free(polygon):
            p = transform(polygon-grid.origin[:2], [0, 0], -grid.origin[2])/grid.resolution
            lo=np.maximum(np.floor(p.min(axis=0)).astype(int),[0,0]); hi=np.minimum(np.floor(p.max(axis=0)).astype(int)+1,[grid.width,grid.height])
            ji=np.argwhere(grid.blocked[lo[1]:hi[1],lo[0]:hi[0]])
            if len(ji):
                ji[:,0]+=lo[1];ji[:,1]+=lo[0]
                edges=np.roll(p,-1,axis=0)-p
                axes=np.vstack((np.eye(2),np.c_[-edges[:,1],edges[:,0]]))
                axes=axes[np.linalg.norm(axes,axis=1)>1e-12]
                projections=p@axes.T;radius=.5*np.abs(axes).sum(axis=1)
                hit=None
                for row in ji:
                    center=np.array([row[1]+.5,row[0]+.5]); cp=center@axes.T
                    if np.all((cp+radius>=projections.min(axis=0)-1e-12)&(cp-radius<=projections.max(axis=0)+1e-12)):
                        hit=row;break
                if hit is not None:
                    j,i=map(int,hit); center=grid.cell_to_world([[i,j]])[0];
                    # Exact closest point on the polygon boundary to this cell center.
                    local=transform([center-grid.origin[:2]],[0,0],-grid.origin[2])[0]/grid.resolution
                    best=None
                    for k,(a0,b0) in enumerate(zip(p,np.roll(p,-1,axis=0))):
                        v=b0-a0;den=float(v@v);t=0. if den==0 else float(np.clip((local-a0)@v/den,0,1))
                        q=a0+t*v;dist=float(np.linalg.norm(local-q))
                        if best is None or dist<best[0]:best=(dist,k,q)
                    closest=transform([best[2]*grid.resolution],grid.origin[:2],grid.origin[2])[0]
                    occ=int(grid.data[j,i]);reason='UNKNOWN_CELL' if occ<0 else 'FOOTPRINT_OVERLAP'
                    return bodies[b], reason, polygon, body_clearance[b], (i,j,center,occ,float(best[0]*grid.resolution)), closest, (f'edge_{best[1]}',None)
            return bodies[b], 'OUT_OF_MAP', polygon, body_clearance[b], None, polygon[0], None
    return None


def _write_footprint(path, polygon):
    with open(path, 'w', newline='') as f:
        w=csv.writer(f);w.writerow(['vertex_index','world_x','world_y'])
        w.writerows([[i,float(x),float(y)] for i,(x,y) in enumerate(polygon)])


def _nearest_failure(path, grid, current, footprints):
    vals=footprint_clearances(grid,path,footprints)
    # Match SimulationParkingGrid validation order: path hard clearance first, then polygon overlap.
    if np.any(vals <= PLANNER_HARD_CLEARANCE_M + 1e-9):
        i=int(np.flatnonzero(vals <= PLANNER_HARD_CLEARANCE_M + 1e-9)[0])
        return i, _state_failure(grid,path[i],footprints), float(vals[i]), 'PATH'
    for i,s in enumerate(path):
        for b,poly in enumerate(footprints.world(s)):
            if not grid.polygon_collision_free(poly):
                fail=_state_failure(grid,s,footprints)
                return i,fail,float(vals[i]),'PATH'
    if not grid.check(current,footprints):
        return -1,_state_failure(grid,current,footprints),float(min(footprint_clearance_components(grid,np.asarray(current)[None,:],footprints)[0][0],footprint_clearance_components(grid,np.asarray(current)[None,:],footprints)[1][0])),'CURRENT'
    return None


def capture_reject(root, event_id, grid, previous, state, path, footprints,
                   identity, timestamps, reason_context=None):
    started=time.perf_counter()
    root=Path(root); event=root/event_id; event.mkdir(parents=True,exist_ok=False)
    path=np.asarray(path,dtype=float); state=np.asarray(state,dtype=float)
    metrics=path_metrics(grid,path,footprints)
    failure=_nearest_failure(path,grid,state,footprints)
    if failure is None:
        idx=-1; fail=None; minimum=metrics['minimum_clearance_m']; scope='UNKNOWN'
    else:
        idx,fail,minimum,scope=failure
    pose=state if idx<0 else path[idx]
    body=fail[0] if fail else 'UNKNOWN'; fail_reason=fail[1] if fail else 'OTHER'
    poly=fail[2] if fail else footprints.world(pose)[0]
    body_clear=fail[3] if fail else minimum
    cell=fail[4] if fail else None
    closest=fail[5] if fail else [float('nan'),float('nan')]
    feature=fail[6] if fail else None
    if cell:
        gx,gy,center,occ,distance=cell
        wx,wy=map(float,center)
    else:
        gx=gy=occ=None;wx=wy=distance=None
    articulation=wrap(pose[3]-pose[2])
    candidate_start=path[0]
    dx=float(state[0]-candidate_start[0]);dy=float(state[1]-candidate_start[1]);dyaw=wrap(state[2]-candidate_start[2])
    stamp_now=int(timestamps['validator_invocation_ns']); map_stamp=int(grid.map_stamp_ns)
    pose_stamp=int(timestamps.get('pose_stamp_ns') or 0); scan_stamp=int(timestamps.get('scan_stamp_ns') or 0)
    data={
        **identity,'event_id':event_id,'capture_scope':scope,'failure_reason':fail_reason,
        'rejected_body':body,'path_index':idx,'segment_index':max(0,idx-1) if idx>=0 else None,
        'path_pose':{'x':float(pose[0]),'y':float(pose[1]),'yaw':float(pose[2]),'trailer_yaw':float(pose[3]),'articulation_rad':float(articulation),'articulation_deg':math.degrees(articulation)},
        'blocking_cell':None if cell is None else {'grid_x':gx,'grid_y':gy,'world_x':wx,'world_y':wy,'occupancy':occ,'distance_to_footprint_m':distance},
        'closest_footprint_point':list(map(float,closest)), 'footprint_feature':feature,
        'clearance_m':{'tractor':metrics['minimum_tractor_clearance_m'],'trailer':metrics['minimum_trailer_clearance_m'],'overall':metrics['minimum_clearance_m'],'rejected_body_at_failure_pose':body_clear},
        'validator':{'valid':bool(metrics['valid'] and grid.check(state,footprints)),'path_metrics_valid':bool(metrics['valid']),'current_pose_valid':bool(grid.check(state,footprints)),'invocation_ros_ns':stamp_now,'capture_ros_ns':int(timestamps['capture_ns']),'duration_ms':None},
        'map':{'width':grid.width,'height':grid.height,'resolution':grid.resolution,'origin':grid.origin.tolist(),'frame_id':getattr(grid,'frame_id','local_map'),'stamp_ns':map_stamp,'sequence':int(grid.map_sequence),'age_s':(stamp_now-map_stamp)*1e-9,'received_age_s':timestamps.get('map_received_wall_age_s')},
        'pose':{'stamp_ns':pose_stamp,'age_s':(stamp_now-pose_stamp)*1e-9 if pose_stamp else None,
                'state':[float(v) for v in state],
                'tractor':{'x':float(state[0]),'y':float(state[1]),'yaw':float(state[2])},
                'trailer':{'x':float(trailer_axle(state,footprints.geometry)[0]),
                           'y':float(trailer_axle(state,footprints.geometry)[1]),'yaw':float(state[3])}},
        'scan':{'stamp_ns':scan_stamp,'age_s':(stamp_now-scan_stamp)*1e-9 if scan_stamp else None},
        'timestamp_deltas_s':{'abs_map_pose':abs(map_stamp-pose_stamp)*1e-9 if pose_stamp else None,'abs_map_scan':abs(map_stamp-scan_stamp)*1e-9 if scan_stamp else None},
        'state_drift_from_candidate_start':{'dx':dx,'dy':dy,'dyaw':dyaw,
            'trailer_yaw_delta':wrap(state[3]-candidate_start[3]),
            'current_trailer_axle':[float(v) for v in trailer_axle(state,footprints.geometry)],
            'candidate_start_trailer_axle':[float(v) for v in trailer_axle(candidate_start,footprints.geometry)]},
        'reason_context':reason_context,
    }
    metadata={'event':data}
    metadata['event']['validator']['duration_ms']=None
    metadata['snapshot_previous']=_save_map(event,'prev',previous)
    if previous is not None:
        pg=SimulationParkingGrid(previous['data'],previous['resolution'],previous['origin'])
        pm=path_metrics(pg,path,footprints); pc=pg.check(state,footprints)
        metadata['offline_validation_previous']={'candidate_path_valid':bool(pm['valid']),'current_pose_valid':bool(pc),'combined_min_clearance_m':pm['minimum_clearance_m'],'tractor_min_clearance_m':pm['minimum_tractor_clearance_m'],'trailer_min_clearance_m':pm['minimum_trailer_clearance_m'],'overall_result':'PASS' if pm['valid'] and pc else 'REJECT'}
    reject_snap=_map_snapshot(grid,timestamps.get('map_received_wall'))
    metadata['snapshot_reject']=_save_map(event,'reject',reject_snap)
    metadata['offline_validation_reject']={'candidate_path_valid':bool(metrics['valid']),'current_pose_valid':bool(grid.check(state,footprints)),'combined_min_clearance_m':metrics['minimum_clearance_m'],'tractor_min_clearance_m':metrics['minimum_tractor_clearance_m'],'trailer_min_clearance_m':metrics['minimum_trailer_clearance_m'],'overall_result':'PASS' if metrics['valid'] and grid.check(state,footprints) else 'REJECT'}
    with open(event/'candidate_path.csv','w',newline='') as f:
        w=csv.writer(f);w.writerow(['path_index','x','y','yaw','trailer_yaw','articulation_rad','articulation_deg'])
        for i,s in enumerate(path):w.writerow([i,*map(float,s[:2]),float(s[2]),float(s[3]),wrap(s[3]-s[2]),math.degrees(wrap(s[3]-s[2]))])
    _write_footprint(event/'tractor_footprint.csv',footprints.world(pose)[0])
    _write_footprint(event/'trailer_footprint.csv',footprints.world(pose)[1])
    if cell:
        with open(event/'neighborhood.csv','w',newline='') as f:
            w=csv.writer(f);w.writerow(['grid_x','grid_y','world_x','world_y','occupancy'])
            for gy0 in range(max(0,gy-8),min(grid.height,gy+9)):
                for gx0 in range(max(0,gx-8),min(grid.width,gx+9)):
                    xy=grid.cell_to_world([[gx0,gy0]])[0]
                    w.writerow([gx0,gy0,float(xy[0]),float(xy[1]),int(grid.data[gy0,gx0])])
    else:
        with open(event/'neighborhood.csv','w',newline='') as f:
            csv.writer(f).writerow(['grid_x','grid_y','world_x','world_y','occupancy'])
    # Per-body minima at every candidate path pose enables offline same-validator replay.
    tc,tr=footprint_clearance_components(grid,path,footprints)
    with open(event/'per_pose_clearance.csv','w',newline='') as f:
        w=csv.writer(f);w.writerow(['path_index','tractor_clearance_m','trailer_clearance_m','overall_clearance_m'])
        for i,(a,b) in enumerate(zip(tc,tr)):w.writerow([i,float(a),float(b),min(float(a),float(b))])
    with open(event/'metadata.json','w') as f:json.dump(metadata,f,indent=2)
    pending={'directory':str(event),'path':path.tolist(),'state':state.tolist(),'footprints':footprints,'identity':identity,'created_ns':stamp_now}
    metadata['event']['validator']['duration_ms']=(time.perf_counter()-started)*1000
    with open(event/'metadata.json','w') as f:json.dump(metadata,f,indent=2)
    return pending,metadata['event']['validator']['duration_ms']


def save_next_map(pending, snapshot):
    if pending is None:return None
    event=Path(pending['directory'])
    meta=_save_map(event,'next',snapshot)
    md=json.loads((event/'metadata.json').read_text());md['snapshot_next']=meta
    (event/'metadata.json').write_text(json.dumps(md,indent=2))
    (event/'next_map_complete').write_text('complete\n')
    return {'event':event.name,'next_sequence':meta['sequence'],'next_stamp_ns':meta['stamp_ns']}
