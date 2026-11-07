"""Read-only collision evidence using the identical runtime sample envelope."""
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
from tractor_trailer_model import transform, hitch, trailer_axle


def evidence(grid, planning, states, footprints, current, start_index, target=None):
    _, nearest = distance_transform_edt(~grid.blocked, return_indices=True)
    result = {'path_index': int(start_index), 'collision_margin_m': grid.margin,
              'sample_allowance_m': .06, 'required_clearance_m': grid.margin+.06,
              'current_tractor_pose': np.asarray(current)[[0, 1, 2]].tolist(),
              'current_trailer_pose': [*trailer_axle(current, footprints.geometry), float(current[3])],
              'blocked_cells': []}
    for index in range(start_index, len(states)):
        state = states[index]
        for body, points in zip(('tractor', 'trailer'),
                (transform(footprints.samples[0], state[:2], state[2]),
                 transform(footprints.samples[1], hitch(state, footprints.geometry), state[3]))):
            distances = grid.distances(points)
            if hasattr(grid,'hard_clearance_m'):
                from tractor_parking_clearance import CLEARANCE_EPS_M
                bad=np.flatnonzero(distances<=grid.hard_clearance_m+CLEARANCE_EPS_M)
                body_index=0 if body=='tractor' else 1
                overlaps=not grid.polygon_collision_free(footprints.world(state)[body_index])
                if overlaps and not len(bad):
                    # Exact polygon overlap is an independent veto even if no
                    # sampled point violates the EDT minimum.
                    bad=np.array([int(np.argmin(distances))])
                    result['exact_footprint_overlap']=True
            else:
                bad = np.flatnonzero(distances <= grid.margin+.06)
            if not len(bad):
                continue
            result.update(first_blocked_path_state_index=index, blocked_body=body,
                          first_blocked_state=np.asarray(state).tolist(),
                          path_to_wall_clearance_m=float(distances.min()),
                          ahead_in_path=index >= start_index,
                          path_states_ahead=index-start_index,
                          distance_from_current_m=float(np.linalg.norm(state[:2]-current[:2])))
            seen = set()
            for sample in bad:
                i, j = grid.world_to_cell(points[sample])[0]
                if not (0 <= i < grid.width and 0 <= j < grid.height):
                    result['outside_grid'] = True
                    continue
                bj, bi = nearest[:, j, i]
                key = (int(bi), int(bj))
                if key in seen:
                    continue
                seen.add(key)
                xy = grid.cell_to_world([bi, bj])[0]
                pi, pj = planning.world_to_cell(xy)[0]
                value = int(planning.data[pj, pi]) if 0 <= pi < planning.width and 0 <= pj < planning.height else None
                cell = {'ij': list(key), 'xy': xy.tolist(), 'occupancy': int(grid.data[bj, bi]),
                        'planning_occupancy': value,
                        'existed_in_planning_snapshot': bool(planning.blocked[pj, pi]) if value is not None else True,
                        'sample_xy': points[sample].tolist(), 'sample_clearance_m': float(distances[sample])}
                if target is not None:
                    local = transform([xy-target.center], [0, 0], -target.heading)[0]
                    cell['distance_to_detected_side_or_back_wall_m'] = float(min(
                        abs(local[0]-target.length/2), abs(local[0]+target.length/2),
                        abs(local[1]+target.width/2)))
                result['blocked_cells'].append(cell)
            return result
    result['reason'] = 'no sampled collision; inspect articulation limit'
    return result


def save(directory, number, result, grid, planning, states, footprints, current, target=None):
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
    stem = directory/f'blocked_path_{number:03d}'
    np.savez_compressed(str(stem)+'.npz', data=grid.data, planning_data=planning.data,
                        planning_blocked=planning.blocked, resolution=grid.resolution,
                        origin=grid.origin, states=states, current=current,
                        tractor=footprints.world(current)[0], trailer=footprints.world(current)[1],
                        blocked_tractor=footprints.world(result.get('first_blocked_state', current))[0],
                        blocked_trailer=footprints.world(result.get('first_blocked_state', current))[1],
                        target=target.polygon() if target else np.empty((0, 2)))
    Path(str(stem)+'.json').write_text(json.dumps(result, indent=2))


def render(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for file in Path(directory).glob('blocked_path_*.npz'):
        z=np.load(file); result=json.loads(file.with_suffix('.json').read_text())
        fig, ax=plt.subplots(figsize=(12, 7)); x,y=z['origin'][:2]; r=float(z['resolution']); h,w=z['data'].shape
        ax.imshow(z['data'], origin='lower', extent=[x,x+w*r,y,y+h*r], cmap='gray_r', vmin=-1, vmax=100)
        s=z['states']; index=result['path_index']
        ax.plot(s[:,0],s[:,1],color='cyan',alpha=.4,label='whole plan')
        ax.plot(s[index:,0],s[index:,1],color='blue',label='remaining plan')
        for key,color in [('tractor','green'),('trailer','green'),('blocked_tractor','red'),('blocked_trailer','red'),('target','orange')]:
            poly=z[key]
            if len(poly):
                poly=np.vstack([poly,poly[0]]); ax.plot(poly[:,0],poly[:,1],color=color)
        cells=result['blocked_cells']
        if cells:
            xy=np.array([c['xy'] for c in cells]);ax.scatter(xy[:,0],xy[:,1],marker='x',color='magenta',label='blocking cells')
        ax.set_xlim(s[:,0].min()-4,s[:,0].max()+3);ax.set_ylim(s[:,1].min()-3,s[:,1].max()+3)
        ax.set_aspect('equal');ax.legend();ax.set_title(f"{file.stem}: {result.get('blocked_body')} index {result.get('first_blocked_path_state_index')}")
        fig.tight_layout();fig.savefig(file.with_suffix('.png'),dpi=150);plt.close(fig)
