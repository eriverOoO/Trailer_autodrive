"""Wall-free geometric hypotheses. No semantic vehicle classification or actuation."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import label


@dataclass
class Config:
    cluster_min_cells: int = 4
    cluster_min_length: float = .35
    cluster_max_length: float = 1.6
    cluster_max_width: float = .95
    row_tolerance: float = .35
    min_gap: float = 2.6
    max_gap: float = 7.0
    bay_width: float = .95
    margin: float = .18
    min_free_fraction: float = .90
    min_observations: int = 3
    min_stationary_seconds: float = 2.0
    association_distance: float = .3
    max_track_age: float = 3.0


class Detector:
    def __init__(self, config=None):
        self.config = config or Config()
        self.tracks = []
        self.next_id = 1
        self.last_stamp = None

    def update(self, data, resolution, origin, pose, stamp):
        """Grid origin and ego pose are [x,y,yaw] in the SAME fixed frame.

        Update once per distinct map stamp. Unknown never proves clearance.
        Static observations are measured in map coordinates, not ego coordinates.
        """
        c = self.config
        if not math.isfinite(stamp) or resolution <= 0:
            raise ValueError('finite stamp and positive resolution required')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return {'ready': False, 'reason': 'duplicate_or_reversed_map_stamp', 'objects': [], 'gaps': []}
        self.last_stamp = stamp
        data = np.asarray(data)
        co, so = math.cos(origin[2]), math.sin(origin[2])
        rot = np.array([[co, -so], [so, co]])
        cp, sp = math.cos(pose[2]), math.sin(pose[2])
        road = np.array([[cp, -sp], [sp, cp]])
        components, count = label(data >= 52, structure=np.ones((3, 3)))
        objects = []
        old = [t for t in self.tracks if stamp-t['last'] <= c.max_track_age]
        used = set()
        for k in range(1, count+1):
            ij = np.argwhere(components == k)
            if len(ij) < c.cluster_min_cells:
                continue
            world = (ij[:, ::-1]+.5)*resolution @ rot.T + np.asarray(origin[:2])
            local = (world-np.asarray(pose[:2])) @ road
            lo, hi = local.min(axis=0)-resolution/2, local.max(axis=0)+resolution/2
            extent = hi-lo
            # Existing road/ego heading is a prior, not a measured object heading.
            if not (c.cluster_min_length <= extent[0] <= c.cluster_max_length and
                    extent[1] <= c.cluster_max_width and hi[1] < -.15 and hi[0] > 0):
                continue
            center = world.mean(axis=0)
            eligible = [(np.linalg.norm(center-t['anchor']), i, t)
                        for i, t in enumerate(old) if i not in used]
            match = min(eligible, default=None, key=lambda item: item[0])
            if match is not None and match[0] <= c.association_distance:
                _, i, track = match
                used.add(i)
                track = dict(track, last=stamp, hits=track['hits']+1)
            else:
                track = dict(id=self.next_id, anchor=center, first=stamp, last=stamp, hits=1)
                self.next_id += 1
            stationary = (track['hits'] >= c.min_observations and
                          stamp-track['first'] >= c.min_stationary_seconds)
            objects.append(dict(id=track['id'], kind='vehicle_sized_static_candidate' if stationary
                                else 'unconfirmed_object', stationary=stationary,
                                center=center.tolist(), bounds=[lo.tolist(), hi.tolist()],
                                extent=extent.tolist(), hits=track['hits'], track=track))
        self.tracks = [o.pop('track') for o in objects]
        gaps = []
        ordered = sorted(objects, key=lambda o: o['bounds'][0][0])
        for a, b in zip(ordered, ordered[1:]):
            if not (a['stationary'] and b['stationary']):
                continue
            alo, ahi = np.array(a['bounds']); blo, bhi = np.array(b['bounds'])
            start, end = ahi[0], blo[0]
            if not c.min_gap <= end-start <= c.max_gap:
                continue
            if abs(ahi[1]-bhi[1]) > c.row_tolerance:
                continue
            # Near-side surfaces locate row. Bay depth is configured, not observed.
            front = min(ahi[1], bhi[1]); back = front-c.bay_width
            xs = np.arange(start+c.margin, end-c.margin, resolution)
            ys = np.arange(back+c.margin, front-c.margin, resolution)
            if len(xs) == 0 or len(ys) == 0:
                continue
            xx, yy = np.meshgrid(xs, ys)
            w = np.c_[xx.ravel(), yy.ravel()] @ road.T + np.asarray(pose[:2])
            cells = np.floor((w-np.asarray(origin[:2])) @ rot/resolution).astype(int)
            valid = ((cells[:, 0] >= 0) & (cells[:, 0] < data.shape[1]) &
                     (cells[:, 1] >= 0) & (cells[:, 1] < data.shape[0]))
            values = np.full(len(cells), -1)
            values[valid] = data[cells[valid, 1], cells[valid, 0]]
            free = float(np.mean((values >= 0) & (values < 50)))
            occupied = int(np.count_nonzero(values >= 50))
            center = np.array([(start+end)/2, (front+back)/2]) @ road.T + pose[:2]
            gaps.append(dict(pair=[a['id'], b['id']], center=center.tolist(), yaw=float(pose[2]),
                             length=float(end-start), width=c.bay_width, width_source='configured_prior',
                             free_fraction=free, occupied_samples=occupied,
                             accepted=free >= c.min_free_fraction and occupied == 0))
        return dict(ready=any(g['accepted'] for g in gaps), reason='geometric_hypotheses_only',
                    objects=objects, gaps=gaps)


def scan_clusters(ranges, angle_min, angle_increment, range_min, range_max,
                  distance_threshold=.20, minimum_points=4):
    """Segment consecutive valid beams; range-adaptive threshold, no gap bridging.

    Results are in scan frame. This is geometry only: chairs/boxes may resemble cars.
    """
    clusters = []; current = []; previous = None
    for i, r in enumerate(ranges):
        valid = math.isfinite(r) and range_min <= r <= range_max
        if valid:
            angle = angle_min+i*angle_increment
            point = np.array([r*math.cos(angle), r*math.sin(angle)])
            threshold = distance_threshold + 2*r*abs(angle_increment)
            split = previous is not None and np.linalg.norm(point-previous) > threshold
        else:
            split = True
        if split:
            if len(current) >= minimum_points:
                clusters.append(np.array(current))
            current = []
        if valid:
            current.append(point); previous = point
        else:
            previous = None
    if len(current) >= minimum_points:
        clusters.append(np.array(current))
    return [dict(points=len(points), center=points.mean(axis=0).tolist(),
                 extent=np.ptp(points, axis=0).tolist()) for points in clusters]
