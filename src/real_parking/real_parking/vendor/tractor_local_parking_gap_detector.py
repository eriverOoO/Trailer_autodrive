"""Find a wall-backed parking gap in the observed local OccupancyGrid.

Only occupied and observed-free cells are used. The road pose supplies the
parking-side sign and a loose direction prior, never an obstacle location.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import binary_closing, label

from tractor_parking_perception import Target
from tractor_trailer_model import transform


@dataclass
class GapObservation:
    first_obstacle: bool = False
    gap_seen: bool = False
    second_obstacle: bool = False
    target: Target | None = None
    obstacle_intervals: tuple = ()
    reason: str = ''


def _runs(mask, minimum):
    labels, count = label(mask)
    return [(int(np.flatnonzero(labels == k)[0]),
             int(np.flatnonzero(labels == k)[-1]) + 1)
            for k in range(1, count + 1)
            if np.count_nonzero(labels == k) >= minimum]


def observe_parked_car_gap(grid, road_pose, slot_length=5.8, slot_width=2.0,
                car_width=1.7):
    """Return observation progress and a target when both car faces are seen.

    The longest road-parallel occupied line on the parking side is the wall.
    Obstacle evidence is projected into longitudinal bins in front of that
    line, so cars may be connected to the wall in the raster map.
    """
    result = GapObservation()
    cells = np.argwhere(grid.data >= 52)
    if len(cells) < 30:
        result.reason = 'insufficient occupied evidence'
        return result
    xy = grid.cell_to_world(cells[:, ::-1])
    local = transform(xy - road_pose[:2], [0, 0], -road_pose[2])
    # x is forward and depth is positive towards the parking side.
    u0, depth0 = local[:, 0], -local[:, 1]
    side = (u0 > -2) & (u0 < 35) & (depth0 > 4) & (depth0 < 9)
    if np.count_nonzero(side) < 30:
        result.reason = 'parking-side wall not observed'
        return result
    wall_local = local[side]
    best = None
    # Search small rotations because the road pose can drift slightly.
    for angle in np.deg2rad(np.arange(-10, 10.1, 1.0)):
        c, s = math.cos(angle), math.sin(angle)
        along = wall_local[:, 0]*c + wall_local[:, 1]*s
        depth = wall_local[:, 0]*s - wall_local[:, 1]*c
        bands = np.floor(depth/.18).astype(int)
        for band in np.unique(bands):
            chosen = bands == band
            if np.count_nonzero(chosen) < 25:
                continue
            distinct = np.unique(np.floor(along[chosen]/.20)).size
            span = np.ptp(along[chosen])
            score = distinct + .15*span
            if span >= 8 and (best is None or score > best[0]):
                best = (score, angle, float(np.median(depth[chosen])))
    if best is None:
        result.reason = 'long parking wall not observed'
        return result
    angle, wall_depth = best[1:]
    c, s = math.cos(angle), math.sin(angle)
    wall_along = wall_local[:, 0]*c + wall_local[:, 1]*s
    wall_depths = wall_local[:, 0]*s - wall_local[:, 1]*c
    wall_points = np.abs(wall_depths-wall_depth) < .20
    wall_min, wall_max = np.quantile(wall_along[wall_points], [.02, .98])
    if wall_max-wall_min < 8:
        result.reason = 'wall span too short'
        return result
    # Ignore the wall itself and project any protruding returns by x. No
    # connected-component assumption is made about the two parked cars.
    along = local[:, 0]*c + local[:, 1]*s
    depth = local[:, 0]*s - local[:, 1]*c
    # The far car can come into range before the wall immediately behind it.
    # The near car can occlude the wall behind it. Keep its returns even when
    # the visible wall starts several metres past the car's front face.
    protruding = (depth > wall_depth-3.35) & (depth < wall_depth-.45) & \
                 (along > wall_min-4.5) & (along < wall_max+2.5)
    bin_size = .20
    bin_origin = wall_min-4.5
    n = int(math.ceil((wall_max+2.5-bin_origin)/bin_size))
    bins = np.floor((along[protruding]-bin_origin)/bin_size).astype(int)
    counts = np.bincount(np.clip(bins, 0, n-1), minlength=n)
    occupied_columns = binary_closing(counts >= 2, structure=np.ones(4))
    runs = _runs(occupied_columns, minimum=max(5, int(1.0/bin_size)))
    intervals = tuple((bin_origin+a*bin_size, bin_origin+b*bin_size) for a, b in runs)
    result.obstacle_intervals = intervals
    result.first_obstacle = bool(intervals)
    if len(intervals) < 2:
        result.reason = 'waiting for second parked obstacle'
        return result
    candidates = []
    for first, second in zip(intervals, intervals[1:]):
        gap_start, gap_end = first[1], second[0]
        available = gap_end-gap_start
        if not (slot_length+.3 <= available <= slot_length+4.5):
            continue
        center_u = (gap_start+gap_end)/2
        # Inspect only the desired bay interior. Unknown is never called free.
        sample_u = np.arange(center_u-slot_length/2+.2,
                             center_u+slot_length/2-.15, .20)
        # Front faces of the two cars locate the parking row relative to the
        # wall. Their measured near-side depth is the smaller depth value.
        car_mask = protruding & (((along >= first[0]) & (along <= first[1])) |
                                  ((along >= second[0]) & (along <= second[1])))
        if np.count_nonzero(car_mask) < 15:
            continue
        near_depth = float(np.quantile(depth[car_mask], .08))
        center_depth = near_depth + car_width/2
        if not (.6 < wall_depth-center_depth < 2.6):
            continue
        sample_d = np.arange(center_depth-slot_width/2+.2,
                             center_depth+slot_width/2-.15, .20)
        uu, dd = np.meshgrid(sample_u, sample_d)
        # Transform axis coordinates back through the road frame to map frame.
        road_xy = np.c_[uu.ravel()*c+dd.ravel()*s,
                        uu.ravel()*s-dd.ravel()*c]
        world = transform(road_xy, road_pose[:2], road_pose[2])
        ij = grid.world_to_cell(world)
        inside = ((ij[:, 0] >= 0) & (ij[:, 1] >= 0) &
                  (ij[:, 0] < grid.width) & (ij[:, 1] < grid.height))
        values = np.full(len(ij), -1)
        values[inside] = grid.data[ij[inside, 1], ij[inside, 0]]
        free_fraction = float(np.mean((values >= 0) & (values < 50)))
        if free_fraction < .70 or np.any(values >= 50):
            continue
        center_road = np.array([center_u*c+center_depth*s,
                                center_u*s-center_depth*c])
        center = transform([center_road], road_pose[:2], road_pose[2])[0]
        confidence = f'observed free={free_fraction:.2f}, obstacle gap={available:.2f}m'
        candidates.append((free_fraction, Target(center, road_pose[2]+angle,
                                                  slot_length, slot_width, confidence)))
    if not candidates:
        result.gap_seen = True
        result.reason = 'two obstacles seen; bay interior not yet sufficiently observed'
        return result
    result.gap_seen = result.second_obstacle = True
    result.target = max(candidates, key=lambda item: item[0])[1]
    return result


def observe_gap(grid, road_pose, slot_length=5.8, slot_width=2.0,
                car_width=1.7):
    """Measure an open-front U cavity from occupied faces and observed free cells.

    Road orientation is only a direction prior. Dimensions and all three wall
    positions come from the grid; slot dimensions provide broad size gates.
    """
    result = GapObservation(reason='waiting for U-shaped occupied boundaries')
    cells = np.argwhere(grid.data >= 52)
    if len(cells) < 30:
        return result
    xy = grid.cell_to_world(cells[:, ::-1])
    local = transform(xy-road_pose[:2], [0, 0], -road_pose[2])
    local = local[(local[:, 1] < -1.5) & (local[:, 1] > -10)]
    if len(local) < 30:
        return result
    candidates = []
    bin_size = grid.resolution
    for angle in np.deg2rad(np.arange(-5, 5.01, .5)):
        c, s = math.cos(angle), math.sin(angle)
        u = local[:, 0]*c + local[:, 1]*s
        d = local[:, 0]*s - local[:, 1]*c
        origin = math.floor(u.min()/bin_size)*bin_size
        n = int(math.ceil((u.max()-origin)/bin_size))+1
        # Long near-side horizontal surfaces reveal an opening between solids.
        for band in np.unique(np.floor(d/.15).astype(int)):
            face = (d >= band*.15) & (d < (band+1)*.15)
            if np.count_nonzero(face) < 20:
                continue
            front = float(np.median(d[face]))
            near = abs(d-front) < .13
            counts = np.bincount(np.floor((u[near]-origin)/bin_size).astype(int), minlength=n)
            runs = _runs(binary_closing(counts > 0, structure=np.ones(3)), 8)
            for left, right in zip(runs, runs[1:]):
                a = origin+left[1]*bin_size
                b = origin+right[0]*bin_size
                length = b-a
                if not (.75*slot_length < length < 1.35*slot_length):
                    continue
                result.first_obstacle = result.gap_seen = True
                result.obstacle_intervals = ((origin+left[0]*bin_size, a),
                                             (b, origin+right[1]*bin_size))
                back_mask = (u > a+.2) & (u < b-.2) & (d > front+.65*slot_width) & (d < front+1.5*slot_width)
                if np.count_nonzero(back_mask) < 20:
                    continue
                depths = np.floor(d[back_mask]/.15).astype(int)
                back_band = max(np.unique(depths), key=lambda k: np.count_nonzero(depths == k))
                back = float(np.median(d[back_mask][depths == back_band]))
                wall = back_mask & (abs(d-back) < .13)
                coverage = np.unique(np.floor((u[wall]-a)/.2)).size*.2/length
                if coverage < .70:
                    continue
                # Both perpendicular inner faces must extend into the cavity.
                # Raster cells represent an area: include the half-cell at
                # each depth gate so a tiny road-yaw change cannot remove a
                # genuine sparse side-face endpoint. Span/count gates remain.
                side_depth = (d > front+.20-bin_size/2) & (d < back-.15+bin_size/2)
                faces = []
                for boundary in (a, b):
                    side = side_depth & (abs(u-boundary) < .25)
                    if np.count_nonzero(side) < 5 or np.ptp(d[side]) < .55*(back-front):
                        break
                    faces.append(float(np.median(u[side])))
                if len(faces) != 2:
                    continue
                # Use the free-facing edges of occupied raster cells.
                a, b = faces[0]+bin_size/2, faces[1]-bin_size/2
                back -= bin_size/2
                front -= bin_size/2
                length, width = b-a, back-front
                if not (.65*slot_width < width < 1.4*slot_width):
                    continue
                uu, dd = np.meshgrid(np.arange(a+.18,b-.15,bin_size),
                                     np.arange(front+.18,back-.15,bin_size))
                road_xy = np.c_[uu.ravel()*c+dd.ravel()*s,
                                uu.ravel()*s-dd.ravel()*c]
                ij = grid.world_to_cell(transform(road_xy,road_pose[:2],road_pose[2]))
                valid = (ij[:,0]>=0)&(ij[:,0]<grid.width)&(ij[:,1]>=0)&(ij[:,1]<grid.height)
                values = np.full(len(ij),-1)
                values[valid]=grid.data[ij[valid,1],ij[valid,0]]
                free = float(np.mean((values>=0)&(values<50)))
                if free < .90 or np.any(values>=50):
                    result.reason = f'U boundaries seen; interior free={free:.2f}, occupied={np.count_nonzero(values>=50)}'
                    continue
                center_u, center_d = (a+b)/2, (front+back)/2
                center = transform([[center_u*c+center_d*s,center_u*s-center_d*c]],road_pose[:2],road_pose[2])[0]
                target = Target(center, road_pose[2]+angle, length, width,
                                f'U map faces; free={free:.3f}, back coverage={coverage:.2f}, size={length:.2f}x{width:.2f}')
                candidates.append((coverage-abs(angle)*.1, target, result.obstacle_intervals))
    if candidates:
        _, result.target, result.obstacle_intervals = max(candidates,key=lambda v:v[0])
        result.first_obstacle = result.gap_seen = result.second_obstacle = True
        result.reason = 'three observed U boundaries and free cavity'
    return result
