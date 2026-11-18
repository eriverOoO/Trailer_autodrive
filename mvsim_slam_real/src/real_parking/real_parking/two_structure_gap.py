"""Opt-in two-structure bay geometry; no rear wall required, no invented free cells.

A measured wall behind the gap (e.g. structures standing against a building wall)
sets the bay's inner edge; without one the configured depth is used.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.ndimage import binary_dilation, label
from scipy.spatial import cKDTree

from .bootstrap import load
load()
from tractor_local_parking_gap_detector import GapObservation
from tractor_parking_perception import Target
from tractor_trailer_model import transform


@dataclass(frozen=True)
class PairConfig:
    slot_width: float = 1.10  # Lateral depth, not longitudinal vehicle length.
    min_gap_length: float = 2.60
    max_gap_length: float = 6.0
    min_lateral_m: float = .55
    max_lateral_m: float = 6.0
    min_longitudinal_m: float = -5.0
    max_longitudinal_m: float = 35.0
    max_heading_deg: float = 10.0
    cluster_join_m: float = .10
    max_structure_extent_m: float = 3.0
    min_structure_cells: int = 3
    interior_margin_m: float = .10
    camera_match_distance_m: float = .20
    # Interior tolerance: isolated occupied specks of at most this many connected cells
    # (LiDAR noise) and this fraction of unobserved cells. The planner's own footprint
    # clearance check still decides whether the rig fits.
    max_speck_cells: int = 2
    max_unknown_fraction: float = .10
    # A structure standing against a wall must stick out from it toward the road by at
    # least this much to be split off the wall (structures are >= 0.30 m deep).
    structure_protrusion_m: float = .15
    # Rear boundary evidence: walls (runs longer than max_structure_extent_m) or recessed
    # wall pieces at least this long. Shorter objects inside the bay stay obstacles.
    min_wall_piece_m: float = 1.0

    def __post_init__(self):
        if not all(math.isfinite(float(v)) for v in vars(self).values()):
            raise ValueError('Two-structure parameters must be finite')
        if not (self.slot_width > 2*self.interior_margin_m > 0
                and 0 < self.min_gap_length < self.max_gap_length
                and 0 <= self.min_lateral_m < self.max_lateral_m
                and self.min_longitudinal_m < self.max_longitudinal_m
                and 0 < self.max_heading_deg <= 30
                and self.cluster_join_m > 0 and self.max_structure_extent_m > 0
                and self.min_structure_cells >= 3 and self.camera_match_distance_m > 0
                and self.max_speck_cells >= 0 and 0 <= self.max_unknown_fraction < 1
                and self.structure_protrusion_m > 0 and self.min_wall_piece_m > 0):
            raise ValueError('Invalid two-structure dimensions or thresholds')


@dataclass
class PairTarget(Target):
    depth_observed: bool = True


@dataclass
class PairObservation(GapObservation):
    lidar_target: Target | None = None
    components: tuple = ()
    free_fraction: float = 0.0
    camera_confirmed: bool = False
    rear_wall: bool = False            # bay inner edge set by a measured wall/boundary
    rear_depth_m: float | None = None  # entrance -> measured rear boundary
    walls: tuple = ()                  # long occupied runs (walls) in the left window
    structures: int = 0                # structure candidates found
    rejected_pairs: tuple = ()         # (gap_length_m or None, reason) of pairs not used


def camera_confirms_pair(components, evidence, distance=.20):
    """Require separate, spatially unambiguous image/scan groups for both objects.

    Evidence may come from different frames during a survey. A broad image
    region spanning both structures cannot confirm either one.
    """
    confirmed = set()
    trees = [cKDTree(points) for points in components]
    for points in evidence:
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
            continue
        matches = [i for i, tree in enumerate(trees)
                   if np.mean(tree.query(points)[0] <= distance) >= .8]
        # Even a minority on the other structure makes an association ambiguous.
        touched = [i for i, tree in enumerate(trees)
                   if np.any(tree.query(points)[0] <= distance)]
        if len(matches) == 1 and len(touched) == 1:
            confirmed.add(matches[0])
    return confirmed == {0, 1}


def observed_interior(grid, target, margin, max_unknown_fraction=0.0, max_speck_cells=0):
    """Check every raster cell intersecting the inset interior.

    Returns (ok, free_fraction, unknown_fraction, largest_blob). An occupied cell is
    tolerated only if its whole 8-connected occupied group has at most max_speck_cells
    cells; unknown cells are tolerated up to max_unknown_fraction. The blob size is
    None when the interior cannot be evaluated on this map.
    """
    corners = target.polygon()
    ij = grid.world_to_cell(corners)
    lo, hi = ij.min(axis=0)-1, ij.max(axis=0)+2
    if np.any(lo < 0) or hi[0] > grid.width or hi[1] > grid.height:
        return False, 0.0, 1.0, None
    jj, ii = np.mgrid[lo[1]:hi[1], lo[0]:hi[0]]
    xy = grid.cell_to_world(np.c_[ii.ravel(), jj.ravel()])
    local = transform(xy-target.center, [0, 0], -target.heading)
    # Include cell footprints at the inset boundary, not only point samples.
    pad = grid.resolution/math.sqrt(2)
    keep = ((np.abs(local[:, 0]) <= target.length/2-margin+pad)
            & (np.abs(local[:, 1]) <= target.width/2-margin+pad))
    rows, cols = jj.ravel()[keep], ii.ravel()[keep]
    values = grid.data[rows, cols]
    if not len(values):
        return False, 0.0, 1.0, None
    free = (values >= 0) & (values < 50)
    unknown = float(np.mean(values < 0))
    largest = 0
    occupied = values >= 50
    if occupied.any():
        # Size whole occupied groups in a window wide enough that a group cut by the
        # window edge already exceeds the speck limit.
        extra = int(max_speck_cells)+1
        r0, c0 = max(lo[1]-extra, 0), max(lo[0]-extra, 0)
        window = grid.data[r0:min(hi[1]+extra, grid.height), c0:min(hi[0]+extra, grid.width)] >= 50
        labels, _ = label(window, structure=np.ones((3, 3), dtype=bool))
        sizes = np.bincount(labels.ravel())
        largest = int(sizes[labels[rows[occupied]-r0, cols[occupied]-c0]].max())
    ok = largest <= max_speck_cells and unknown <= max_unknown_fraction
    return bool(ok), float(np.mean(free)), unknown, largest


def split_wall(local, points, config, resolution):
    """Split structures off a long occupied run (a wall) by how far they stick out.

    local/points: road-frame and world coordinates of one connected component longer
    than max_structure_extent_m. Per longitudinal bin the cell nearest the road is kept;
    a line fitted to the wall-level bins is the wall face, and runs of bins nearer than
    structure_protrusion_m are structures. Returns a list of world point arrays.
    """
    u, v = local[:, 0], local[:, 1]
    bins = np.floor((u-u.min())/resolution).astype(int)
    nearest = np.full(bins.max()+1, np.inf)
    np.minimum.at(nearest, bins, v)
    have = np.isfinite(nearest)
    centres = u.min()+(np.arange(len(nearest))+.5)*resolution
    use = have.copy()
    for _ in range(5):
        if use.sum() < 2:
            return []
        slope, intercept = np.polyfit(centres[use], nearest[use], 1)
        line = slope*centres+intercept
        use = have & (nearest >= line-config.structure_protrusion_m)
    if abs(math.atan(slope)) > math.radians(config.max_heading_deg):
        return []
    protruding = np.flatnonzero(have & (nearest < line-config.structure_protrusion_m))
    if not len(protruding):
        return []
    join = max(1, int(math.ceil(config.cluster_join_m/resolution)))
    runs = np.split(protruding, np.flatnonzero(np.diff(protruding) > join+1)+1)
    structures = []
    for run in runs:
        keep = ((bins >= run[0]) & (bins <= run[-1])
                & (v < slope*u+intercept-config.structure_protrusion_m/2))
        if (keep.sum() >= config.min_structure_cells
                and np.max(np.ptp(local[keep], axis=0)) <= config.max_structure_extent_m):
            structures.append(points[keep])
    return structures


def observe_two_structure_gap(grid, road_pose, config=PairConfig(), *,
                              camera_evidence=(), require_camera=True, minimum_goal_width=0.):
    """Measure left-side facing edges; bay depth from a measured rear wall or configured.

    Heading is the line between the two road-facing structure anchors, bounded
    by the initial road-heading prior. Unknown space is never filled here.
    """
    road_pose = np.asarray(road_pose, dtype=float)
    if road_pose.shape != (3,) or not np.isfinite(road_pose).all():
        raise ValueError('Expected finite initial road pose [x,y,yaw]')
    result = PairObservation(reason='waiting for two observed left-side structures (found 0)')
    ji = np.argwhere(grid.data >= 50)
    if not len(ji):
        return result
    xy = grid.cell_to_world(ji[:, ::-1])
    road = transform(xy-road_pose[:2], [0, 0], -road_pose[2])
    keep = ((road[:, 0] > config.min_longitudinal_m) & (road[:, 0] < config.max_longitudinal_m)
            & (road[:, 1] > config.min_lateral_m) & (road[:, 1] < config.max_lateral_m))
    mask = np.zeros_like(grid.data, dtype=bool)
    selected = ji[keep]
    mask[selected[:, 0], selected[:, 1]] = True
    radius = max(1, int(math.ceil(config.cluster_join_m/(2*grid.resolution))))
    labels, _ = label(binary_dilation(mask, iterations=radius))
    components, walls = [], []

    def add(points):
        local = transform(points-road_pose[:2], [0, 0], -road_pose[2])
        anchor = np.array([(local[:, 0].min()+local[:, 0].max())/2, local[:, 1].min()])
        components.append((anchor, points, float(np.ptp(local[:, 0]))))

    # Iterate labels of occupied cells only, not all dilated background labels.
    for k in np.unique(labels[mask]):
        cells = np.argwhere(mask & (labels == k))
        if len(cells) < config.min_structure_cells:
            continue
        points = grid.cell_to_world(cells[:, ::-1])
        local = transform(points-road_pose[:2], [0, 0], -road_pose[2])
        if np.max(np.ptp(local, axis=0)) > config.max_structure_extent_m:
            # A wall, possibly with structures standing against it.
            walls.append(points)
            for structure in split_wall(local, points, config, grid.resolution):
                add(structure)
            continue
        add(points)
    components.sort(key=lambda value: value[0][0])
    wall_points = np.vstack(walls) if walls else np.zeros((0, 2))
    result.first_obstacle = bool(components)
    result.walls = tuple(walls)
    result.structures = len(components)
    if len(components) < 2:
        result.components = tuple(c[1] for c in components)
        result.reason = f'waiting for two observed left-side structures (found {len(components)})'
        return result
    valid, provisional, rejected = [], [], []
    pad = grid.resolution/math.sqrt(2)
    for i, (a, first, _) in enumerate(components):
        for j in range(i+1, min(i+5, len(components))):
            b, second, _ = components[j]
            # Skip only parts recessed behind both faces (rear wall pieces), never a front object.
            between = [c[0][1] for c in components[i+1:j]]
            if between and min(between) < max(a[1], b[1])+config.structure_protrusion_m:
                break
            angle = math.atan2(b[1]-a[1], b[0]-a[0])
            if abs(angle) > math.radians(config.max_heading_deg):
                rejected.append((None, f'pair heading {math.degrees(angle):.1f} deg > {config.max_heading_deg:.1f}'))
                continue
            heading = road_pose[2]+angle
            one = transform(first-road_pose[:2], [0, 0], -heading)
            two = transform(second-road_pose[:2], [0, 0], -heading)
            start, end = float(one[:, 0].max()+pad), float(two[:, 0].min()-pad)
            length = end-start
            if not config.min_gap_length <= length <= config.max_gap_length:
                rejected.append((length, f'gap {length:.2f} m outside {config.min_gap_length:.2f}..'
                                         f'{config.max_gap_length:.2f} m'))
                continue
            entrance = float((one[:, 1].min()+two[:, 1].min())/2-pad)
            # Nearest wall cell behind the entrance inside the gap span is the rear boundary;
            # the bay's inner edge sits on it when within slot_width.
            pieces = [c[1] for c in components[i+1:j] if c[2] >= config.min_wall_piece_m]
            rear_points = np.vstack([wall_points, *pieces])
            depth = None
            if len(rear_points):
                rear = transform(rear_points-road_pose[:2], [0, 0], -heading)
                behind = ((rear[:, 0] > start+pad) & (rear[:, 0] < end-pad)
                          & (rear[:, 1] > entrance+pad) & (rear[:, 1] < entrance+config.slot_width+pad))
                if behind.any():
                    depth = float(rear[behind, 1].min()-pad-entrance)
            inner = entrance+depth if depth is not None else entrance+config.slot_width
            # Keep the rear boundary fixed. A shallow setup may place part of
            # the parked rig on the road side of the two structures.
            goal_width = max(config.slot_width, minimum_goal_width)
            center = transform([[(start+end)/2, inner-goal_width/2]], road_pose[:2], heading)[0]
            target = PairTarget(center, heading, length, goal_width,
                            'two LiDAR structures; inner edge on measured rear wall' if depth is not None
                            else 'two LiDAR structures; fixed lateral depth; no rear wall within depth')
            free, fraction, unknown, blob = observed_interior(
                grid, target, config.interior_margin_m, config.max_unknown_fraction, config.max_speck_cells)
            target.depth_observed = free
            clear_of_obstacles = blob is not None and blob <= config.max_speck_cells
            camera = camera_confirms_pair((first, second), camera_evidence,
                                         config.camera_match_distance_m)
            if blob is None:
                reason = 'bay interior outside usable detection grid'
            elif not clear_of_obstacles:
                reason = (f'gap found; interior blocked or unknown (free={fraction:.3f}, '
                          f'unknown={unknown:.3f}/{config.max_unknown_fraction:.3f}, '
                          f'occupied_group_cells={blob}/{config.max_speck_cells})')
            elif require_camera and not camera:
                reason = 'waiting for calibrated camera confirmation of both LiDAR structures'
            elif not free:
                reason = 'pair confirmed; depth unobserved; verify parking path on measured grid'
            elif camera:
                reason = 'two structures and observed free interior confirmed'
            else:
                reason = 'LiDAR-only test: camera confirmation bypassed'
            observation = PairObservation(
                first_obstacle=True, gap_seen=True, second_obstacle=True,
                target=target if clear_of_obstacles and (camera or not require_camera) else None,
                obstacle_intervals=((float(one[:, 0].min()-pad), start),
                                    (end, float(two[:, 0].max()+pad))),
                lidar_target=target, components=(first, second), free_fraction=fraction,
                camera_confirmed=camera, rear_wall=depth is not None, rear_depth_m=depth,
                walls=tuple(walls), structures=len(components),
                reason=reason)
            (valid if observation.target is not None else provisional).append(observation)
    choices = valid or provisional
    if choices:
        # Prefer confirmed pairs, then the nearest bay along the initial road.
        result = min(choices, key=lambda obs: (not obs.lidar_target.depth_observed,
                                              np.linalg.norm(obs.lidar_target.center-road_pose[:2])))
    else:
        result.components = tuple(c[1] for c in components)
        result.reason = (f'{len(components)} structures found but no usable pair: '
                         + ('; '.join(r for _, r in rejected[:3]) or 'an object in front sits between them'))
    result.rejected_pairs = tuple(rejected)
    return result
