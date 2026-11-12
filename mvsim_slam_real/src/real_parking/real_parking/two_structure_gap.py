"""Opt-in two-structure bay geometry; no rear wall or invented free cells."""

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

    def __post_init__(self):
        if not all(math.isfinite(float(v)) for v in vars(self).values()):
            raise ValueError('Two-structure parameters must be finite')
        if not (self.slot_width > 2*self.interior_margin_m > 0
                and 0 < self.min_gap_length < self.max_gap_length
                and 0 <= self.min_lateral_m < self.max_lateral_m
                and self.min_longitudinal_m < self.max_longitudinal_m
                and 0 < self.max_heading_deg <= 30
                and self.cluster_join_m > 0 and self.max_structure_extent_m > 0
                and self.min_structure_cells >= 3 and self.camera_match_distance_m > 0):
            raise ValueError('Invalid two-structure dimensions or thresholds')


@dataclass
class PairObservation(GapObservation):
    lidar_target: Target | None = None
    components: tuple = ()
    free_fraction: float = 0.0
    camera_confirmed: bool = False


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


def observed_interior(grid, target, margin):
    """Check every raster cell intersecting the inset interior; unknown stays blocked."""
    corners = target.polygon()
    ij = grid.world_to_cell(corners)
    lo, hi = ij.min(axis=0)-1, ij.max(axis=0)+2
    if np.any(lo < 0) or hi[0] > grid.width or hi[1] > grid.height:
        return False, 0.0
    jj, ii = np.mgrid[lo[1]:hi[1], lo[0]:hi[0]]
    xy = grid.cell_to_world(np.c_[ii.ravel(), jj.ravel()])
    local = transform(xy-target.center, [0, 0], -target.heading)
    # Include cell footprints at the inset boundary, not only point samples.
    pad = grid.resolution/math.sqrt(2)
    keep = ((np.abs(local[:, 0]) <= target.length/2-margin+pad)
            & (np.abs(local[:, 1]) <= target.width/2-margin+pad))
    values = grid.data[jj.ravel()[keep], ii.ravel()[keep]]
    if not len(values):
        return False, 0.0
    free = (values >= 0) & (values < 50)
    return bool(np.all(free)), float(np.mean(free))


def observe_two_structure_gap(grid, road_pose, config=PairConfig(), *,
                              camera_evidence=(), require_camera=True):
    """Measure left-side facing edges and apply a configurable lateral depth.

    Heading is the line between the two road-facing structure anchors, bounded
    by the initial road-heading prior. Unknown space is never filled here.
    """
    road_pose = np.asarray(road_pose, dtype=float)
    if road_pose.shape != (3,) or not np.isfinite(road_pose).all():
        raise ValueError('Expected finite initial road pose [x,y,yaw]')
    result = PairObservation(reason='waiting for two observed left-side structures')
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
    components = []
    # Iterate labels of occupied cells only, not all dilated background labels.
    for k in np.unique(labels[mask]):
        cells = np.argwhere(mask & (labels == k))
        if len(cells) < config.min_structure_cells:
            continue
        points = grid.cell_to_world(cells[:, ::-1])
        local = transform(points-road_pose[:2], [0, 0], -road_pose[2])
        if np.max(np.ptp(local, axis=0)) > config.max_structure_extent_m:
            continue
        anchor = np.array([(local[:, 0].min()+local[:, 0].max())/2,
                           local[:, 1].min()])
        components.append((anchor, points))
    components.sort(key=lambda value: value[0][0])
    result.first_obstacle = bool(components)
    valid, provisional = [], []
    for (a, first), (b, second) in zip(components, components[1:]):
        angle = math.atan2(b[1]-a[1], b[0]-a[0])
        if abs(angle) > math.radians(config.max_heading_deg):
            continue
        heading = road_pose[2]+angle
        one = transform(first-road_pose[:2], [0, 0], -heading)
        two = transform(second-road_pose[:2], [0, 0], -heading)
        pad = grid.resolution/math.sqrt(2)
        start, end = float(one[:, 0].max()+pad), float(two[:, 0].min()-pad)
        length = end-start
        if not config.min_gap_length <= length <= config.max_gap_length:
            continue
        entrance = float((one[:, 1].min()+two[:, 1].min())/2-pad)
        center = transform([[(start+end)/2, entrance+config.slot_width/2]],
                           road_pose[:2], heading)[0]
        target = Target(center, heading, length, config.slot_width,
                        'two LiDAR structures; fixed lateral depth; no rear-wall assumption')
        free, fraction = observed_interior(grid, target, config.interior_margin_m)
        camera = camera_confirms_pair((first, second), camera_evidence,
                                     config.camera_match_distance_m)
        observation = PairObservation(
            first_obstacle=True, gap_seen=True, second_obstacle=True,
            target=target if free and (camera or not require_camera) else None,
            obstacle_intervals=((float(one[:, 0].min()-pad), start),
                                (end, float(two[:, 0].max()+pad))),
            lidar_target=target, components=(first, second), free_fraction=fraction,
            camera_confirmed=camera,
            reason=('two structures and observed free interior confirmed' if free and camera else
                    'LiDAR-only test: camera confirmation bypassed' if free and not require_camera else
                    'waiting for calibrated camera confirmation of both LiDAR structures' if free else
                    f'gap found; interior blocked or unknown (free={fraction:.3f})'))
        (valid if observation.target is not None else provisional).append(observation)
    choices = valid or provisional
    if choices:
        # Prefer confirmed pairs, then the nearest bay along the initial road.
        result = min(choices, key=lambda obs: float(np.linalg.norm(obs.lidar_target.center-road_pose[:2])))
    return result
