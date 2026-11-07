"""Size display outlines from the shared vehicle geometry, in metres."""
from pathlib import Path

import numpy as np
import yaml


def default_geometry_file():
    # Prefer the editable source in this workspace, including symlink installs.
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'
        if candidate.is_file():
            return candidate
    from ament_index_python.packages import get_package_share_directory
    return (Path(get_package_share_directory('mvsim_slam_real_bringup'))
            / 'config/vehicle_geometry.yaml')


def sized_polygons(polygons, geometry_file):
    geometry = yaml.safe_load(Path(geometry_file).read_text())
    result = []
    for role, polygon in zip(('tractor', 'trailer'), polygons):
        target = np.array([geometry[role + '_length'], geometry[role + '_width']], dtype=float)
        if not np.isfinite(target).all() or (target <= 0).any():
            raise ValueError(role + ' length and width must be finite and positive')
        points = np.asarray(polygon, dtype=float)
        span = np.ptp(points, axis=0)
        if not np.isfinite(points).all() or (span <= 0).any():
            raise ValueError(role + ' outline must have finite, nonzero dimensions')
        # Scale about the existing rear-axle / hitch origin, preserving its
        # relative location inside the outline and the original outline style.
        result.append(points * (target / span))
    return result
