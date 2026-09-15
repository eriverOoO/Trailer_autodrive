"""Sensor validation and LaserScan geometry, independent of ROS."""
import math
from .core import footprints, trailer_pose, rectangle


def fresh(stamp, now, timeout):
    return math.isfinite(stamp) and math.isfinite(now) and 0 <= now-stamp <= timeout


def inside(point, polygon):
    signs = []
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        signs.append((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0]))
    return all(s >= -1e-9 for s in signs) or all(s <= 1e-9 for s in signs)


def scan_obstacles(ranges, angle_min, increment, range_min, range_max, pose, state, geometry, offset):
    """Use real scan angle metadata, reject invalid scans, mask only own bodies.

    Infinity is a valid no-return ray; NaN, zero and out-of-range samples are
    invalid. Contiguous hits become conservative axis-aligned obstacle boxes.
    """
    if (len(ranges) < 8 or not all(math.isfinite(v) for v in
            (angle_min, increment, range_min, range_max)) or increment == 0 or
            range_min < 0 or range_max <= range_min):
        raise ValueError('Invalid LaserScan metadata')
    own = footprints(state, geometry, 0.03)
    x, y, yaw = pose
    x += offset[0]*math.cos(yaw)-offset[1]*math.sin(yaw)
    y += offset[0]*math.sin(yaw)+offset[1]*math.cos(yaw)
    groups, group, valid = [], [], 0
    for i, distance in enumerate(ranges):
        if distance == math.inf:
            valid += 1
            if group:
                groups.append(group)
                group = []
            continue
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            if group:
                groups.append(group)
                group = []
            continue
        valid += 1
        angle = yaw+offset[2]+angle_min+i*increment
        p = (x+distance*math.cos(angle), y+distance*math.sin(angle))
        if any(inside(p, poly) for poly in own):
            continue
        if group and math.dist(p, group[-1]) > 0.4:
            groups.append(group)
            group = []
        group.append(p)
    if group:
        groups.append(group)
    if valid < max(8, len(ranges)*0.25):
        raise ValueError('Insufficient valid LaserScan samples')
    obstacles = []
    for group in groups:
        xs, ys = zip(*group)
        xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
        obstacles.append(rectangle((xmin+xmax)/2, (ymin+ymax)/2, 0,
            (xmax-xmin)/2+0.10, (xmax-xmin)/2+0.10, ymax-ymin+0.20))
    return tuple(obstacles)
