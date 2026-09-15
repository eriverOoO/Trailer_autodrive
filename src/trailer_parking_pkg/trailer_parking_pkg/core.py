"""Off-axle tractor/trailer kinematics, footprint checks and Hybrid A*.

Metres/radians throughout. State origin is the tractor REAR AXLE, +x forward,
+y left. beta = trailer_yaw - tractor_yaw (same sign as the hitch plugin).
"""
from dataclasses import dataclass
import heapq
import itertools
import math
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Geometry:
    wheelbase: float = 2.86
    hitch_offset: float = 0.95
    trailer_axle: float = 3.85
    front: float = 3.69
    rear: float = 0.8604965
    width: float = 2.197298
    max_steer: float = 0.60
    # Stock Prius bumpers overlap around 22..25 degrees despite a 45-degree
    # mechanical hinge limit. Keep 20 degrees for this short coupling.
    max_beta: float = math.radians(20)

    def __post_init__(self):
        values = vars(self)
        if not all(math.isfinite(v) and v > 0 for v in values.values()):
            raise ValueError('Geometry values must be finite and positive')
        if self.max_beta >= math.pi / 2 or self.max_steer >= math.pi / 2:
            raise ValueError('Invalid steering/articulation limit')

    @property
    def rig_length(self):
        return self.front + self.hitch_offset + self.trailer_axle + self.rear


@dataclass(frozen=True)
class State:
    x: float
    y: float
    yaw: float
    beta: float

    def valid(self):
        return all(math.isfinite(v) for v in vars(self).values())


def trailer_pose(s, g):
    yaw = wrap(s.yaw + s.beta)
    return (s.x - g.hitch_offset * math.cos(s.yaw) - g.trailer_axle * math.cos(yaw),
            s.y - g.hitch_offset * math.sin(s.yaw) - g.trailer_axle * math.sin(yaw), yaw)


def advance(s, distance, steer, g):
    """RK4 integration by signed tractor distance, valid in forward/reverse."""
    if not s.valid() or not math.isfinite(distance) or not math.isfinite(steer):
        raise ValueError('Non-finite kinematic input')
    if abs(steer) > g.max_steer + 1e-9:
        raise ValueError('Steering outside model limit')
    curvature = math.tan(steer) / g.wheelbase

    def rate(q):
        _, _, yaw, beta = q
        trailer_rate = (-math.sin(beta) - g.hitch_offset * curvature * math.cos(beta)) / g.trailer_axle
        return (math.cos(yaw), math.sin(yaw), curvature, trailer_rate - curvature)

    q = (s.x, s.y, s.yaw, s.beta)
    a = rate(q)
    b = rate(tuple(v + distance * k / 2 for v, k in zip(q, a)))
    c = rate(tuple(v + distance * k / 2 for v, k in zip(q, b)))
    d = rate(tuple(v + distance * k for v, k in zip(q, c)))
    r = [v + distance * (i + 2*j + 2*k + l) / 6 for v, i, j, k, l in zip(q, a, b, c, d)]
    return State(r[0], r[1], wrap(r[2]), wrap(r[3]))


def rectangle(x, y, yaw, front, rear, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((x + a*c - b*s, y + a*s + b*c)
                 for a, b in ((front, width/2), (-rear, width/2),
                              (-rear, -width/2), (front, -width/2)))


def footprints(s, g, margin=0.0):
    a = rectangle(s.x, s.y, s.yaw, g.front+margin, g.rear+margin, g.width+2*margin)
    x, y, yaw = trailer_pose(s, g)
    b = rectangle(x, y, yaw, g.front+margin, g.rear+margin, g.width+2*margin)
    # Include the short exposed hitch between the two bumpers.
    hx = s.x - g.hitch_offset * math.cos(s.yaw)
    hy = s.y - g.hitch_offset * math.sin(s.yaw)
    bar = rectangle(hx, hy, yaw, 0.12+margin, 0.20+margin, 0.15+2*margin)
    return a, b, bar


def overlaps(a, b):
    """Separating-axis test for convex polygons; touching counts as collision."""
    for poly in (a, b):
        for p, q in zip(poly, poly[1:] + poly[:1]):
            nx, ny = p[1]-q[1], q[0]-p[0]
            pa = [x*nx+y*ny for x, y in a]
            pb = [x*nx+y*ny for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


@dataclass(frozen=True)
class Scene:
    obstacles: tuple = ()
    bounds: tuple = (-30.0, 30.0, -30.0, 30.0)
    margin: float = 0.12

    def free(self, s, g):
        if not s.valid() or abs(s.beta) > g.max_beta:
            return False
        xmin, xmax, ymin, ymax = self.bounds
        for poly in footprints(s, g, self.margin):
            if any(not (xmin <= x <= xmax and ymin <= y <= ymax) for x, y in poly):
                return False
            for obstacle in self.obstacles:
                # Cheap bounding-box rejection before SAT.
                if (max(p[0] for p in poly) < min(p[0] for p in obstacle) or
                    min(p[0] for p in poly) > max(p[0] for p in obstacle) or
                    max(p[1] for p in poly) < min(p[1] for p in obstacle) or
                    min(p[1] for p in poly) > max(p[1] for p in obstacle)):
                    continue
                if overlaps(poly, obstacle):
                    return False
        return True


@dataclass(frozen=True)
class Slot:
    x: float
    y: float
    yaw: float
    length: float
    width: float

    def goal(self, g):
        # Shift rear axle so the ENTIRE straight rig is centred in the slot.
        shift = g.rig_length/2 - g.front
        return State(self.x + shift*math.cos(self.yaw),
                     self.y + shift*math.sin(self.yaw), self.yaw, 0.0)

    def contains(self, s, g, clearance=0.08):
        c, sn = math.cos(self.yaw), math.sin(self.yaw)
        for poly in footprints(s, g):
            for x, y in poly:
                dx, dy = x-self.x, y-self.y
                if (abs(dx*c+dy*sn) > self.length/2-clearance or
                    abs(-dx*sn+dy*c) > self.width/2-clearance):
                    return False
        return True


@dataclass(frozen=True)
class Step:
    state: State
    direction: int
    steer: float


def goal_reached(s, slot, g):
    target = slot.goal(g)
    return (math.hypot(s.x-target.x, s.y-target.y) < 0.35 and
            abs(wrap(s.yaw-target.yaw)) < math.radians(4) and
            abs(s.beta) < math.radians(4) and slot.contains(s, g))


class PlanningError(RuntimeError):
    pass


def plan(start, slot, scene, g=Geometry(), max_nodes=90000, timeout=45.0):
    """Bounded weighted Hybrid A* over (x, y, yaw, beta, direction).

    Motion primitives are integrated/collision-checked every 0.10 m. Search
    retains direction in its key because reversing carries an explicit cost.
    It is a feasible-path search, not an optimality guarantee.
    """
    if slot.length < g.rig_length+0.16 or slot.width < g.width+0.16:
        raise PlanningError('Slot too small for the complete articulated rig')
    if not scene.free(start, g) or not scene.free(slot.goal(g), g):
        raise PlanningError('Start or goal footprint is occupied/out of bounds')
    if goal_reached(start, slot, g):
        return [Step(start, 0, 0)]
    began = time.monotonic()
    try:
        from .optimizer import shooting_plan
    except ImportError:
        shooting_plan = None
    if shooting_plan is not None:
        candidate = shooting_plan(start, slot, scene, g, timeout=timeout*0.7)
        if candidate:
            return candidate
    timeout = max(0.0, timeout-(time.monotonic()-began))
    goal = slot.goal(g)
    def key(s, direction):
        return (round(s.x/0.35), round(s.y/0.35), round(wrap(s.yaw)/0.10),
                round(s.beta/0.10), direction)
    def heuristic(s):
        tx, ty, _ = trailer_pose(s, g)
        gx, gy, _ = trailer_pose(goal, g)
        return (math.hypot(s.x-goal.x, s.y-goal.y) +
                0.7*math.hypot(tx-gx, ty-gy) +
                2.0*abs(wrap(s.yaw-goal.yaw)) + 2.0*abs(s.beta))

    # Immutable records avoid corrupting parent chains when a grid cell reopens.
    records = [(start, 0, 0.0, -1, (), 0.0)]
    serial = itertools.count()
    queue = [(heuristic(start), next(serial), 0)]
    costs = {key(start, 0): 0.0}
    deadline = time.monotonic() + timeout
    beta_targets = tuple((g.max_beta-0.02)*f for f in (-1, -0.5, 0, 0.5, 1))
    expanded = 0
    while queue and expanded < max_nodes and time.monotonic() < deadline:
        _, _, index = heapq.heappop(queue)
        s, old_dir, old_steer, _, _, cost = records[index]
        if cost > costs.get(key(s, old_dir), math.inf) + 1e-9:
            continue
        if goal_reached(s, slot, g):
            segments = []
            while index > 0:
                rec = records[index]
                segments.append(rec[4])
                index = rec[3]
            return [Step(start, 0, 0.0)] + [p for seg in reversed(segments) for p in seg]
        expanded += 1
        for direction in (1, -1):
            for beta_target in beta_targets:
                q = s
                segment = []
                for _ in range(7):
                    # Invert the off-axle beta dynamics, so reverse primitives
                    # regulate articulation instead of amplifying it open-loop.
                    beta_rate = direction * 0.65 * (beta_target-q.beta)
                    curvature = (-math.sin(q.beta)-g.trailer_axle*beta_rate) / (g.trailer_axle+g.hitch_offset*math.cos(q.beta))
                    steer = max(-g.max_steer, min(g.max_steer, math.atan(g.wheelbase*curvature)))
                    q = advance(q, direction*0.1, steer, g)
                    if not scene.free(q, g):
                        break
                    segment.append(Step(q, direction, steer))
                if len(segment) != 7:
                    continue
                new_cost = (cost + 0.7*(1.0 if direction > 0 else 1.15) +
                            (2.0 if old_dir and direction != old_dir else 0.0) +
                            0.08*abs(steer-old_steer) + 0.08*abs(q.beta))
                k = key(q, direction)
                if new_cost >= costs.get(k, math.inf):
                    continue
                costs[k] = new_cost
                records.append((q, direction, steer, index, tuple(segment), new_cost))
                heapq.heappush(queue, (new_cost+2.5*heuristic(q), next(serial), len(records)-1))
    raise PlanningError(f'No feasible path within budget ({expanded} expansions); remain stopped')


def gazebo_command(speed, steering):
    """Gazebo Classic ackermann plugin flips target_rot by speed sign internally."""
    return speed, steering * (1.0 if speed >= 0 else -1.0)


class Tracker:
    """Short-horizon sampled control; full state feedback includes trailer yaw.

    Progress is monotonic and never skips a direction cusp. Direction changes
    insert a stop. ROS runtime independently checks observations and clearance.
    """
    def __init__(self, path, geometry=Geometry(), speed=0.35):
        self.path, self.g, self.speed = path, geometry, speed
        self.index = 1
        self.direction = 0

    def command(self, state, scene):
        if len(self.path) <= 1 or self.index >= len(self.path):
            return 0.0, 0.0
        direction = self.path[self.index].direction
        end = self.index
        while end+1 < len(self.path) and self.path[end+1].direction == direction:
            end += 1
        closest = min(range(max(1, self.index-2), min(end, self.index+12)+1),
                      key=lambda i: math.hypot(state.x-self.path[i].state.x,
                                               state.y-self.path[i].state.y))
        self.index = max(self.index, closest)
        endpoint = self.path[end].state
        if (math.hypot(state.x-endpoint.x, state.y-endpoint.y) < 0.16 and
                self.index >= end-2):
            self.index = end+1
            self.direction = 0
            return 0.0, 0.0
        if self.direction != direction:
            self.direction = direction
            return 0.0, 0.0
        target = self.path[min(end, self.index+7)].state
        feed = self.path[self.index].steer
        candidates = sorted(set(max(-self.g.max_steer, min(self.g.max_steer, feed+d))
                                for d in (-0.30, -0.15, -0.06, 0.0, 0.06, 0.15, 0.30)))
        best = None
        travel = min(0.7, max(0.12, math.hypot(state.x-target.x, state.y-target.y)))
        for steer in candidates:
            q = state
            safe = True
            for _ in range(10):
                q = advance(q, direction*travel/10, steer, self.g)
                if not scene.free(q, self.g):
                    safe = False
                    break
            if not safe:
                continue
            score = ((q.x-target.x)**2 + (q.y-target.y)**2 +
                     4.0*wrap(q.yaw-target.yaw)**2 +
                     8.0*wrap(q.beta-target.beta)**2 + 0.015*(steer-feed)**2)
            if best is None or score < best[0]:
                best = score, steer
        if best is None:
            return 0.0, 0.0
        return direction*self.speed, best[1]
