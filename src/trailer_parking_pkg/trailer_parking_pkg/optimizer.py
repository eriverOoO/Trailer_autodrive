"""Continuous shooting warm start; every result is independently collision checked."""
import math
import logging
import time
import numpy as np
from scipy.optimize import least_squares
from .core import State, Step, advance, goal_reached, wrap


def shooting_plan(start, slot, scene, g, timeout=25.0, target_state=None):
    goal = target_state or slot.goal(g)
    deadline = time.monotonic()+timeout
    obstacle_data = []
    for poly in scene.obstacles:
        p = np.asarray(poly)
        axes = []
        for edge in np.roll(p, -1, axis=0)-p:
            n = np.array([-edge[1], edge[0]])
            if np.linalg.norm(n) > 1e-6:
                axes.append(n/np.linalg.norm(n))
        obstacle_data.append((p, axes))

    def rollout(values, signs):
        q = start
        out = []
        n = len(signs)
        for length, steer, direction in zip(values[:n], values[n:], signs):
            for _ in range(12):
                q = advance(q, direction*length/12, steer, g)
                out.append((q.x, q.y, q.yaw, q.beta))
        return np.asarray(out)

    def residual(values, signs):
        if time.monotonic() > deadline:
            raise TimeoutError
        states = rollout(values, signs)
        n = len(signs)
        x, y, yaw, beta = states.T
        trailer_yaw = yaw+beta
        tx = x-g.hitch_offset*np.cos(yaw)-g.trailer_axle*np.cos(trailer_yaw)
        ty = y-g.hitch_offset*np.sin(yaw)-g.trailer_axle*np.sin(trailer_yaw)
        errors = [np.array([(x[-1]-goal.x)*40, (y[-1]-goal.y)*40,
                            wrap(yaw[-1]-goal.yaw)*80, wrap(beta[-1]-goal.beta)*80]),
                  np.maximum(abs(beta)-g.max_beta+0.03, 0)*30,
                  values[:n]*0.005, np.diff(values[n:])*0.005]
        for px, py, angle in ((x, y, yaw), (tx, ty, trailer_yaw)):
            c, s = np.cos(angle), np.sin(angle)
            ux, uy = np.array([g.front+scene.margin, -g.rear-scene.margin,
                               -g.rear-scene.margin, g.front+scene.margin]), np.array([1, 1, -1, -1])*(g.width/2+scene.margin)
            corners = np.stack((px[:, None]+c[:, None]*ux-s[:, None]*uy,
                                py[:, None]+s[:, None]*ux+c[:, None]*uy), axis=2)
            for obstacle, axes in obstacle_data:
                separations = []
                for axis in axes:
                    pa, pb = corners @ axis, obstacle @ axis
                    separations.append(np.maximum(pa.min(axis=1)-pb.max(), pb.min()-pa.max(axis=1)))
                for ax, ay in ((c, s), (-s, c)):
                    pa = corners[:, :, 0]*ax[:, None]+corners[:, :, 1]*ay[:, None]
                    pb = obstacle[:, 0]*ax[:, None]+obstacle[:, 1]*ay[:, None]
                    separations.append(np.maximum(pa.min(axis=1)-pb.max(axis=1), pb.min(axis=1)-pa.max(axis=1)))
                distance = np.max(separations, axis=0)
                errors.append(np.maximum(0.03-distance, 0)*40)
            xmin, xmax, ymin, ymax = scene.bounds
            errors.extend([np.maximum(xmin-corners[:, :, 0], 0).ravel()*40,
                           np.maximum(corners[:, :, 0]-xmax, 0).ravel()*40,
                           np.maximum(ymin-corners[:, :, 1], 0).ravel()*40,
                           np.maximum(corners[:, :, 1]-ymax, 0).ravel()*40])
        return np.concatenate(errors)

    patterns = [(1, -1, 1, -1, 1, 1, 1, 1), (-1, 1, -1, 1, 1, 1, 1, 1),
                (1, 1, -1, -1, -1, -1, 1, 1),
                (-1, -1, -1, -1, 1, 1, -1, -1),
                (1, 1, -1, -1, -1, -1, -1, -1), (-1,)*8,
                (1, 1, 1, 1, -1, -1, -1, -1), (1,)*8,
                (-1, -1, -1, -1, 1, 1, 1, 1)]
    longitudinal = (goal.x-start.x)*math.cos(start.yaw)+(goal.y-start.y)*math.sin(start.yaw)
    if longitudinal > 0:
        patterns.insert(0, (1,)*8)
    if target_state is not None:
        patterns.insert(0, (-1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, 1))
    else:
        patterns.insert(0, (-1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1))
    for signs in patterns:
        n = len(signs)
        lengths = np.full(n, max(1, math.hypot(start.x-goal.x, start.y-goal.y)/n))
        initial = np.r_[lengths, np.zeros(n)]
        if n == 12:
            if target_state is not None:
                initial[:6] = 0.7
                initial[n:n+6] = [0.55, -0.55, 0.55, -0.55, 0.55, -0.55]
            else:
                initial[:6] = 1.6
                initial[6:12] = 0.7
                initial[n:] = [-0.5, 0.4, 0.4, 0.0, -0.35, -0.5,
                               -0.55, 0.55, -0.55, 0.55, -0.55, 0.55]
        try:
            fit = least_squares(residual, initial, args=(signs,),
                bounds=(np.r_[np.full(n, 0.02), np.full(n, -g.max_steer)],
                        np.r_[np.full(n, 10.0), np.full(n, g.max_steer)]),
                max_nfev=200, ftol=1e-7, xtol=1e-7, gtol=1e-7)
        except TimeoutError:
            return None
        logging.getLogger(__name__).debug('pattern=%s cost=%s terminal=%s controls=%s', signs, fit.cost, rollout(fit.x, signs)[-1], fit.x)
        q, path = start, [Step(start, 0, 0)]
        good = True
        for length, steer, direction in zip(fit.x[:n], fit.x[n:], signs):
            n = max(1, math.ceil(length/0.08))
            for _ in range(n):
                q = advance(q, direction*length/n, float(steer), g)
                if not scene.free(q, g):
                    good = False
                    break
                path.append(Step(q, direction, float(steer)))
            if not good:
                break
        reached = (math.hypot(q.x-goal.x, q.y-goal.y) < 0.10 and
                   abs(wrap(q.yaw-goal.yaw)) < 0.025 and abs(wrap(q.beta-goal.beta)) < 0.025)
        if good and (reached if target_state else goal_reached(q, slot, g)):
            return path
    return None
