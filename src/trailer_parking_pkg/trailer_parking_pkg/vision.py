"""Metric ground projection and camera-only planar visual odometry.

Assumptions: calibrated rigid cameras, level floor, ground features, and a
measured initial pose. YOLO segmentation removes moving/non-ground objects
from odometry. No odometry, hitch-angle or Gazebo state topic is consumed.
"""
import math
import cv2
import numpy as np
from .core import Slot, rectangle, wrap


class GroundCamera:
    def __init__(self, k, distortion, rotation, translation):
        self.k = np.asarray(k, dtype=float).reshape(3, 3)
        self.distortion = np.asarray(distortion, dtype=float)
        self.rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
        self.translation = np.asarray(translation, dtype=float).reshape(3)
        if (not np.isfinite(self.k).all() or self.k[0, 0] <= 0 or self.k[1, 1] <= 0 or
                not np.isfinite(self.rotation).all() or not np.isfinite(self.translation).all() or
                not np.isfinite(self.distortion).all() or self.translation[2] <= 0 or
                not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-5) or
                not np.isclose(np.linalg.det(self.rotation), 1.0)):
            raise ValueError('Invalid camera calibration')

    def project(self, pixels, maximum=22.0):
        pixels = np.asarray(pixels, dtype=float).reshape(-1, 1, 2)
        uv = cv2.undistortPoints(pixels, self.k, self.distortion).reshape(-1, 2)
        rays = np.column_stack((uv, np.ones(len(uv)))) @ self.rotation.T
        z = rays[:, 2]
        scale = np.divide(-self.translation[2], z, out=np.full(len(z), np.nan), where=z < -0.05)
        points = self.translation + rays*scale[:, None]
        valid = np.isfinite(points).all(axis=1) & (scale > 0) & (np.linalg.norm(points[:, :2], axis=1) < maximum)
        return points[:, :2], valid

    def ground_mask(self, shape):
        h, w = shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        xy, valid = self.project(np.column_stack((xx.ravel(), yy.ravel())), maximum=16)
        valid &= np.linalg.norm(xy, axis=1) > 2.8
        return (valid.reshape(h, w)*255).astype(np.uint8)


def to_world(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + [x, y]


class GroundOdometry:
    def __init__(self, camera, initial_pose):
        self.camera, self.pose = camera, tuple(initial_pose)
        self.previous = None
        self.points = None
        self.stamp = None
        self.mask_shape = None
        self.base_mask = None

    def update(self, image, stamp, non_ground_polygons=()):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.mask_shape != gray.shape:
            self.base_mask = self.camera.ground_mask(gray.shape)
            self.mask_shape = gray.shape
        mask = self.base_mask.copy()
        for poly in non_ground_polygons:
            cv2.fillPoly(mask, [np.asarray(poly, dtype=np.int32)], 0)
        # Erode exclusion boundaries to avoid optical flow on car edges.
        mask = cv2.erode(mask, np.ones((9, 9), np.uint8))
        if self.previous is None:
            self.previous, self.stamp = gray, stamp
            self.points = cv2.goodFeaturesToTrack(gray, 500, 0.01, 8, mask=mask)
            return None, 'INITIALIZING'
        dt = stamp-self.stamp
        if dt <= 0 or dt > 1.0 or self.points is None or len(self.points) < 16:
            return None, 'VISUAL_ODOMETRY_LOST: restart at a measured pose'
        points, status, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, self.points, None)
        if points is None or status is None:
            return None, 'NO_FLOW'
        back, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, points, None)
        if back is None:
            return None, 'NO_BACKWARD_FLOW'
        prev = self.points.reshape(-1, 2)
        curr = points.reshape(-1, 2)
        inside = (curr[:, 0] >= 0) & (curr[:, 0] < gray.shape[1]) & (curr[:, 1] >= 0) & (curr[:, 1] < gray.shape[0])
        keep = (status.ravel() != 0) & (back_status.ravel() != 0) & inside
        keep &= np.linalg.norm(back.reshape(-1, 2)-prev, axis=1) < 1.0
        pixels = curr[keep].astype(int)
        indices = np.flatnonzero(keep)
        keep[indices] &= mask[pixels[:, 1], pixels[:, 0]] > 0
        old_xy, old_ok = self.camera.project(prev[keep], maximum=16)
        new_xy, new_ok = self.camera.project(curr[keep], maximum=16)
        good = old_ok & new_ok
        if np.count_nonzero(good) < 16:
            return None, 'TOO_FEW_GROUND_FEATURES'
        matrix, inliers = cv2.estimateAffinePartial2D(new_xy[good], old_xy[good],
            method=cv2.RANSAC, ransacReprojThreshold=0.07, maxIters=1000, confidence=0.99)
        if matrix is None or inliers is None or np.count_nonzero(inliers) < 12 or inliers.mean() < 0.65:
            return None, 'GROUND_FIT_REJECTED'
        scale = np.hypot(matrix[0, 0], matrix[1, 0])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        shift = matrix[:, 2]
        if abs(scale-1) > 0.025 or abs(yaw) > 0.7*dt+0.02 or np.linalg.norm(shift) > 1.2*dt+0.08:
            return None, 'MOTION_OR_SCALE_REJECTED'
        world = to_world([shift], self.pose)[0]
        self.pose = (float(world[0]), float(world[1]), wrap(self.pose[2]+yaw))
        self.previous, self.stamp = gray, stamp
        self.points = cv2.goodFeaturesToTrack(gray, 500, 0.01, 8, mask=mask)
        return self.pose, 'OK'


def slot_from_mask(pixels, camera, pose):
    points, good = camera.project(pixels)
    points = to_world(points[good], pose).astype(np.float32)
    if len(points) < 4:
        return None
    (x, y), (a, b), angle = cv2.minAreaRect(points)
    yaw = math.radians(angle)
    if b > a:
        a, b, yaw = b, a, yaw+math.pi/2
    # Choose the equivalent slot heading closest to the tractor's heading.
    if abs(wrap(yaw-pose[2])) > math.pi/2:
        yaw += math.pi
    if not (2 < a < 25 and 1.5 < b < 6):
        return None
    # Rectangular paint/segmentation only: clipped masks must be rejected upstream.
    area = abs(cv2.contourArea(cv2.convexHull(points)))
    if area/(a*b) < 0.65:
        return None
    return Slot(float(x), float(y), wrap(yaw), float(a), float(b))


def obstacle_from_mask(pixels, camera, pose, length=4.56, width=2.20):
    """Conservative parked-car footprint from the ground-contact mask boundary.

    Uses the lowest mask band, not the bounding-box centre. Hidden body depth
    is reserved away from the viewing camera. This is not general monocular 3D.
    """
    pixels = np.asarray(pixels)
    if len(pixels) < 3:
        return None
    bottom = pixels[pixels[:, 1] >= pixels[:, 1].max()-5]
    points, good = camera.project(bottom)
    points = points[good]
    if len(points) < 1:
        return None
    contact = np.median(points, axis=0)
    ray = contact - camera.translation[:2]
    distance = np.linalg.norm(ray)
    if distance < 0.1:
        return None
    # Both car_front and car_back are faces; reserve a full car behind the face.
    heading = math.atan2(ray[1], ray[0]) + pose[2]
    world = to_world([contact], pose)[0]
    return rectangle(float(world[0]), float(world[1]), heading, length+0.25, 0.25, width+0.40)


def merge_slots(slots):
    """Merge adjacent collinear single-car masks, with strict width/gap gates."""
    result = list(slots)
    for i, a in enumerate(slots):
        for b in slots[i+1:]:
            delta = wrap(a.yaw-b.yaw)
            if min(abs(delta), abs(abs(delta)-math.pi)) > 0.08:
                continue
            c, s = math.cos(a.yaw), math.sin(a.yaw)
            dx, dy = b.x-a.x, b.y-a.y
            u, v = dx*c+dy*s, -dx*s+dy*c
            gap = abs(u)-(a.length+b.length)/2
            if abs(v) > 0.20 or not (-0.3 <= gap <= 0.35):
                continue
            lo, hi = min(-a.length/2, u-b.length/2), max(a.length/2, u+b.length/2)
            mid = (lo+hi)/2
            result.append(Slot(a.x+mid*c, a.y+mid*s, a.yaw, hi-lo, min(a.width, b.width)-abs(v)))
    return result
