"""Classical image candidates and calibrated 2D LiDAR projection."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Candidate:
    """An image region to compare with projected scan returns."""

    label: str
    contour: np.ndarray


def detect_candidates(image, roi_top=0.25, cone_hue_min=3, cone_hue_max=28,
                      white_saturation_max=50, white_value_min=195):
    """Find orange tapered shapes and bright, low-saturation body candidates."""
    height, width = image.shape[:2]
    top = int(height * roi_top)
    hsv = cv2.cvtColor(image[top:], cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(hsv, (cone_hue_min, 105, 75), (cone_hue_max, 255, 255))
    white = cv2.inRange(hsv, (0, 0, white_value_min), (179, white_saturation_max, 255))
    kernel = np.ones((3, 3), dtype=np.uint8)
    # Reconnect orange regions across thin horizontal reflective bands.
    band_height = max(3, 2 * (height // 80) + 1)
    cone_kernel = np.ones((band_height, 3), dtype=np.uint8)
    image_area = height * width
    candidates = []
    for label, mask, close_kernel in (('cone', orange, cone_kernel), ('white body', white, kernel)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if min(w, h) < 8 or area < max(30, image_area * 0.00008):
                continue
            if label == 'cone':
                if not 0.25 <= w / h <= 1.5:
                    continue
                shape = mask[y:y + h, x:x + w]
                upper = np.count_nonzero(shape[h // 5:2 * h // 5], axis=1).mean()
                lower = np.count_nonzero(shape[3 * h // 5:4 * h // 5], axis=1).mean()
                if lower <= 1.2 * upper:
                    continue
            elif not (0.75 <= w / h <= 4.5 and 0.35 * w * h <= area <= 0.22 * image_area):
                continue
            candidates.append(Candidate(label, contour + np.array([0, top], dtype=np.int32)))
    return candidates


class ScanProjection:
    """Project scan-frame points into a measured camera optical frame."""

    def __init__(self, data):
        self.image_size = tuple(data['image_size'])
        self.camera_frame = data['camera_frame']
        self.scan_frame = data['scan_frame']
        self.matrix = np.asarray(data['camera_matrix'], dtype=np.float64)
        self.distortion = np.asarray(data['distortion_coefficients'], dtype=np.float64)
        self.rotation = np.asarray(data['lidar_to_camera_rotation'], dtype=np.float64)
        self.translation = np.asarray(data['lidar_to_camera_translation_m'], dtype=np.float64)
        if (len(self.image_size) != 2 or any(type(n) is not int or n <= 0 for n in self.image_size)
                or not all(isinstance(frame, str) and frame.strip()
                           for frame in (self.camera_frame, self.scan_frame))
                or self.camera_frame == self.scan_frame
                or self.matrix.shape != (3, 3) or self.rotation.shape != (3, 3)
                or self.translation.shape != (3,) or self.distortion.ndim != 1
                or self.distortion.size not in (4, 5, 8, 12, 14)
                or not all(np.isfinite(array).all() for array in
                           (self.matrix, self.distortion, self.rotation, self.translation))):
            raise ValueError('Invalid camera-LiDAR calibration dimensions, frames, or values')
        if (self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0
                or not np.allclose(self.matrix[2], (0, 0, 1))
                or not np.allclose(self.rotation @ self.rotation.T, np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(self.rotation), 1.0, atol=1e-4)):
            raise ValueError('Invalid camera matrix or lidar-to-camera rotation')

    def projected_returns(self, scan):
        """Keep each image pixel paired with its measured scan-frame XY point."""
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        angles = scan.angle_min + np.flatnonzero(valid) * scan.angle_increment
        ranges = ranges[valid]
        points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles),
                                  np.zeros_like(ranges)))
        scan_xy = points[:, :2].copy()
        points = points @ self.rotation.T + self.translation
        front = points[:, 2] > 0.0
        points, scan_xy = points[front], scan_xy[front]
        if not points.size:
            return np.empty((0, 2), dtype=np.int32), np.empty((0, 2))
        zero = np.zeros(3)
        pixels, _ = cv2.projectPoints(points, zero, zero, self.matrix, self.distortion)
        pixels = np.rint(pixels.reshape(-1, 2))
        width, height = self.image_size
        inside = np.isfinite(pixels).all(axis=1) & (pixels >= 0).all(axis=1)
        inside &= (pixels[:, 0] < width) & (pixels[:, 1] < height)
        return pixels[inside].astype(np.int32), scan_xy[inside]

    def pixels(self, scan):
        """Return image pixels for the existing overlay interface."""
        return self.projected_returns(scan)[0]


def associate_candidates(candidates, pixels, scan_xy, min_returns=3, cluster_distance=.20):
    """Select a contiguous foreground return cluster inside each contour.

    This is a geometric association hypothesis, not semantic ground truth.
    Keeping metric returns allows downstream map geometry to recheck it.
    """
    associations = []
    for candidate in candidates:
        keep = np.array([cv2.pointPolygonTest(candidate.contour, (float(u), float(v)), False)
                         >= 0 for u, v in pixels], dtype=bool)
        points = scan_xy[keep]
        if len(points) < min_returns:
            continue
        splits = np.flatnonzero(np.linalg.norm(np.diff(points, axis=0), axis=1)
                                > cluster_distance) + 1
        clusters = [group for group in np.split(points, splits) if len(group) >= min_returns]
        if clusters:
            foreground = min(clusters, key=lambda group: float(np.median(np.linalg.norm(group, axis=1))))
            associations.append({'label': candidate.label, 'scan_xy': foreground.tolist()})
    return associations


def draw_diagnostic(image, candidates, pixels, status):
    """Draw scan returns and count their overlap with each image contour."""
    if pixels is not None:
        for u, v in pixels:
            cv2.circle(image, (int(u), int(v)), 2, (255, 255, 0), -1)
    for candidate in candidates:
        x, y, w, h = cv2.boundingRect(candidate.contour)
        label = candidate.label
        if pixels is not None:
            in_box = pixels[(pixels[:, 0] >= x) & (pixels[:, 0] < x + w)
                            & (pixels[:, 1] >= y) & (pixels[:, 1] < y + h)]
            overlap = sum(cv2.pointPolygonTest(candidate.contour, (float(u), float(v)), False) >= 0
                          for u, v in in_box)
            label += f': {overlap} returns'
        color = (0, 140, 255) if candidate.label == 'cone' else (240, 240, 240)
        cv2.drawContours(image, [candidate.contour], 0, color, 2)
        cv2.putText(image, label, (x, max(35, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    cv2.putText(image, status, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (30, 230, 30), 1)
    return image
