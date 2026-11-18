"""Real image detections and contiguous LaserScan groups for field inspection."""
import math
from pathlib import Path

import cv2
import numpy as np

CLASSES = ('background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle',
           'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor')


class Detector:
    def __init__(self, model_dir):
        model_dir = Path(model_dir)
        self.net = cv2.dnn.readNetFromCaffe(str(model_dir/'deploy.prototxt'),
                                          str(model_dir/'mobilenet_iter_73000.caffemodel'))

    def detect(self, image, confidence=.65):
        self.net.setInput(cv2.dnn.blobFromImage(image, .007843, (300, 300), 127.5,
                                              swapRB=False, crop=False))
        height, width = image.shape[:2]
        result = []
        for row in self.net.forward()[0, 0]:
            label = int(row[1])
            if row[2] < confidence or not 0 < label < len(CLASSES):
                continue
            box = row[3:7] * np.array([width, height, width, height])
            x1, y1, x2, y2 = box.astype(int)
            box = [int(np.clip(x1, 0, width-1)), int(np.clip(y1, 0, height-1)),
                   int(np.clip(x2, 0, width-1)), int(np.clip(y2, 0, height-1))]
            if box[2] > box[0] and box[3] > box[1]:
                result.append({'label': CLASSES[label], 'confidence': float(row[2]), 'box': box})
        return result


def scan_groups(scan, gap=.18, min_points=3):
    """Group adjacent valid beams; preserve missing-beam gaps and scan wrap."""
    groups, current = [], []
    for i, value in enumerate(scan.ranges):
        if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
            if current:
                groups.append(current)
            current = []
            continue
        angle = scan.angle_min + i*scan.angle_increment
        point = [value*math.cos(angle), value*math.sin(angle)]
        if current and math.dist(current[-1], point) > gap:
            groups.append(current)
            current = []
        current.append(point)
    if current:
        groups.append(current)
    full_circle = abs(len(scan.ranges)*abs(scan.angle_increment)-2*math.pi) <= 2*abs(scan.angle_increment)
    ends_valid = (len(scan.ranges) > 0 and all(math.isfinite(v) and scan.range_min <= v <= scan.range_max
                                           for v in (scan.ranges[0], scan.ranges[-1])))
    if full_circle and ends_valid and len(groups)>1 and math.dist(groups[-1][-1], groups[0][0]) <= gap:
        groups[0] = groups.pop()+groups[0]
    return [np.asarray(group) for group in groups if len(group) >= min_points]


def nearest_scan(scans, stamp_ns, max_delta_s=.10):
    """Select timestamp-near data, without claiming geometric correspondence."""
    if not scans or stamp_ns <= 0:
        return None, None
    scan = min(scans, key=lambda m: abs(message_stamp(m)-stamp_ns))
    delta = (message_stamp(scan)-stamp_ns)/1e9
    return (scan, delta) if message_stamp(scan)>0 and abs(delta) <= max_delta_s else (None, delta)


def message_stamp(message):
    return message.header.stamp.sec*10**9 + message.header.stamp.nanosec
