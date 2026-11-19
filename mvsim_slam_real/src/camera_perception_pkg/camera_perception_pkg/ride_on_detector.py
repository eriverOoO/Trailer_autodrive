"""Single-class YOLO11 ONNX inference without torch in the ROS process."""
import ast
import math
from pathlib import Path

import cv2
import numpy as np

from .parking_objects import Candidate


def letterbox(image, size):
    height, width = image.shape[:2]
    scale = min(size/width, size/height)
    resized = (round(width*scale), round(height*scale))
    dx, dy = (size-resized[0])/2, (size-resized[1])/2
    left, right = round(dx-.1), round(dx+.1)
    top, bottom = round(dy-.1), round(dy+.1)
    image = cv2.resize(image, resized, interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(image, top, bottom, left, right,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    tensor = np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)/255.
    return tensor, scale, (left, top)


class RideOnOnnxDetector:
    def __init__(self, model_path, confidence=.65, nms_iou=.7):
        if not math.isfinite(confidence) or not 0 < confidence < 1:
            raise ValueError('detector_confidence must be between zero and one')
        self.confidence, self.nms_iou = confidence, nms_iou
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise ValueError(f'Ride-on ONNX model missing: {path}; run tools/prepare_ride_on_runtime.sh')
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError('ONNX Runtime unavailable; run tools/prepare_ride_on_runtime.sh '
                               'and set vision_python to .venv-vision-ros/bin/python') from exc
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), sess_options=options,
                                            providers=['CPUExecutionProvider'])
        inputs = self.session.get_inputs()
        if len(inputs) != 1 or inputs[0].type != 'tensor(float)':
            raise ValueError('Expected a single float32 YOLO input')
        shape = inputs[0].shape
        if (len(shape) != 4 or shape[:2] != [1, 3] or type(shape[2]) is not int
                or shape[2] != shape[3] or shape[2] < 32 or shape[2] % 32):
            raise ValueError('Export a static square batch=1 YOLO ONNX model')
        metadata = self.session.get_modelmeta().custom_metadata_map
        try:
            names = ast.literal_eval(metadata.get('names', ''))
        except (ValueError, SyntaxError) as exc:
            raise ValueError('Model class metadata missing') from exc
        if names != {0: 'ride_on_car'}:
            raise ValueError('Model must have exactly one class: ride_on_car')
        if metadata.get('task') != 'detect':
            raise ValueError('Expected a YOLO detection model, not segmentation/classification')
        outputs = self.session.get_outputs()
        if len(outputs) != 1 or outputs[0].shape[:2] != [1, 5]:
            raise ValueError('Expected decoded YOLO output [1, 5, N] without embedded NMS')
        self.input_name, self.size = inputs[0].name, shape[2]

    def detect(self, image):
        tensor, scale, (left, top) = letterbox(image, self.size)
        try:
            output = self.session.run(None, {self.input_name: tensor})[0]
        except Exception as exc:
            raise RuntimeError(f'Ride-on inference failed: {exc}') from exc
        if output.ndim != 3 or output.shape[:2] != (1, 5):
            raise ValueError('Unexpected YOLO output shape')
        rows = output[0].T
        rows = rows[np.isfinite(rows).all(axis=1) & (rows[:, 4] >= self.confidence)
                    & (rows[:, 4] <= 1.) & (rows[:, 2] > 0) & (rows[:, 3] > 0)]
        if not len(rows):
            return []
        boxes = np.column_stack((rows[:, 0]-rows[:, 2]/2, rows[:, 1]-rows[:, 3]/2, rows[:, 2:4]))
        keep = cv2.dnn.NMSBoxes(boxes.tolist(), rows[:, 4].tolist(), self.confidence, self.nms_iou)
        height, width = image.shape[:2]
        candidates = []
        for index in np.asarray(keep, dtype=int).reshape(-1):
            x, y, w, h = boxes[index]
            x1, y1 = np.clip([(x-left)/scale, (y-top)/scale], [0, 0], [width-1, height-1])
            x2, y2 = np.clip([(x+w-left)/scale, (y+h-top)/scale], [0, 0], [width-1, height-1])
            if x2 <= x1 or y2 <= y1:
                continue
            contour = np.rint([[x1, y1], [x2, y1], [x2, y2], [x1, y2]]).astype(np.int32)
            candidates.append(Candidate('ride_on_car', contour, float(rows[index, 4])))
        return candidates
