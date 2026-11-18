"""Compare the learned detector with COCO car on a hand-labelled C920 sample."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
CAPTURE = ROOT.parents[1] / 'mvsim_slam_real/artifacts/sensor_compare_push/20261118_155446'
# Visible body only. The truck-mounted camera crosses the near side of the shell.
REAL_BOXES = {
    0: (201, 21, 640, 335),
    2: (155, 23, 640, 332),
    5: (77, 23, 640, 341),
    12: (79, 24, 640, 344),
    17: (79, 26, 640, 342),
    18: (77, 156, 229, 329),
    19: None,
    20: (78, 23, 640, 352),
    26: (64, 25, 640, 350),
    35: (69, 27, 640, 345),
    38: (79, 24, 640, 352),
    42: (94, 30, 640, 351),
}


def overlap(one, two):
    x = max(0., min(one[2], two[2]) - max(one[0], two[0]))
    y = max(0., min(one[3], two[3]) - max(one[1], two[1]))
    intersection = x * y
    union = (one[2] - one[0]) * (one[3] - one[1]) + (two[2] - two[0]) * (two[3] - two[1]) - intersection
    return intersection / union if union else 0.


def rectangle(img, box, color):
    if box is not None:
        x1, y1, x2, y2 = (int(value) for value in box)
        cv2.rectangle(img, (x1, y1), (min(x2, img.shape[1] - 1), y2), color, 2)


def evaluate(weights, label, output, confidence):
    model = YOLO(str(weights))
    class_id = next(i for i, name in model.names.items() if name == label)
    files = [CAPTURE / f'image_{index:04d}.jpg' for index in REAL_BOXES]
    metrics = {}
    tiles = []
    results = model.predict([str(file) for file in files], device='cpu', conf=confidence,
                            imgsz=416, classes=[class_id], verbose=False)
    for (index, actual), result in zip(REAL_BOXES.items(), results, strict=True):
        predictions = result.boxes.xyxy.tolist()
        best = max((overlap(actual, box) for box in predictions), default=0.) if actual else 0.
        hit = best >= .5
        metrics[str(index)] = {
            'ground_truth': actual, 'best_iou': round(best, 4), 'hit': hit,
            'predictions': len(predictions),
            'top_confidence': round(max(result.boxes.conf.tolist(), default=0.), 4),
        }
        image = result.orig_img.copy()
        rectangle(image, actual, (78, 190, 53))
        for box in predictions:
            rectangle(image, box, (255, 180, 43))
        cv2.putText(image, f'{index:04d}  IoU={best:.2f}  boxes={len(predictions)}',
                    (15, 463), cv2.FONT_HERSHEY_SIMPLEX, .65, (50, 220, 255), 2)
        tiles.append(cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA))
    sheet = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)])
    if not cv2.imwrite(str(ROOT / f'{output}_real_review.jpg'), sheet):
        raise RuntimeError(f'Could not write {output} image')
    return {'positives': sum(box is not None for box in REAL_BOXES.values()),
            'hits_at_iou_0_5': sum(row['hit'] for row in metrics.values()),
            'unmatched_predictions': sum(row['predictions'] - int(row['hit']) for row in metrics.values()),
            'frames': metrics}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('weights', type=Path)
    parser.add_argument('--confidence', type=float, default=.25)
    args = parser.parse_args()
    results = {'confidence': args.confidence,
               'pretrained': evaluate(ROOT / 'yolo11n.pt', 'car', 'pretrained', args.confidence),
               'fine_tuned': evaluate(args.weights, 'ride_on_car', 'fine_tuned', args.confidence)}
    (ROOT / 'real_evaluation.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    print({name: {key: value for key, value in run.items() if key != 'frames'}
           for name, run in results.items() if isinstance(run, dict)})


if __name__ == '__main__':
    main()
