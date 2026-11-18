"""Choose the weights at an operating threshold on the synthetic validation set."""
import argparse
import json
from pathlib import Path
import shutil

from ultralytics import YOLO

from evaluate_real import overlap

ROOT = Path(__file__).resolve().parent


def boxes_for(result):
    label = ROOT / 'dataset' / 'labels' / 'val' / (Path(result.path).stem + '.txt')
    fields = label.read_text().split()
    if not fields:
        return None
    h, w = result.orig_shape
    x, y, bw, bh = map(float, fields[1:])
    return ((x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h)


def score(weights, images, confidence):
    model = YOLO(str(weights))
    hits = misses = unmatched = 0
    ious = []
    for result in model.predict(images, device='0', imgsz=416, conf=confidence,
                                stream=True, verbose=False):
        expected = boxes_for(result)
        detected = result.boxes.xyxy.tolist()
        iou = max((overlap(expected, box) for box in detected), default=0.) if expected else 0.
        hit = iou >= .5
        hits += hit
        misses += expected is not None and not hit
        unmatched += len(detected) - hit
        if expected is not None:
            ious.append(iou)
    denom = 2 * hits + misses + unmatched
    return {'hits': hits, 'misses': misses, 'unmatched': unmatched, 'f1': 2 * hits / denom,
            'mean_best_iou': sum(ious) / len(ious)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('checkpoints', type=Path)
    parser.add_argument('--confidence', type=float, default=.25)
    args = parser.parse_args()
    images = [str(path) for path in sorted((ROOT / 'dataset' / 'images' / 'val').glob('*.jpg'))]
    scores = {}
    for path in sorted(args.checkpoints.glob('*.pt')):
        scores[path.name] = score(path, images, args.confidence)
    selected = max(scores, key=lambda name: (scores[name]['f1'], scores[name]['mean_best_iou']))
    shutil.copyfile(args.checkpoints / selected, ROOT / 'ride_on_yolo11n.pt')
    path = ROOT / 'synthetic_selection.json'
    path.write_text(json.dumps({'confidence': args.confidence, 'selected': selected, 'scores': scores},
                               indent=2))
    print(f'{selected}: {scores[selected]}')


if __name__ == '__main__':
    main()
