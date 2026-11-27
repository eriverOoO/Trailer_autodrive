"""Extract independent photo cells and write the manually checked YOLO labels."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheel-augmentation', action='store_true')
    options = parser.parse_args()
    dataset = ROOT / ('dataset_wheels' if options.wheel_augmentation else 'dataset')
    sheets = [(ROOT / 'annotations.json', ROOT / 'sheets')]
    if options.wheel_augmentation:
        added = ROOT / 'wheel_augmentation'
        sheets.append((added / 'annotations.json', added))
    for annotations, image_dir in sheets:
        extract_sheets(annotations, image_dir, dataset)
    (dataset / 'data.yaml').write_text('train: images/train\nval: images/val\nnames:\n  0: ride_on_car\n')
    for split in ('train', 'val'):
        labels = list((dataset / 'labels' / split).glob('*.txt'))
        print(split, f'{len(labels)} frames, {sum(bool(p.stat().st_size) for p in labels)} positive')


def extract_sheets(annotations, image_dir, dataset):
    spec = json.loads(annotations.read_text(encoding='utf-8'))
    for name, sheet in spec.items():
        pixels = (image_dir / f'{name}.png').read_bytes()
        image = cv2.imdecode(np.frombuffer(pixels, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'Invalid image sheet: {name}')
        height, width = image.shape[:2]
        xs = [round(value * width) for value in sheet.get('xs', (0., 1/3, 2/3, 1.))]
        ys = [round(value * height) for value in sheet.get('ys', (0., 1/3, 2/3, 1.))]
        columns = len(xs) - 1
        assert len(sheet['boxes']) == columns * (len(ys) - 1), name
        images = dataset / 'images' / sheet['split']
        labels = dataset / 'labels' / sheet['split']
        images.mkdir(parents=True, exist_ok=True)
        labels.mkdir(parents=True, exist_ok=True)
        for index, box in enumerate(sheet['boxes']):
            if box == 'skip':
                continue
            row, col = divmod(index, columns)
            # Remove the narrow dividers generated around the individual photos.
            tile = image[ys[row] + 4:ys[row + 1] - 4, xs[col] + 4:xs[col + 1] - 4]
            stem = f'{name}_{index + 1:02d}'
            ok, encoded = cv2.imencode('.jpg', tile)
            if not ok:
                raise RuntimeError(f'Could not save {stem}')
            (images / f'{stem}.jpg').write_bytes(encoded.tobytes())
            target = labels / f'{stem}.txt'
            if box is None:
                target.write_text('')
                continue
            left, top, right, bottom = (value / 100 for value in box)
            assert 0 <= left < right <= 1 and 0 <= top < bottom <= 1, stem
            values = ((left + right) / 2, (top + bottom) / 2, right - left, bottom - top)
            target.write_text('0 ' + ' '.join(f'{value:.4f}' for value in values) + '\n')


if __name__ == '__main__':
    main()
