"""Extract independent photo cells and write the manually checked YOLO labels."""
import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent


def main():
    spec = json.loads((ROOT / 'annotations.json').read_text(encoding='utf-8'))
    for name, sheet in spec.items():
        image = cv2.imread(str(ROOT / 'sheets' / f'{name}.png'))
        if image is None:
            raise FileNotFoundError(name)
        height, width = image.shape[:2]
        xs = [round(value * width) for value in sheet.get('xs', (0., 1/3, 2/3, 1.))]
        ys = [round(value * height) for value in sheet.get('ys', (0., 1/3, 2/3, 1.))]
        columns = len(xs) - 1
        assert len(sheet['boxes']) == columns * (len(ys) - 1), name
        images = ROOT / 'dataset' / 'images' / sheet['split']
        labels = ROOT / 'dataset' / 'labels' / sheet['split']
        images.mkdir(parents=True, exist_ok=True)
        labels.mkdir(parents=True, exist_ok=True)
        for index, box in enumerate(sheet['boxes']):
            if box == 'skip':
                continue
            row, col = divmod(index, columns)
            # Remove the narrow dividers generated around the individual photos.
            tile = image[ys[row] + 4:ys[row + 1] - 4, xs[col] + 4:xs[col + 1] - 4]
            stem = f'{name}_{index + 1:02d}'
            if not cv2.imwrite(str(images / f'{stem}.jpg'), tile):
                raise RuntimeError(f'Could not save {stem}')
            target = labels / f'{stem}.txt'
            if box is None:
                target.write_text('')
                continue
            left, top, right, bottom = (value / 100 for value in box)
            assert 0 <= left < right <= 1 and 0 <= top < bottom <= 1, stem
            values = ((left + right) / 2, (top + bottom) / 2, right - left, bottom - top)
            target.write_text('0 ' + ' '.join(f'{value:.4f}' for value in values) + '\n')
    # Resolve the dataset relative to this YAML on every run.
    (ROOT / 'dataset' / 'data.yaml').write_text(
        'train: images/train\nval: images/val\nnames:\n  0: ride_on_car\n')
    for split in ('train', 'val'):
        labels = list((ROOT / 'dataset' / 'labels' / split).glob('*.txt'))
        print(split, f'{len(labels)} frames, {sum(bool(p.stat().st_size) for p in labels)} positive')


if __name__ == '__main__':
    main()
