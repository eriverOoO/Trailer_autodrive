"""Train/evaluate segmentation; never infer that existing weights cover new views."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--device', default='0')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--validate-only', action='store_true')
    args = parser.parse_args()
    if not args.weights.is_file() or not args.data.is_file():
        parser.error('Supply existing local weights and a labelled dataset YAML')
    import yaml
    spec = yaml.safe_load(args.data.read_text())
    names = spec['names']
    names = list(names.values()) if isinstance(names, dict) else names
    if names != ['parking_space', 'car_front', 'car_back']:
        parser.error('Dataset classes must match parking_space, car_front, car_back in that order')
    from ultralytics import YOLO
    model = YOLO(str(args.weights))
    if model.task != 'segment':
        parser.error('Segmentation weights are required')
    if args.validate_only:
        model.val(data=str(args.data), split='test', device=args.device, imgsz=args.imgsz)
    else:
        model.train(data=str(args.data), epochs=args.epochs, device=args.device, imgsz=args.imgsz,
                    project='runs/parking', name='front_rear_segment', seed=42,
                    flipud=0.0, fliplr=0.5, perspective=0.0)


if __name__ == '__main__':
    main()
