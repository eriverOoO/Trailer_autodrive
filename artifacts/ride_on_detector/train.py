"""Fine-tune the single-class baseline without touching the ROS configuration."""
import argparse
from pathlib import Path

import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--epochs', type=int, default=65)
    parser.add_argument('--device', default='0')
    parser.add_argument('--name')
    parser.add_argument('--wheel-augmentation', action='store_true')
    options = parser.parse_args()
    wheels = options.wheel_augmentation
    if options.device == 'cpu':
        torch.set_num_threads(min(6, torch.get_num_threads()))
    weights = ROOT / ('ride_on_yolo11n.pt' if wheels else 'yolo11n.pt')
    dataset = ROOT / ('dataset_wheels' if wheels else 'dataset') / 'data.yaml'
    name = options.name or ('yolo11n_ride_on_ft_wheels' if wheels else 'yolo11n_ride_on_ft')
    YOLO(str(weights)).train(
        data=str(dataset), device=options.device, epochs=options.epochs,
        imgsz=416, batch=12, workers=0, patience=0, seed=42, amp=False, cache='ram',
        optimizer='AdamW', lr0=.0002 if wheels else .0005, lrf=.1, warmup_epochs=1., freeze=10,
        project=str(ROOT / 'runs'), name=name, mosaic=0., save_period=10,
        fliplr=.5, degrees=2., translate=.05, scale=.16, hsv_h=.008, hsv_s=.22, hsv_v=.30,
    )


if __name__ == '__main__':
    main()
