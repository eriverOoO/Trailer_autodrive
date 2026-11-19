"""Export the verified single-class detector for a torch-free ROS runtime."""
import argparse
import hashlib
import json
from pathlib import Path

import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, default=ROOT/'artifacts/ride_on_detector/ride_on_yolo11n.pt')
    args = parser.parse_args()
    if not args.weights.is_file() or args.weights.stat().st_size < 10000:
        raise ValueError('Missing model or unresolved Git LFS pointer; run git lfs pull first')
    torch.set_num_threads(2)
    model = YOLO(str(args.weights))
    if model.task != 'detect' or model.names != {0: 'ride_on_car'}:
        raise ValueError('Expected the one-class ride_on_car detection checkpoint')
    output = Path(model.export(format='onnx', imgsz=416, batch=1, opset=12,
                               simplify=False, dynamic=False, nms=False, device='cpu'))
    metadata = {'source': str(args.weights), 'source_sha256': hashlib.sha256(args.weights.read_bytes()).hexdigest(),
                'onnx_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
                'input_shape': [1, 3, 416, 416], 'output_shape': [1, 5, 3549],
                'class': 'ride_on_car', 'opset': 12, 'nms': False}
    output.with_suffix('.export.json').write_text(json.dumps(metadata, indent=2))
    print(output)


if __name__ == '__main__':
    main()
