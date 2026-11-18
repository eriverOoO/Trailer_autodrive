#!/usr/bin/env bash
set -euo pipefail
TASK_MODEL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/models/mobilenet_ssd"
TASK_MODEL_BASE=https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/bb17b6c3eef36d80be441ae8e5339be66e8e3b7a
mkdir -p "$TASK_MODEL_DIR"
curl -fL --retry 2 -o "$TASK_MODEL_DIR/deploy.prototxt" "$TASK_MODEL_BASE/deploy.prototxt"
curl -fL --retry 2 -o "$TASK_MODEL_DIR/mobilenet_iter_73000.caffemodel" "$TASK_MODEL_BASE/mobilenet_iter_73000.caffemodel"
curl -fL --retry 2 -o "$TASK_MODEL_DIR/LICENSE" "$TASK_MODEL_BASE/LICENSE"
cd "$TASK_MODEL_DIR"
sha256sum --check <<'HASHES'
2d180f723b3109e21f8287f6b3c691390d07b60eed998327cd3259ffa0e50608  deploy.prototxt
52eed8be80522c152a17fb56740de705b79881bde1a167e0e747310523685fc7  mobilenet_iter_73000.caffemodel
HASHES
