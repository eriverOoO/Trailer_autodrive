#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
command -v uv >/dev/null || { echo 'Install uv before running this setup.' >&2; exit 1; }
if [ ! -x "$TASK_ROOT/.venv-parking-linux/bin/python" ]; then
  uv venv "$TASK_ROOT/.venv-parking-linux" --python /usr/bin/python3
fi
uv pip install --python "$TASK_ROOT/.venv-parking-linux/bin/python" \
  torch==2.6.0 torchvision==0.21.0 --index https://download.pytorch.org/whl/cpu
uv pip install --python "$TASK_ROOT/.venv-parking-linux/bin/python" \
  ultralytics==8.4.172 onnx==1.17.0 opencv-python==4.11.0.86 numpy==2.2.6
if [ ! -x "$TASK_ROOT/.venv-vision-ros/bin/python" ]; then
  uv venv "$TASK_ROOT/.venv-vision-ros" --python /usr/bin/python3 --system-site-packages
fi
uv pip install --python "$TASK_ROOT/.venv-vision-ros/bin/python" numpy==1.24.4 onnxruntime==1.20.1
"$TASK_ROOT/.venv-parking-linux/bin/python" "$TASK_ROOT/tools/export_ride_on_onnx.py"
echo 'Prepared ride-on detector and isolated ROS vision runtime (no device opened).'
