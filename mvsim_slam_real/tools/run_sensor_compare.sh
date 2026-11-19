#!/usr/bin/env bash
set -e
TASK_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_VISION_PYTHON="$TASK_WORKSPACE/../.venv-vision-ros/bin/python"
if [ ! -x "$TASK_VISION_PYTHON" ]; then
  echo 'Run bash tools/prepare_ride_on_runtime.sh first.' >&2
  exit 1
fi
cd "$TASK_WORKSPACE"
source tools/g6_env.bash
exec "$TASK_VISION_PYTHON" tools/sensor_compare.py "$@"
