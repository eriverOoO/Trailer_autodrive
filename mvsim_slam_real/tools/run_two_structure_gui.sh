#!/usr/bin/env bash
# Open the two-structure parking test window (mode selection, start/stop, bay view).
set -e
TASK_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TASK_WORKSPACE"
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 tools/two_structure_test_gui.py "$@"
