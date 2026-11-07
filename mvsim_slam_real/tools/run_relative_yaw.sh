#!/usr/bin/env bash
# LiDAR-only SLAM + independent relative IMU yaw; no actuation.
set -e
TASK_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$TASK_WORKSPACE/install/setup.bash"
source "$TASK_WORKSPACE/visualization/tractor_trailer_rviz_2d/install/setup.bash"
export PYTHONNOUSERSITE=1
exec ros2 launch tractor_trailer_rviz_2d relative_yaw.launch.py "$@"
