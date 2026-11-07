#!/usr/bin/env bash
# Record a real parking run. Start this BEFORE real_parking.launch.py so the
# emergency-stop moment is captured. Output: log/bag_<timestamp>/
set -e
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$here/g6_env.bash"
mkdir -p "$here/../log"
cd "$here/../log"
exec ros2 bag record -o "bag_$(date +%Y%m%d_%H%M%S)" \
  --qos-profile-overrides-path "$here/parking_bag_qos.yaml" \
  /scan /lidar_raw /map /tractor/localization/local_map /tractor/localization/status \
  /parking/status /parking/detection_map /tractor/chassis_polygon /trailer/chassis_polygon \
  /topic_control_signal /parking/request_cmd_vel /tractor/imu/data /trailer/imu/data \
  /tf /tf_static "$@"
