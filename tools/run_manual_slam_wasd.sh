#!/usr/bin/env bash
# Original ros2_parking data_collection controls (no camera) + observed SLAM.
set -e
TASK_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TASK_WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
source visualization/tractor_trailer_rviz_2d/install/setup.bash
export PYTHONNOUSERSITE=1

# Prefer registered aliases. The fallback identifies the previously verified hub
# sockets after that hub was moved; never infer identity from ttyUSB numbering.
lidar_port="${LIDAR_PORT:-/dev/lidar_g6}"
tractor_port="${TRACTOR_IMU_PORT:-/dev/imu_tractor}"
trailer_port="${TRAILER_IMU_PORT:-/dev/imu_trailer}"
arduino_port="${ARDUINO_PORT:-/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_054433A4937351314608-if00}"
if [[ -z "${LIDAR_PORT:-}" && ! -e "$lidar_port" ]]; then
  lidar_port=/dev/serial/by-path/pci-0000:80:14.0-usb-0:10.1:1.0-port0
fi
if [[ -z "${TRACTOR_IMU_PORT:-}" && ! -e "$tractor_port" ]]; then
  tractor_port=/dev/serial/by-path/pci-0000:80:14.0-usb-0:10.2:1.0-port0
fi
if [[ -z "${TRAILER_IMU_PORT:-}" && ! -e "$trailer_port" ]]; then
  trailer_port=/dev/serial/by-path/pci-0000:80:14.0-usb-0:10.3:1.0-port0
fi
for port in "$lidar_port" "$tractor_port" "$trailer_port" "$arduino_port"; do
  if [[ ! -c "$port" || ! -r "$port" || ! -w "$port" ]]; then
    echo "Serial device unavailable: $port" >&2
    exit 1
  fi
  if fuser "$port" >/dev/null 2>&1; then
    echo "Serial device already in use: $port" >&2
    exit 1
  fi
done
run_directory="log/manual_wasd_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_directory"
echo "Logs: $TASK_WORKSPACE/$run_directory"
slam_pid= gui_pid=
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$gui_pid" ]] || kill -TERM "$gui_pid" 2>/dev/null || true
  [[ -z "$slam_pid" ]] || kill -TERM "$slam_pid" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 launch tractor_trailer_rviz_2d relative_yaw.launch.py \
  lidar_port:="$lidar_port" tractor_imu_port:="$tractor_port" trailer_imu_port:="$trailer_port" \
  > "$run_directory/launch.log" 2>&1 &
slam_pid=$!
/usr/bin/python3 tools/parking_data_control.py --port "$arduino_port" \
  > "$run_directory/keyboard.log" 2>&1 &
gui_pid=$!
wait -n "$slam_pid" "$gui_pid"
