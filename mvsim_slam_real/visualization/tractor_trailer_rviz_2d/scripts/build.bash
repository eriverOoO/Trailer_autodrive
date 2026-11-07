#!/usr/bin/env bash
set -eo pipefail
package_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
cd "$package_root"
colcon --log-base "$package_root/log" build \
  --base-paths "$package_root" --build-base "$package_root/build" \
  --install-base "$package_root/install" \
  --packages-select tractor_trailer_rviz_2d --symlink-install
