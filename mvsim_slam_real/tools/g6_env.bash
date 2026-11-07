# Source from bash: source ~/mvsim_slam_real/tools/g6_env.bash
_g6_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [ -f "$_g6_workspace/install/setup.bash" ]; then
    source "$_g6_workspace/install/setup.bash"
fi
export CMAKE_PREFIX_PATH="$_g6_workspace/third_party/ydlidar_sdk_install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
unset _g6_workspace
