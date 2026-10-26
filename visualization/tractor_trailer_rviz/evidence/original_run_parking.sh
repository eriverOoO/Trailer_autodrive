#!/usr/bin/env bash
set -eo pipefail
final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/mvsim_slam_final"
[[ -f "$final_root/scripts/run.py" ]] || { echo "Final project missing: $final_root" >&2; exit 1; }
mkdir -p "$final_root/runs"
exec 9>"$final_root/runs/.parking.lock"
flock -n 9 || { echo 'An autonomous parking run is already active.' >&2; exit 1; }
source "$final_root/scripts/environment.bash"
# Refuse another simulator; never signal a process we did not start.
/usr/bin/python3 "$final_root/scripts/preflight.py"
run_name="B_gui_$(date +%Y%m%d_%H%M%S)_$$"
run_dir="$final_root/runs/$run_name"
mkdir "$run_dir"
export ROS_LOG_DIR="$run_dir/ros" GLOG_log_dir="$run_dir/glog"
mkdir -p "$ROS_LOG_DIR" "$GLOG_log_dir"
export RESEARCH_BUDGET_FILE="$run_dir/supervisor_budget.json"
/usr/bin/python3 -c 'import json,os,time; open(os.environ["RESEARCH_BUDGET_FILE"],"w").write(json.dumps({"deadline_monotonic":time.monotonic()+3600,"kind":"final operation supervisor only; CDE budgets unchanged"}))'
echo "B autonomous GUI parking | seed 1729 | logs: $run_dir"
# Runner creates the directory itself; precreated environment log dirs are allowed.
exec /usr/bin/python3 "$final_root/scripts/run.py" --name "$run_name" --configuration 2 --mode parking --duration 1300 --seed 1729 --coherent-gyro-pairing --execution-load-profile single_trial --gui
