set -o pipefail
source /home/venezuelachocolatemilk/mvsim_slam_real/verification/real_g6_tractor_imu_20261020_153135/environment.bash
ros2 run rviz2 rviz2 -d /home/venezuelachocolatemilk/mvsim_slam_real/verification/real_g6_tractor_imu_20261020_153135/rviz_map_origin_view.rviz --ros-args -r __node:=tractor_trailer_rviz_2d -p use_sim_time:=false 2>&1 | tee /home/venezuelachocolatemilk/mvsim_slam_real/verification/real_g6_tractor_imu_20261020_153135/rviz_map_view.log
