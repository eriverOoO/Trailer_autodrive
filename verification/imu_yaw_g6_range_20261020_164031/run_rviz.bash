set -o pipefail
source /home/venezuelachocolatemilk/mvsim_slam_real/verification/imu_yaw_g6_range_20261020_164031/environment.bash
ros2 run rviz2 rviz2 -d /home/venezuelachocolatemilk/mvsim_slam_real/verification/imu_yaw_g6_range_20261020_164031/rviz_live_12m.rviz --ros-args -r __node:=tractor_trailer_rviz_2d -p use_sim_time:=false 2>&1 | tee /home/venezuelachocolatemilk/mvsim_slam_real/verification/imu_yaw_g6_range_20261020_164031/rviz.log
