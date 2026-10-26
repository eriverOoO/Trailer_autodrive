set -o pipefail
source /home/venezuelachocolatemilk/mvsim_slam_real/verification/real_g6_tractor_imu_20261020_153135/environment.bash
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py lidar_port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:7:1.0-port0 frame_mode:=sensor use_imu:=false start_imu:=false enable_rviz:=false 2>&1 | tee /home/venezuelachocolatemilk/mvsim_slam_real/verification/real_g6_tractor_imu_20261020_153135/g6_slam.log
