-- LiDAR-origin SLAM with tracking at the actual IMU origin.
include "g6_lidar_only.lua"
options.tracking_frame = "tractor_imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true
return options
