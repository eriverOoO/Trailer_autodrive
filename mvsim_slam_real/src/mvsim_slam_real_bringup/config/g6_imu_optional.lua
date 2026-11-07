-- Vehicle SLAM: measured LiDAR and IMU mounts are required.
include "g6_vehicle.lua"
options.tracking_frame = "tractor_imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true
return options
