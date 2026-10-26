-- Sensor-origin bench SLAM. No unknown vehicle extrinsic is invented.
include "reference_lidar_only.lua"
options.tracking_frame = "laser_frame"
options.published_frame = "laser_frame"
TRAJECTORY_BUILDER_2D.min_range = 0.28
-- Retain the B configuration's 12 m SLAM cutoff and scan-matching tuning.
return options
