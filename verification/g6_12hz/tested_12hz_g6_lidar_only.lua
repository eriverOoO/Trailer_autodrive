-- Sensor-origin bench SLAM. No unknown vehicle extrinsic is invented.
include "reference_lidar_only.lua"
options.tracking_frame = "laser_frame"
options.published_frame = "laser_frame"
TRAJECTORY_BUILDER_2D.min_range = 0.28
TRAJECTORY_BUILDER_2D.max_range = 15.0
-- Retain the B scan-matching tuning and 12 m missing-data ray length.
return options
