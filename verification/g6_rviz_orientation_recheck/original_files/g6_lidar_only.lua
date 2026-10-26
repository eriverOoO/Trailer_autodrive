-- Sensor-origin bench SLAM. No unknown vehicle extrinsic is invented.
include "reference_lidar_only.lua"
options.tracking_frame = "laser_frame"
options.published_frame = "laser_frame"
TRAJECTORY_BUILDER_2D.min_range = 0.28
TRAJECTORY_BUILDER_2D.max_range = 15.0
-- Retain the original missing-data ray length.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0
return options
