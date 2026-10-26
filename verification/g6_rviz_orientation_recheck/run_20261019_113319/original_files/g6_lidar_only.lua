-- Sensor-origin bench SLAM. No unknown vehicle extrinsic is invented.
include "reference_lidar_only.lua"
options.tracking_frame = "laser_frame"
options.published_frame = "laser_frame"
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 1.0
-- Limit missing-return clearing rays to the current scan range.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
return options
