-- Requires measured base_footprint -> laser_frame extrinsics.
include "g6_lidar_only.lua"
options.tracking_frame = "base_footprint"
options.published_frame = "base_footprint"
return options
