# Architecture

Original packages (7) are copied intact, including vision resources and firmware outside packages. New packages: ebimu_driver (UART parser/configuration CLI/two nodes); real_vehicle_bridge (standard scan gateway, optional original RPLidar library source, static TF, articulated estimator, control adapter); real_parking (unchanged final B modules plus native C++ optimizer built locally, independent diagnostic entries and original AutoParking); mvsim_slam_real_bringup (YAML/Lua/launch/RViz).

`map → odom → base_footprint` is published solely by Cartographer. base_footprint is the provisional tractor BODY CENTER. Static children: base_link (same), laser_frame, tractor_imu_link; dynamic child trailer_link is the trailer BODY CENTER inferred from hitch and integrated trailer yaw. Trailer IMU is a static child of trailer_link. local_map is an identity static child of map, retaining the B interface convention.

Planner state [x,y,tractor_yaw,trailer_yaw] uses tractor REAR AXLE, not scan center. Body/axle/hitch offsets are transformed explicitly. Original trailer footprint convention is relative to HITCH; new geometry conversion constructs identical-size rectangular bodies (.99 x .55m; user-measured 2026-10-24) in appropriate frames. Independent positions/distances must satisfy geometric consistency; configuration rejects contradictions.

Maps: Cartographer /map is unchanged. /parking/detection_map preserves observed/unknown evidence, and /tractor/localization/local_map feeds B collision checks. Scan-derived free-space augmentation uses stamped TF at the scan time, configured lidar offset and original .15m no-hit margin; it never erases occupied cells. Unknown/outside remain blocked. Hardware state is invalid until actual fresh SLAM matching, scan and both IMUs exist, with explicit yaw alignment/bias acknowledgment.

AutoParking retains Final B detector, C/D/E scheduler, steering-priority, clearance+shot+resume20, LQR and state transitions. Its Geometry.from_world factory is adapted externally to validated YAML; copied algorithm files remain unchanged. Integrated LQR outputs Twist yaw rate; external adapter recovers steering with atan(wheelbase*omega/v), clamps/rate-limits/quantizes steering, and maps signed speed to provisional PWM. Preview commands are isolated on /parking/motion_preview; hardware commands require all explicit gates. Legacy vision motion planner is never included in new launch.

Development mode launches software nodes without opening ANY device and reports unavailable hardware, never fabricated sensor success. Optional synthetic test fixtures exist only under tests/ and verification/. Original launch_pkg/main.launch.py may drive hardware and is not a development command.

Measured straight geometry: tractor rear edge → hinge = .15m; hinge → trailer front edge = .06m. Body gap = .21m, overall length = 2.19m. Hinge x is -.645m in tractor body-centre coordinates and +.555m in trailer body-centre coordinates. Trailer axle is measured .19m ahead of the body rear, giving body-centre x=-.305m and hinge-to-axle distance .86m. Geometry confirmation remains false.

Tractor rear axle is measured 0.19m ahead of the body rear: x=-.305m relative to body centre. With .55m wheelbase, front steering axle x=+.245m. Rear axle to hinge distance is .34m.
