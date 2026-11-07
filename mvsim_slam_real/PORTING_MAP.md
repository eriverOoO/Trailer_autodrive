# Porting Map

## Verified active Final B call path
`~/mvsim_slam/run_parking.sh` → `mvsim_slam_final/scripts/run.py` → `src/parking_system/scripts/parking_node.py` (simulation input bridge) and `auto_parking_entry.py` → vendor `AutoParking`.
Environment: COMPARISON_POLICY=B, HYBRID_V2_PROFILE=clearance+shot+resume20. Optimizer variants install native clearance, shot, resumable search; steering_priority.install and candidate_scheduler_online.install are active. These exact modules will be copied; no tutorial substitute.
Controller returns signed m/s and steering radians; AutoParking.send converts steering to yaw rate: omega=v*tan(delta)/wheelbase. Planner state is tractor REAR AXLE x,y,tractor yaw,trailer yaw. Trailer polygons are expressed relative to hitch.

## Original ROS packages and installed entry points
- camera_perception_pkg: src_example/camera_perception_pkg; dependencies: ament_copyright, ament_flake8, ament_pep257, python3-pytest, cv_bridge, interfaces_pkg, rclpy, std_msgs, std_srvs, sensor_msgs
  'image_publisher_node = camera_perception_pkg.image_publisher_node:main',
  'car_detector_node = camera_perception_pkg.car_detector_node:main',
  'yolov8_node = camera_perception_pkg.yolov8_node:main',
  'traffic_light_detector_node = camera_perception_pkg.traffic_light_detector_node:main',
  'parking_space_info_extractor_node = camera_perception_pkg.parking_space_info_extractor_node:main',
  'parking_space_detector_node = camera_perception_pkg.parking_space_detector_node:main',
- debug_pkg: src_example/debug_pkg; dependencies: ament_copyright, ament_flake8, ament_pep257, python3-pytest, cv_bridge, interfaces_pkg, rclpy, sensor_msgs, std_msgs
  'path_visualizer_node = debug_pkg.path_visualizer_node:main',
  'yolov8_visualizer_node = debug_pkg.yolov8_visualizer_node:main',
- decision_making_pkg: src_example/decision_making_pkg; dependencies: ament_copyright, ament_flake8, ament_pep257, python3-pytest, cv_bridge, interfaces_pkg, rclpy, std_msgs, std_srvs, sensor_msgs, visualization_msgs
  'motion_planner_node = decision_making_pkg.motion_planner_node:main',
  'parking_space_path_planner_node = decision_making_pkg.parking_space_path_planner_node:main',
- interfaces_pkg: src_example/interfaces_pkg; dependencies: ament_cmake, rosidl_default_generators, rosidl_default_runtime, ament_lint_auto, ament_lint_common, std_msgs, geometry_msgs
- launch_pkg: src_example/launch_pkg; dependencies: ament_cmake, ament_index_python, launch, launch_ros, camera_perception_pkg, decision_making_pkg, lidar_perception_pkg, serial_communication_pkg, debug_pkg, rviz2, ament_lint_auto, ament_lint_common
- lidar_perception_pkg: src_example/lidar_perception_pkg; dependencies: ament_copyright, ament_flake8, ament_pep257, python3-pytest, rclpy, sensor_msgs, std_msgs
  'lidar_publisher_node = lidar_perception_pkg.lidar_publisher_node:main',
  'lidar_processor_node = lidar_perception_pkg.lidar_processor_node:main',
  'lidar_obstacle_detector_node = lidar_perception_pkg.lidar_obstacle_detector_node:main',
- serial_communication_pkg: src_example/serial_communication_pkg; dependencies: ament_copyright, ament_flake8, ament_pep257, python3-pytest, interfaces_pkg, rclpy
  'serial_sender_node = serial_communication_pkg.serial_sender_node:main',

## Protected drive layer
Entire `src_example/serial_communication_pkg` (including metadata/resources/tests); interfaces_pkg message definitions; all four `src_example/control/**/*.ino`.
Topic: `/topic_control_signal`; message interfaces_pkg/MotionCommand: int32 steering,left_speed,right_speed. Serial: parameter port default /dev/ttyACM0, 115200; command `s{steering}l{left_speed}r{right_speed}\n`. Firmware steering -7..7, motor signed PWM limited externally to -255..255; 15-byte input buffer (14 chars plus NUL). Worst valid negative command is 13 chars before newline. Board does not implement receive watchdog. No protected source or copy may be edited.

## Original sensors and vision
LiDAR package embeds RPLidar UART protocol; original lidar_raw → lidar_processed. Original publisher hardcodes port/scan assumptions; new optional RPLidar source and standard scan gateway will live outside it. Manufacturer remains undecided.
Camera and YOLO packages retained in full; main.launch.py also launches vision motion planning and serial control and therefore is NOT included by new bringup. New vision launch will run perception only. YOLO engine weights/external dependencies require later provisioning. Original lidar_obstacle_detector_node setup entry points to a missing module; preserved as an existing defect.

## Reuse vs new implementation
Reuse byte-identical vendor detector/model/clearance/controller/AutoParking and optimizer sources, including raw reject capture. Keep existing detection gates, 30/35 degree articulation, hard clearance .18m and controller behavior. Geometry loading is externally adapted from YAML; no simulation World used at runtime.
Replace simulator-only parking_node.py bridge (fixed 1081 beams, base_scan, epoch <10s gyro bias calibration, unique entire /tf publisher, GT-derived chassis polygons) with real state/TF/map bridge. Use original gyro integration approach with supplied bias and explicit initial-articulation acknowledgment; no automatic sensor calibration.

## Data flow
LaserScan source → `/lidar_raw` → standard gateway `/scan` → Cartographer `/map`, map→odom→base_footprint. State estimator transforms tracking center to rear axle, publishes `/parking/state`, maps and chassis polygons expected by original AutoParking.
Two independent EBIMU nodes: `/tractor/imu/data`, `/trailer/imu/data`; each mounting rotation applied to gyro/orientation. Trailer gyro integrated against initial Cartographer heading and acknowledged hitch angle. Tractor IMU monitored and optional orientation diagnostics; Cartographer remains LiDAR-only by default.
AutoParking original detector → B Hybrid A*/CDE → original LQR → `/parking/request_cmd_vel` → NEW adapter (yaw rate to steering radians to steps; m/s to provisional PWM) → original MotionCommand → preserved sender → original firmware.
Static TF sensor offsets; trailer TF is dynamic from articulated state. No simulator, /clock or ground truth inputs.

## Manufacturer evidence
Local PDFs read using pdftotext: ~/Downloads/EBIMU-9DOFV6_rev10.pdf (printed pages 12,20,21: *R,P,Y,gx,gy,gz CR LF; gyro DPS), EBIMU-9DOF_시작하기_rev12.pdf, EBIMU-9DOFV6_상황별_설정.pdf. Extracted copies in references/. UART settings/config commands are from actual manual and user instructions.
