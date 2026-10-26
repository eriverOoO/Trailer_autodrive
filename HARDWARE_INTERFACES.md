# Hardware interfaces

2026-10-19 update: the connected EBIMU responds `<imu9dofv612b>` and reports
sof2/sog1/soa1/sots1/sor10 at 115200 baud. The new explicit
`orientation_format=quaternion`, `include_acceleration=true`,
`include_timestamp=true` schema accepts 11 fields in wire order
qz,qy,qx,qw,gx,gy,gz,ax,ay,az,timestamp_ms. Quaternion components are reordered
to ROS x,y,z,w and normalized (reject norms more than 0.05 from unity).
Gyro DPS becomes rad/s and gravity-inclusive acceleration in g becomes m/s².
Optional sensor milliseconds are validated but headers still use host receipt
time; this is not hardware clock synchronization. Legacy six/nine-field schemas
retain their defaults. Driver startup transmits no commands.

`ebimu.launch.py` uses the dedicated `ebimu_slam.yaml` profile and checks port
ownership before opening. G6 `use_imu:=true` starts the driver unless
`start_imu:=false`; `imu_mount_file` requires measured extrinsics unless external
TF is selected. Sensor-origin fusion tracks tractor_imu_link and publishes
laser_frame, with measured laser_frame→tractor_imu_link TF. Vehicle fusion tracks
the same IMU and publishes base_footprint. Keep mounting_rpy zero when physical
mounting is expressed in TF, avoiding double rotation.

Hardware validation: 958 real ROS Imu messages at 100Hz, monotonic host stamps,
normalized orientations, acceleration norms 9.65–9.92m/s², and a live Cartographer
subscriber. No LiDAR scans/maps were tested, and physical extrinsics are still
unmeasured. Covariances are initial estimates, not measured noise calibration.
See diagnostics/imu_new_20261019/slam_live_summary.json. Historical notes below
describe the original offline integration.

No hardware was connected/tested. UART APIs exist for future explicit use only.

EBIMU-9DOFV6 rev10 printed p12: `*R,P,Y,gx,gy,gz\r\n`; p20 Euler degrees; p21 gyro DPS. Driver requires exactly these six fields, `<soc1><sof1><sog1><soa0>` plus other optional channels disabled. 115200 8N1, flow control OFF; driver opens independently selected nonempty port only when enable_device=true. Startup writes zero commands. Unknown acceleration has covariance[0]=-1; orientation can be disabled likewise. Host receipt timestamps are used because the six-field stream has no hardware timestamp. mounting_rpy rotates raw vectors into the declared frame and right-multiplies orientation by inverse mounting rotation; physical mount TF is separately configured. Axis/polarity alignment requires bench confirmation. Configuration CLI defaults to OFFLINE PREVIEW, with --send and explicit port required for future use; calibration also requires --stationary-confirmed. No calibration was sent.

LiDAR manufacturer is UNDECIDED. /lidar_raw LaserScan standard interface uses sensor-data QoS. Gateway /scan checks geometry/beam counts/stamps; missing scans are unavailable. Optional rplidar_source uses preserved UART library and explicit port/enable_device only. Binning uses degree→radian and mm→m; acquisition lacks per-ray clock, so time_increment=0 is an acknowledged instantaneous approximation. For a selected production sensor replace upstream driver and keep /scan/laser_frame. Positive increment is required; reflect ordered timestamp scans upstream. Original lidar_processor_node remains available on lidar_raw→lidar_processed but default SLAM does not use its application-specific filtering.

Arduino: original serial_communication_pkg consumes interfaces_pkg/MotionCommand on /topic_control_signal, port parameter, baud 115200. Three int32 fields steering,left_speed,right_speed. Original protocol remains `s{steering}l{left_speed}r{right_speed}\n`. Firmware driving.ino has steering ±7 and signed analogWrite motor speed; external adapter clamps PWM ±255. Its 15-byte buffer supports worst valid command `s-7l-255r-255` (13 chars). New adapter recovers delta=atan(wheelbase*omega/v), maps delta/max_steer to ±7 with configurable sign, uses provisional PWM/(m/s), caps forward/reverse velocities and steering rate. E-stop holds previous steering and zeroes both motors. It rejects rotate-in-place/nonfinite/invalid state/stale IMU; stop conditions cannot invent motion. Preview is /parking/motion_preview. No sender runs in default launch. Hardware gate additionally requires scale/polarity acknowledgment.

Unresolved hardware limitation: firmware has NO receive watchdog, acknowledgment, encoder feedback or independent emergency cutoff. A stopped/crashed host cannot guarantee motor stop. This is recorded; firmware is protected and not changed. Do not interpret software timeout as board-level failsafe. Motor factors, steering sign, physical offsets, initial articulated alignment and gyro bias remain PROVISIONAL/unconfirmed.

Optional fusion schema: explicitly enable raw gravity-inclusive soa1 acceleration, parse nine fields RPY,gyro XYZ,accel XYZ, convert g→9.80665 m/s² and publish measured covariance. Default remains six-field soa0. Optional Lua tracks tractor_imu_link (Cartographer requires acceleration at tracking origin); actual fusion is not hardware validated. Legacy RPLidar bins without an actual return remain NaN/unknown, not evidence of maximum-range free space.
