# mvsim_slam_real

Ubuntu 22.04 / ROS 2 Humble용 독립 트랙터–트레일러 자율주차 워크스페이스입니다. 기존 최종 B 탐지·Hybrid A*·steering-priority·C/D/E·LQR·주차 상태 머신과 Arduino 계층을 보존합니다. 하드웨어 없이 개발·검증하며 기본 구동은 비활성화입니다. 실측 본체 크기는 각각 0.99 × 0.55m입니다. 일자 정렬 시 트랙터 뒤끝→연결 축은 0.15m, 연결 축→트레일러 앞끝은 0.06m로, 본체 사이 간격은 0.21m이고 전체 길이는 2.19m입니다. 차축·센서·모터 값은 아직 PROVISIONAL입니다.

## G6 전방 기준 (2026-10-21 실물 확인)

라이다의 사용자가 지정한 실제 전방을 `laser_frame`의 **+X**로 맞췄습니다.
기본 `config/g6.yaml`의 `reversion: true`가 원시 스캔을 180° 회전시키며,
`g6_lidar.launch.py`와 `g6_slam.launch.py`가 이 설정을 함께 사용합니다.
기본 `rviz/g6.rviz`는 지도 +X를 오른쪽, +Y를 위쪽으로 표시합니다.
SLAM 시작 시 전방은 오른쪽이고, 이후 회전하면 지도 위 센서 방향도 함께 변합니다.
`g6_front_up.rviz`는 +X를 위쪽으로 표시하는 별도 화면 설정입니다.
정상 SLAM 거리 범위는 0.15~12m이며, 0.15~0.8m는 이번 방향 확인용 임시 설정입니다.

## 연결된 EBIMU와 G6 SLAM (2026-10-19)

새 IMU의 버전 응답은 `<imu9dofv612b>`이며, 현재 출력 설정은
`sof2/sog1/soa1/sots1/sor10`입니다. 115200 baud에서
`*qz,qy,qx,qw,gx,gy,gz,ax,ay,az,timestamp_ms\r\n`를 100Hz로 수신합니다.
`ebimu_slam.yaml`은 이 형식을 ROS `sensor_msgs/Imu`로 변환합니다.
자이로는 deg/s→rad/s, 중력을 포함한 가속도는 g→m/s²로 변환하고,
쿼터니언은 ROS x,y,z,w 순서로 재배열합니다. 기본 센서 축을 그대로 유지하며,
물리적 장착 방향은 별도 TF로 지정합니다. 수신 중 설정·보정 명령은 보내지 않습니다.
센서 timestamp는 파싱하지만 ROS header에는 호스트 수신 시각을 사용하며,
버스트 수신 시 10ms 간격으로 보간합니다. 센서와 라이다의 하드웨어 시계 동기화는 하지 않습니다.

```bash
cd ~/mvsim_slam_real
source /opt/ros/humble/setup.bash
source install/setup.bash
# 현재 확인한 /dev/serial/by-id/... 포트를 ebimu_slam.yaml에서 사용
ros2 launch mvsim_slam_real_bringup ebimu.launch.py
# 다른 터미널 (동일하게 source)
ros2 topic echo /tractor/imu/data --qos-reliability best_effort
```

G6와 함께 사용하려면 `config/g6_imu_mount.yaml`의 `xyz`(m), `rpy`(rad)에
**laser_frame 기준 IMU 위치와 축 방향의 실측값**을 채우고 `measured: true`로 설정합니다.
IMU 드라이버의 `mounting_rpy`는 0으로 둡니다. sensor 모드는 차체 좌표 측정 없이
라이다 기준 자세를 발행하며, Cartographer tracking origin은 IMU 위치입니다.

```bash
# 위의 독립 IMU 실행을 종료한 뒤 실행. G6_PORT는 실제 라이다 포트로 교체.
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py \
  lidar_port:=G6_PORT frame_mode:=sensor use_imu:=true \
  imu_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  enable_rviz:=true
```

이미 실행 중인 IMU 노드를 사용하려면 `start_imu:=false`를 지정합니다.
이미 실측 TF를 다른 노드가 발행한다면 `publish_imu_tf:=false`를 지정합니다.
vehicle 모드는 기존 실측 LiDAR mount와 base_footprint 기준 IMU mount가 필요합니다.
IMU 융합은 기본 OFF이고, 실측하지 않은 IMU TF 파일로는 융합 실행을 거부합니다.
현재 두 UART 장치에 같은 포트를 지정할 수 없습니다.

실기 검증: `/tractor/imu/data` 958개, 100Hz, 가속도 크기 9.65–9.92m/s²,
정규화된 쿼터니언과 증가하는 header 시각을 확인했습니다. 별도 ROS domain에서
실제 Cartographer의 해당 IMU 토픽 구독과 설정 로딩까지 확인했습니다.
당시 라이다가 연결되어 있지 않아 실제 scan 융합·지도 생성은 미검증이며,
센서 장착 좌표도 아직 미측정입니다. 결과는
`diagnostics/imu_new_20261019/slam_live_summary.json`에 있습니다.
아래 기존 개발 기록의 하드웨어 미검증 설명은 초기 작업 시점 기준입니다.

## 빌드와 기본 실행

트랙터·트레일러 IMU는 실제 조회한 설정 30개가 동일합니다 (2026-10-24).
두 센서는 동일한 115200 baud / Quaternion·gyro·중력 포함 accel·timestamp / 100Hz
프로파일을 사용하고, 고정 장치명은 `/dev/imu_tractor`, `/dev/imu_trailer`입니다.
`imu.yaml`의 드라이버 축 보정은 둘 다 0이며, 트레일러의 반대 장착 방향은
`vehicle_geometry.yaml`의 `trailer_imu_offset` yaw=π에서 한 번만 적용합니다.
개별 센서 보정값·영점·bias는 복사하지 않았으며 실제 축 방향·수평 정렬은 추가 검증이 필요합니다.

```bash
# ROS와 workspace를 source한 터미널. 다른 IMU 노드를 종료한 뒤 실행.
ros2 launch mvsim_slam_real_bringup dual_imu.launch.py
# 기존 센서 TF 발행기가 이미 있다면 publish_sensor_tf:=false 추가.
```

위 명령은 IMU 두 개와 센서 정적 TF만 실행하며 차량 제어·센서 보정은 하지 않습니다.
실기 검증에서 두 토픽은 각각 약 100Hz, 장착 TF는 tractor yaw=0°·trailer yaw=180°로
확인했습니다. 결과: `diagnostics/imu_match_20261024/dual_imu_live_summary.json`.

```bash
cd ~/mvsim_slam_real
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch mvsim_slam_real_bringup real_parking.launch.py
```

기본 `development_mode=true`, LiDAR gateway·두 IMU·Cartographer·TF·상태추정·B AutoParking·Adapter를 실행합니다. IMU는 장치를 열지 않습니다. LiDAR 입력은 `/lidar_raw` 표준 LaserScan이며 입력이 없으면 SLAM·추정 상태는 unavailable입니다. 비전·RViz·시리얼 송신은 기본 비활성화입니다. `enable_rviz:=true`로 RViz를 표시합니다. 하드웨어 대신 정상 측정을 생성하지 않습니다. 실제 하드웨어 실행은 이번 작업에서 수행하지 않았습니다.

Launch 옵션: `development_mode`, `enable_lidar`, `lidar_driver` (`external` 또는 `rplidar`), `enable_tractor_imu`, `enable_trailer_imu`, `enable_slam`, `enable_imu_fusion`, `enable_vision`, `enable_rviz`, `enable_actuation`, `geometry_file`, `output_directory`, `lidar_port`, `tractor_imu_port`, `trailer_imu_port`, `arduino_port`, `camera_path`, `vision_model`.

Launch 자식 Python 노드는 `PYTHONNOUSERSITE=1`을 사용합니다. 이 PC의 사용자 NumPy와 시스템 SciPy 버전 충돌을 피하며 전역 환경이나 설치를 변경하지 않습니다. 개별 노드와 테스트도 필요하면 아래처럼 해당 셸에서만 지정합니다.

```bash
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
GEOMETRY="$HOME/mvsim_slam_real/src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml"
```

## 개별 실행

두 차량을 일자로 정지시킨 뒤 센서 안정화를 기다리고 각 IMU의 초기 대비 좌우 회전량을
2D RViz에서 보려면 `bash tools/run_relative_yaw.sh`를 실행합니다.
기본 5초 안정화 후 READY를 표시하며 IMU는 Cartographer에 입력하지 않습니다.
실행·준비 조건·기준 재설정은 [초기 대비 좌우 회전량 안내](tools/RELATIVE_YAW.md)를 참조하세요.

카메라 없는 WASD 수동조종과 SLAM은 `bash tools/run_manual_slam_wasd.sh`로 함께 실행합니다.
`ros2_parking` 데이터 수집기와 같은 방식으로 W/S는 PWM ±10(한계 ±250), A/D는 조향 ±1(한계 ±7),
R은 속도·조향 초기화, F는 전체 종료입니다. 0.1초마다 증감하며 키를 떼어도 값을 유지합니다.
동시 입력은 W > S > A > D > R > C > F 우선순위입니다. 카메라 연결·C 촬영 기능만 제외했습니다.
시작 RViz는 두 차체가 일자로 평행하며, 5초 안정화 후 첫 READY에서 양쪽 회전량을 0°로 시작합니다.
수동조작은 Arduino에 직접 전송하며 SLAM은 관찰용입니다. 실제 두 차체도 일자로 정렬하고 시작합니다.

기존 터미널 키보드 진단과 SLAM 수신 상태 확인 도구는
[`tools/manual_slam_check.py`](tools/manual_slam_check.py)입니다.
실행 방법과 기존 제어기 연결 조건은 [수동 SLAM 확인 안내](tools/MANUAL_SLAM_CHECK.md)를 참조하세요.

아래 명령은 위의 ROS 환경을 source한 새 워크스페이스에서 실행합니다. 하드웨어 수신은 기본 OFF입니다. 알고리즘 독립 노드들은 통합 AutoParking과 함께 구동할 필요가 없습니다.

| 기능 | 실제 실행 명령 |
|---|---|
| 표준 LiDAR Processor/Gateway | `ros2 run real_vehicle_bridge lidar_gateway` |
| 선택 RPLidar Publisher (장치 OFF) | `ros2 run real_vehicle_bridge rplidar_source` |
| 기존 LiDAR Processor | `ros2 run lidar_perception_pkg lidar_processor_node` |
| Tractor IMU (장치 OFF) | `ros2 run ebimu_driver ebimu_node --ros-args -r __node:=tractor_imu -p output_topic:=/tractor/imu/data -p frame_id:=tractor_imu_link` |
| Trailer IMU (장치 OFF) | `ros2 run ebimu_driver ebimu_node --ros-args -r __node:=trailer_imu -p output_topic:=/trailer/imu/data -p frame_id:=trailer_imu_link` |
| 정적 센서 TF | `ros2 run real_vehicle_bridge sensor_tf --ros-args -p geometry_file:="$GEOMETRY"` |
| Cartographer·OccupancyGrid | `ros2 launch mvsim_slam_real_bringup slam.launch.py` |
| 상태추정 | `ros2 run real_vehicle_bridge state_estimator --ros-args -p geometry_file:="$GEOMETRY"` |
| Gap Detector (진단) | `ros2 run real_parking gap_detector` |
| B Hybrid A* (진단) | `ros2 run real_parking hybrid_astar --ros-args -p geometry_file:="$GEOMETRY"` |
| LQR (독립 출력) | `ros2 run real_parking lqr --ros-args -p geometry_file:="$GEOMETRY"` |
| Arduino Adapter (출력 OFF) | `ros2 run real_vehicle_bridge control_adapter --ros-args -p geometry_file:="$GEOMETRY"` |
| 전체 B 상태 머신 | `ros2 run real_parking auto_parking --ros-args --params-file src/mvsim_slam_real_bringup/config/parking.yaml -p world_file:="$GEOMETRY" -p blocked_debug_directory:="$HOME/mvsim_slam_real/log/manual"` |

원본 LiDAR Publisher의 정식 엔트리는 `ros2 run lidar_perception_pkg lidar_publisher_node`입니다. 포트가 하드코딩되어 있으므로 이번 개발에서는 실행하지 않았습니다. 신규 Publisher/Gateway를 권장합니다. 원본 setup에 등록된 `lidar_obstacle_detector_node`와 `traffic_light_detector_node`는 실제 모듈이 없는 기존 항목이어서 실행 대상으로 안내하지 않습니다.

원형 Arduino 송신기의 엔트리는 `serial_communication_pkg / serial_sender_node`입니다. **장비 준비 후에만** `ros2 run serial_communication_pkg serial_sender_node --ros-args -p port:=<ARDUINO_PORT> -p sub_topic:=/topic_control_signal`로 지정합니다. 이번에는 실행하지 않았습니다. 펌웨어와 시리얼 패키지를 수정하지 않습니다.

## 센서·차량 설정

설정은 `src/mvsim_slam_real_bringup/config/`에 있습니다. YAML 변경 후 `colcon build --symlink-install`을 실행합니다.

- `vehicle_geometry.yaml`: 본체 간격 `connection_length`는 트랙터 뒤끝→연결 축 `tractor_rear_to_hitch_distance`와 트레일러 앞끝→연결 축 `trailer_front_to_hitch_distance`의 합입니다. 두 거리는 비대칭으로 설정할 수 있습니다. 독립 wheelbase, rear axle, steering axis, trailer axle, hitch, hitch-to-axle, connection length, max steer/rate, 전후진 속도, LiDAR·두 IMU 오프셋. 위치 m, 각도 rad. 서로 일치하지 않는 기구학 값은 거부합니다. 트레일러는 동적 힌지 관계입니다. 계획 기준점은 트랙터 뒤 차축, SLAM tracking frame은 차체 중심입니다.
- `imu.yaml`: EBIMU-9DOFV6 두 개의 독립 port, topic, frame, baud, axis correction. 실제 수신을 활성화하려면 `development_mode=false`와 각 포트를 명시합니다. 두 포트가 같으면 Launch가 거부합니다.
- `lidar.yaml`: LiDAR 모델은 미정입니다. 선택한 제조사 ROS 드라이버를 `/lidar_raw`로 연결합니다. RPLidar 호환 장치에만 `lidar_driver=rplidar`와 `lidar_port`를 지정합니다. `clockwise`, offset, ranges는 장치 확인 후 설정합니다. 실제 beam timestamp가 없는 legacy RPLidar binning은 instantaneous 근사입니다.
- `controller.yaml`: PWM/(m/s), 조향/모터 방향, 속도·명령 신선도와 확인 플래그. 변환 계수·부호는 실측 전 미확정입니다. `enable_actuation=false`가 기본입니다.
- `hardware.yaml`: 확정 IMU/미정 LiDAR와 구동 비활성화 현황. 포트는 Launch 인자로 받으며 이 파일은 장비 설정 기록입니다.
- `parking.yaml`: 기존 B slot prior, 검색 속도, LQR 속도 factor, timeout, 완료 조건. slot prior는 탐지된 공간 치수와 구분됩니다. 원본 탐지 조건·.18m clearance·30/35° 관절각 한계는 보존했습니다.
- `slam.yaml`, `reference_lidar_only.lua`: 최종 B 기본 설정. `reference_imu_optional.lua`는 별도 선택이며 실제 센서 fusion은 미검증입니다.

EBIMU 기본 수신 형식은 `*Roll,Pitch,Yaw,gx,gy,gz\r\n`이며 gyro DPS를 rad/s로 변환합니다. 수신 노드 시작 시 `<cg>`나 설정 명령은 보내지 않습니다. 제조사 PDF 세 개를 로컬에서 읽었으며 추출본은 `references/`에 있습니다.

```bash
# 아래는 OFFLINE PREVIEW만 합니다. 장치 접속·송신 없음.
ros2 run ebimu_driver ebimu_configure configuration
ros2 run ebimu_driver ebimu_configure version
ros2 run ebimu_driver ebimu_configure configure
ros2 run ebimu_driver ebimu_configure calibrate_gyro
```

향후 실제 설정 전송에는 `--send --port <IMU_PORT>`가 별도로 필요하고, gyro 보정에는 `--stationary-confirmed`가 추가로 필요합니다. 기본 configure는 `<sem0><soc1><sof1><sog1><soa0>` 및 나머지 추가 채널 OFF, `<sor10>` 100Hz를 사용합니다. 선택적 IMU fusion은 `<soa1>` 중력 포함 가속도 9필드가 필요합니다: configure `--accelerometer`와 Launch `enable_imu_fusion=true`를 함께 설정해야 합니다. 실제 중력 방향·축·장착 자세를 검증한 뒤 사용합니다. `<soa0>` 측정 부재를 정상 가속도로 채우지 않습니다.

## 알고리즘 인터페이스

통합 AutoParking은 `/parking/state`, `/parking/detection_map`, `/tractor/localization/local_map`, chassis polygons를 받고 `/parking/request_cmd_vel`을 발행합니다. 이 Twist의 angular.z는 **yaw rate rad/s**이며 조향 rad가 아닙니다. Adapter가 `atan(wheelbase*omega/v)`로 복구합니다. Adapter preview는 `/parking/motion_preview`, 기존 메시지 출력은 `/topic_control_signal`입니다. 기본 모드에서 후자는 메시지를 발행하지 않습니다.

독립 Hybrid A*는 `/parking/planner_request` String JSON `{ "start":[x,y,yaw0,yaw1], "goal":[x,y,yaw0,yaw1], "gears":[1,-1] }`를 받고 최신 planning map으로 계획합니다. `/parking/planner_result`는 `valid,states,gears,steering` 또는 실패 reason을 담습니다. 독립 LQR는 이 결과와 `/parking/state`를 받아 `/parking/lqr_cmd_vel`에 출력합니다. 통합 AutoParking의 제어 토픽과 구분하며 독립 도구를 무조건 Adapter에 remap하지 않습니다. Gap 진단은 `/parking/gap_observation`, `/parking/diagnostic_target`입니다.

TF: Cartographer만 map→odom→base_footprint를 발행합니다. sensor_tf는 센서·base_link·local_map 정적 TF, estimator는 base_footprint→trailer_link 동적 TF를 발행합니다. RViz는 scan/map/양 차체 pose/TF/측정 Target/양 차체 계획 경로/실제 tracking path/제어 reference/주차 상태를 표시합니다.

## 비전 보존

기존 camera_perception_pkg, debug_pkg, decision_making_pkg와 모든 리소스를 보존합니다. 카메라 장착·모델 준비 후 `ros2 launch mvsim_slam_real_bringup vision.launch.py camera_path:=<CAMERA_PATH> model:=<MODEL_FILE> device:=cpu`를 사용합니다. 이 Launch는 perception만 실행합니다. 원본 `launch_pkg/main.launch.py`는 구동 노드를 포함하므로 새 개발 모드에서 사용하지 않습니다.

개별 perception 엔트리: `image_publisher_node`, `yolov8_node`, `parking_space_detector_node`, `parking_space_info_extractor_node`, `car_detector_node` (camera_perception_pkg), `path_visualizer_node`, `yolov8_visualizer_node` (debug_pkg). 기존 motion_planner_node와 parking_space_path_planner_node(decision_making_pkg)는 보존했지만 새 제어와 동시에 실행하지 않습니다. 이 환경의 ROS Python에서 torch가 없어 YOLO 런타임은 미검증/의존성 미충족입니다. 모델 weights도 별도 준비해야 하며 핵심 빌드·실행에는 영향을 주지 않습니다. 시스템 Python 패키지는 변경하지 않았습니다.

## 하드웨어 없는 검증

```bash
cd ~/mvsim_slam_real
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 -m pytest tests -q
/usr/bin/python3 tools/verify_preservation.py
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=222 ROS_LOCALHOST_ONLY=1 \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 tools/ros_smoke.py
```

ros_smoke는 별도 ROS domain에서 합성 센서 메시지를 생성합니다. 기본 데이터 없는 Launch 확인과, 합성 LaserScan→실제 Cartographer→gyro state→B 검색 상태→Adapter preview 및 입력 중단을 검사합니다. **실차·센서 테스트가 아닙니다.** 생성된 정상 데이터는 SYNTHETIC_ONLY fixture임을 로그와 JSON에 명시합니다. 검증 로그는 `verification/`에 저장합니다. 최종 11개 패키지 빌드, 20개 테스트, 기본/합성/선택 설정 Launch와 정상 종료가 통과했습니다. 자세한 범위는 [VERIFICATION.md](VERIFICATION.md)에 있습니다.

## 센서 연결 전에 남은 일

차축·힌지·센서 위치, 연결봉·축간거리·최대 조향을 실측하고 YAML을 맞춥니다. 센서 축 방향/조향 부호/모터 변환 계수와 UART 설정을 검증해야 합니다. 초기 실제 관절각을 정렬하고 gyro bias를 확인해야 합니다. `geometry_confirmed`, `initial_alignment_confirmed`, `trailer_bias_confirmed`, 모터 scale/polarity 확인 전 구동 Launch는 거부합니다. LiDAR 모델 선정, 카메라/YOLO 의존성·모델 준비, 실제 SLAM·각도·제어 검증은 별도 후속 작업입니다.

보존 펌웨어에 수신 watchdog·encoder feedback·독립 emergency cutoff가 없습니다. 소프트웨어 Adapter는 중단 시 zero speed를 발행할 수 있지만 호스트/USB 고장 후 보드의 모터 정지를 보장하지 않습니다. 이는 해결하지 않은 기존 구동 계층 한계이며, 이번에 펌웨어를 수정하지 않았습니다. 임시 치수로 안전한 실차 주차가 검증되었다고 주장하지 않습니다.

재개 시 `PROGRESS.md`, `progress.json`과 실제 코드/로그를 먼저 확인하고 완료된 분석을 반복하지 않습니다. 원본과 보호 복사본 수정, git push, 실제 장치 연결·보정·펌웨어 업로드·구동은 이번 작업에서 금지합니다.

## 벽 없는 실차 관측 실험 (별도 기능 브랜치)

[현장 실행·기록·판정 절차](docs/WALLLESS_VALIDATION.md)를 참조하세요.
`wallless_validation.launch.py`는 기존 G6/Cartographer 출력을 관측하고,
원시 스캔 클러스터, 오른쪽 두 물체 사이 공간 후보, 위치 추정 반복성 측정용 로그를 제공합니다.
차량 구동이나 기존 자동주차기 연결은 하지 않습니다. 차량 종류를 확정하는 분류기는 아닙니다.
