# 최종 B와 같은 2D RViz — 실차 입력 전용

2D 코드·디자인 이식과 소프트웨어 시험을 완료했다. **현재 실제 두 IMU·G6·Cartographer 연동은 미검증이다.**
기본 LIVE 실행은 실제 입력만 기다린다. 가상 지도·Scan·IMU·Ground Truth를 자동으로 공급하지 않는다.
기존 `visualization/tractor_trailer_rviz/`의 3D 파일은 보존했고 새 Launch에서는 실행하지 않는다.

## 실제 기준 코드

최종 실행 경로를 끝까지 확인했다.

```text
~/mvsim_slam/run_parking.sh
  -> mvsim_slam_final/scripts/run.py --configuration 2 --mode parking --gui
  -> src/parking_system/scripts/auto_parking_entry.py
  -> vendor/tractor_parallel_parking_auto.py AutoParking.tick -> visualize()
  -> /parking/debug_markers -> src/parking_system/config/slam.rviz
```

`ParkingNode`는 `active_world_geometry.json`의 차체 꼭짓점을 차체 로컬 좌표로 바꿔 Polygon으로 발행한다.
`AutoParking.on_polygon()`이 `Footprints`를 구성하면서 원본 XML의 바퀴 영역과 전륜 조향 범위를 합친다.
그 결과 트랙터 10개·트레일러 9개 꼭짓점의 hull을 `visualize()`가 그린다.
정적 형상 추출에 초기 기준점 숫자를 사용했으며, 이 숫자를 실차 위치 입력으로 사용하지 않는다.

`reference_renderer.py`의 `visualize()` 메서드는 원본과 동일하다. 제어·계획 노드를 import하거나 생성하지 않는다.
`models/reference_footprints.json`은 위 원본 기하 코드로 계산한 로컬 꼭짓점이다. Mesh/URDF는 사용하지 않는다.

| 항목 | 원본과 같은 값 |
|---|---|
| 외곽선 | LINE_STRIP, RGB=(0,1,0.7), alpha=0.85, 두께 0.04m, 높이 0.08m |
| 화살표 | 원본의 **트레일러 화살표 한 개**, RGB=(1,0,0), alpha=1, 0.8/0.08/0.18m, z=0.15m |
| 화살표 위치 | 트레일러 hull 꼭짓점 평균 |
| 텍스트 | 흰색 TEXT_VIEW_FACING, 높이 0.4m, 트랙터 기준점 x,y+1,z=1 |
| 텍스트 형식 | `{실제 상태} phi={trailer_yaw-tractor_yaw:.1f} deg` |
| Marker 갱신 | 원본 메서드의 약 0.4초 주기 유지; 데이터/중단 감시는 20Hz |
| Map | `/map`, alpha=0.8, map 색상 규칙, Transient Local |
| Scan | `/scan`, FlatColor 255/20/20, 3px/0.03m, Decay=0, Best Effort |
| Grid | 60칸, 1m, 회색100/100/100 |
| Camera | map, TopDownOrtho, Scale25, X10/Y0, Angle0, 배경35/35/35 |

원본에는 독립 히치 선 Marker나 트랙터 화살표가 없다. 차체 hull의 연결부와 히치 기준 배치로 표현한다.
그 요소를 새로 만들어 추가하지 않았다. 예측 경로·관측 주차면 등의 기존 Display는 원본 토픽을 읽으며,
실제 발행자가 없으면 비어 있다. 주차 상태·계획·제어를 이 패키지가 생성하지 않는다.

## 1. 빌드

```bash
bash /home/venezuelachocolatemilk/mvsim_slam_real/visualization/tractor_trailer_rviz_2d/scripts/build.bash
```

이 디렉터리 내부의 build/install/log만 사용한다. 기존 실차 workspace install이나 원본 시뮬레이터를 빌드하지 않는다.

각 LIVE 터미널에서 다음 환경을 적용한다. ROS domain은 **기존 실차 SLAM과 같아야 한다.**
아래 0은 이번 조사에서 확인한 실차 기본 환경이다. SLAM이 다른 domain이면 해당 값으로 바꾼다.
시뮬레이터 원본 환경 스크립트(domain219, use_sim_time)를 source하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source /home/venezuelachocolatemilk/mvsim_slam_real/visualization/tractor_trailer_rviz_2d/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
cd /home/venezuelachocolatemilk/mvsim_slam_real/visualization/tractor_trailer_rviz_2d
export ROS_LOG_DIR="$PWD/evidence/ros_live_logs"
```

## 2. 기본 2D 화면 실행 — 기존 실제 SLAM을 읽기만 함

```bash
ros2 launch tractor_trailer_rviz_2d visualization.launch.py \
  output_directory:="$PWD/evidence/live_$(date +%Y%m%d_%H%M%S)"
```

G6, Cartographer, 기존 IMU 드라이버, 차량 제어, 이전 3D 노드를 시작하지 않는다.
새 노드가 발행하는 것은 `/vehicle_viz_2d/model`과 `/vehicle_viz_2d/status`뿐이다
(ROS 기본 parameter_events/rosout 제외). **TF는 발행하지 않는다.**
Fixed Frame은 map이고 Marker는 map 좌표로 계산된다. 원본의 `local_map`은 map의 identity alias였으므로
직접 map을 사용해 동일 배치를 얻는다. 기존 TF·Scan·SLAM pose를 회전시키거나 덮어쓰지 않는다.

현재 기본 config/live.yaml의 실측 기하·장착축 확인은 비어 있다.
입력이 없으면 NO_DATA, 기하/정렬이 없으면 미확인 상태를 표시하고 차량이나 phi를 만들지 않는다.
현재 SLAM이 실행되지 않으면 RViz의 Map/Scan/TF 경고가 정상적으로 나타날 수 있다.

## 3. 실차 기하·장착축 설정

새 설정 파일을 복사해 실측한 값을 기록한다. 임시 시뮬레이션 치수나 이전 실차 임시값을 실측값으로 쓰지 않는다.

```bash
cp config/live.yaml config/measured_vehicle.yaml
```

`config/measured_vehicle.yaml`에서 확인할 필드:

- `slam_frame`: 실제 Cartographer 위치 추적 프레임. 기존 센서 모드는 laser_frame,
  차량 모드는 base_footprint다. base_footprint가 후륜축이라고 가정하지 않는다.
- `geometry.slam_to_rear_axle_xy`: 추적 프레임 기준점에서 후륜축까지 벡터, **추적 프레임 축으로 표현**, m.
- `geometry.slam_to_body_yaw_rad`: 추적 프레임에서 트랙터 전방 축까지 장착 yaw, rad.
- `rear_axle_to_hitch_xy`: 트랙터 후륜축에서 히치까지 벡터, 트랙터 축으로 표현.
- `trailer_origin_to_hitch_xy`, `trailer_origin_to_axle_xy`: 트레일러 로컬 기준점에서 히치·축까지 벡터.
- 위 측정이 완료된 후 `geometry.confirmed: true`, `source: MEASURED_REAL_VEHICLE`.
- 두 센서 각각 `sensor_to_body_rpy`: R_body_sensor, 즉 센서 축을 차체 축으로 옮기는 고정축 RPY(rad).
  Z 수직, Quaternion 회전 의미, 좌우 방향을 실제 손 회전으로 확인한 뒤
  `mounting_and_sign_confirmed: true`, 실제 확인한 `sign: 1` 또는 `-1`을 기록한다.

시각적 hull의 크기·비율은 원본 그대로다. 실제 차체 치수와 동일하다고 주장하지 않으며 충돌/제어에 쓰지 않는다.
실제 히치 벡터는 별도로 적용한다. 원본 hull의 트레일러 로컬 원점 자체가 히치이므로
원본 외곽선을 실제 히치 위치에서 회전시킨다. 물리 트레일러 축 위치도 별도로 계산해 status에 기록한다.
가동 범위를 초과한 큰 접힘에서는 실제 차체 간섭까지 모델링한 것이 아니므로 충돌 없음으로 해석하면 안 된다.

```bash
ros2 launch tractor_trailer_rviz_2d visualization.launch.py \
  config_file:="$PWD/config/measured_vehicle.yaml" \
  output_directory:="$PWD/evidence/live_measured_$(date +%Y%m%d_%H%M%S)"
```

이미 발행 중인 IMU를 읽으려면 위 파일의 topic/frame을 실제 값과 일치시킨다.
Launch의 tractor_topic/trailer_topic/slam_frame으로 토픽·SLAM 프레임을 지정할 수도 있다.
기존 `/tractor/imu/data`, `/trailer/imu/data`로 **발행하지는 않는다**.

## 4. 선택 사항: 실제 EBIMU 수신 브리지

물리 포트·센서 역할·G6 포트가 구분된 뒤에만 실행한다.

```bash
/usr/bin/python3 scripts/list_devices.py
ls -l /dev/serial/by-id/* /dev/serial/by-path/*
```

현재 조사에서는 CP2102 serial0001 한 개뿐이며 물리 역할은 미확인이다.
CP2102라는 이름만으로 IMU 또는 G6라고 판단하지 않았다. 이번 작업에서는 UART를 열지 않았다.
기존 드라이버가 포트를 사용 중이면 해당 토픽을 구독하고 새 브리지로 같은 포트를 열지 않는다.

다음 명령은 사용자가 확인한 포트를 입력받는다. 두 번째 센서가 없으면 start_trailer:=false를 유지한다.

```bash
read -r -p '확인된 트랙터 EBIMU by-id 경로: ' TRACTOR_IMU_PORT
read -r -p '확인된 트레일러 EBIMU by-id 경로: ' TRAILER_IMU_PORT
read -r -p '확인된 G6 포트 (미연결이면 Enter): ' LIDAR_PORT
ros2 launch tractor_trailer_rviz_2d imu_bridges.launch.py \
  start_tractor:=true start_trailer:=true ports_confirmed:=true \
  tractor_port:="$TRACTOR_IMU_PORT" trailer_port:="$TRAILER_IMU_PORT" \
  lidar_port:="$LIDAR_PORT" \
  output_directory:="$PWD/evidence/uart_$(date +%Y%m%d_%H%M%S)"
```

기본은 두 start_* 모두 false이며 포트를 하드코딩하지 않는다. 별도 물리 포트와 점유 여부를 검사한다.
센서 명령 TX=0. 설정/초기화/캘리브레이션 명령을 보내지 않는다.
단, 포트 open 자체의 호스트 DTR/RTS 변화 가능성은 있다.
기본 파서는 **과거 실측**의 ASCII `qz,qy,qx,qw,gx,gy,gz,ax,ay,az,timestamp_ms`다.
실제 현재 형식과 일치하는지 원시 로그로 먼저 확인한다. 기본 형식이 맞다고 추정해 설정 명령을 보내지 않는다.
Euler 단독이 실제 확인된 센서는 해당 role의 `*_orientation:=euler *_gyro:=false *_accel:=false *_timestamp:=false`를 지정한다.
Binary는 지원하지 않는다. accel=true는 soa1의 body 중력 포함 가속도에만 해당한다.
수신하지 않은 항목은 covariance[0]=-1, 받은 항목의 공분산은 미계측=0(unknown)이다.
브리지는 각 role 디렉터리의 rx.bin/samples.jsonl에 실제 바이트와 샘플을 저장한다.
RX=0이면 하드웨어 수신 실패이며 RViz 성공으로 보고하지 않는다.

브리지 토픽은 `/vehicle_viz_2d/{tractor,trailer}/imu/data`, 프레임은 `vehicle_viz_2d_{role}_imu`다.
수신이 끊기면 새 Imu를 생성하지 않는다. 센서 timestamp reset/repeat는 INVALID로 중단하고 재시작·재정렬이 필요하다.

## 5. 초기 일직선 방향 정렬 / 실제 검증

실제 두 차체를 일직선으로 정렬하고 센서를 정지시킨다. 두 IMU·SLAM이 LIVE이고
실측 기하·장착축이 확인된 후 다음 요청을 보낸다. 이는 센서 설정 변경이 아니라 시각화 기준 기록이다.

```bash
ros2 service call /vehicle_viz_2d/align_straight std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /vehicle_viz_2d/status
```

초기 지도 방향은 Cartographer 방향 + 측정된 추적 프레임 장착 yaw로 정렬한다.
이후 각 차체 yaw는 그 초기 지도 방향 + **각 IMU의 독립 변화량**이다.
Roll/Pitch로 2D 차체를 기울이지 않는다. Cartographer yaw로 트랙터 회전을 대체하지 않는다.
Cartographer yaw는 추적 기준점→후륜축 lever arm 보정에도 사용된다.
트레일러 yaw를 트랙터 yaw로 대신하지 않는다.

phi는 원본과 같은 `wrap(trailer_yaw - tractor_yaw)`다. 따라서 트랙터+30°, 트레일러+10°이면 **phi=-20°**다.
±180° wrap을 처리하고 각 센서의 unwrapped yaw를 동일 시각에 보간한다.
최신 입력 timestamp 차이는 기본50ms 이내여야 하며 extrapolation은 하지 않는다.
브리지 timestamp는 호스트 수신 시각이다. 센서 내부 시간·전송 지연의 물리적 동기화를 보장하지 않는다.
gyro_z 모드는 명시적으로 선택하고 정지 bias를 확인해야 한다. 적분 drift를 포함하며 AHRS yaw와 구분한다.

손으로 센서만 움직여 한 단계씩 확인한다. 차량 이동·조향은 하지 않는다.

1. 트레일러 센서는 고정하고 트랙터 센서만 회전: 트랙터 외곽선만 방향 변화, 트레일러 전역 yaw 유지.
2. 트랙터 고정, 트레일러만 회전: 트레일러 외곽선·빨간 화살표 방향 변화.
3. 서로 다른 각도 / 같은 각도: phi가 각각 부호 정의와 일치 / 거의0인지 확인.
4. 한 브리지 Ctrl+C: 해당 센서 STALE, phi 제거, 유지되는 외곽선에는 LAST_POSE_NOT_LIVE 표시.
5. 수신 재개 후 ALIGNMENT LOST이면 다시 일직선 정렬 서비스를 호출한다. 자동 정상 복귀로 위장하지 않는다.

실제 `/parking/status`가 없거나3초 이상 오래되면 상태명은 STATE_UNKNOWN이다.
SEARCH_FORWARD를 고정 문자열로 사용하지 않는다. 전체 입력이 없으면 NO_DATA, 잘못된 quaternion/frame은 INVALID.
정렬 전·수신 중단 시 phi를 정상 수치로 표시하지 않는다. Marker는 노드 종료 후0.8초 이내 만료한다.

## 6. 명시적 소프트웨어 TEST — 실제 하드웨어 성공 결과 아님

아래 실행만 가상 입력을 시작한다. domain88, `/vehicle_viz_2d_test/*`, `vehicle_viz_2d_test_*`를 사용한다.
기본 LIVE Launch에는 이 소스가 포함되지 않는다. 화면에는 원본 흰색 텍스트 형식에 TEST/SIMULATED를 명시한다.

```bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
ros2 launch tractor_trailer_rviz_2d test_preview.launch.py
```

재현 가능한16개 런타임 검사와 창 캡처:

```bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 scripts/verify_test_runtime.py \
  --output "$PWD/evidence/synthetic_$(date +%Y%m%d_%H%M%S)"
```

단위·디자인 비교 검사:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD:$PYTHONPATH" \
  /usr/bin/python3 -m pytest tests -q -p no:cacheprovider
```

## 종료 / 원복 / 현재 한계

각 새 Launch 터미널에서 Ctrl+C. 자신이 시작한 프로세스만 종료한다.
기존 G6·SLAM Launch와 원본 설정은 변경하지 않아 기존 실행 명령을 그대로 사용할 수 있다.
이전 3D 패키지는 삭제하지 않았으며, 해당 패키지의 Launch를 별도로 실행하지 않으면 새 화면에 나타나지 않는다.
그래픽 초기화 문제가 있을 때만 해당 RViz 프로세스 환경에 LIBGL_ALWAYS_SOFTWARE=1을 지정할 수 있다.

실제 센서 두 개의 역할·포트·축·Yaw 부호, 실측 기하, 실제 SLAM 지도 연동, 전송 지연·정지 안정성은 미검증이다.
원본의 주차면·계획 경로·제어 목표는 실제 해당 입력이 있을 때만 기존 Display에서 보인다.
데이터가 없다는 이유로 가상 목표나 상태 머신을 생성하지 않는다.
전체 한국어 보고서: `~/mvsim_slam_real/TRACTOR_TRAILER_2D_RVIZ_MATCH_REPORT.txt`.
