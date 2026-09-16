# Trailer Autodrive — 전후방 카메라 기반 평행주차

ROS 2 Humble / Ubuntu 22.04 / Gazebo Classic용 실험 구현입니다.
`trailer_parking_pkg`에 **LiDAR + YOLO**와 **카메라 + YOLO 전용** 실행 파일을
추가했습니다. 둘 다 견인차 전방 카메라와 트레일러 후방 카메라를 사용합니다.
상공 카메라, `/odom`, `/front_pose`, `/trailer/articulation_angle`,
`/gazebo/model_states`를 제어용 위치 입력으로 사용하지 않습니다.

현재 코드는 연구용 첫 구현입니다. ROS 패키지 빌드와 수치/메시지 테스트는
수행했지만, 기존 11.1m 공간에서 카메라 인식부터 Gazebo 주차 완료까지의
성공은 아직 검증하지 못했습니다. 확인 범위와 제약은 아래를 참고하세요.

## 모델에서 확인한 치수

실물 유아차 치수가 아니라 저장소의 Prius SDF 치수입니다.

| 항목 | 값 | 근거 |
|---|---:|---|
| 한 대 차체 길이 | 약 4.5505m | 차체 collision들의 전후 끝 |
| 차체 최대 너비 | 약 1.7881m | trunk collision |
| 타이어 collision 포함 보수적 너비 | 약 2.1973m | 뒷바퀴 중심 ±0.786m, 구형 collision 반지름 0.31265m |
| 견인차 휠베이스 | 2.86m | 앞바퀴 y=-1.41, 뒷바퀴 y=1.45 |
| 견인차 뒤축 → 히치 | 0.95m | 히치 y=2.4 |
| 히치 → 트레일러 뒤축 | 3.85m | 히치 y=-2.4, 뒤축 y=1.45 |
| 일직선 두 차량 전체 길이 | 약 9.3505m | 모델 원점 간 4.8m + 한 대 길이 |
| 정적 주차 차량 중심 간 거리 | 약 15.6453m | SLOT_1_POSE / SLOT_4_POSE |
| 앞뒤 주차 차량 사이 빈 길이 | 약 11.0948m | 중심 거리에서 한 대 길이를 뺀 근사 |
| 주차칸 페인트의 실제 너비 | 미확정 | `track.world`에는 치수 있는 주차칸 객체가 없고 바닥 텍스처만 있음 |

히치 플러그인 제한은 ±45°지만, 이 짧은 연결에서는 실제 SDF 범퍼끼리 약
22–25°에서 간섭할 수 있습니다. 새 제어기의 기본 연결각 제한은 **±20°**입니다.
임의로 40–45°까지 높이면 수치 경로가 나와도 실제 범퍼는 부딪힐 수 있습니다.
수치 테스트의 칸 너비 3.2m는 **검증용 가정**이며 페인트 실측값이 아닙니다.

```bash
python3 tools/inspect_parking_model.py --start-index 1
```

## 평행주차 동작의 근거

1. [Gómez-Bravo, Cuesta, Ollero, IFAC 2005](https://skoge.folk.ntnu.no/prost/proceedings/ifac2005/Fullpapers/03832.pdf):
   트레일러 방향 조정 동작을 먼저 수행하고 주차 동작을 연결하는 실차 평행주차
   사례를 참고했습니다. 논문의 특정 차량용 회전 반경·시간값은 복사하지 않았습니다.
2. [Cao et al., Hybrid A*-Based Reverse Path-Planning of a Vehicle with Single Trailer](https://www.mdpi.com/2079-9292/15/5/1114):
   뒤축 뒤의 히치를 포함하는 저속 운동학, 연결각 제한, 트레일러 후진의
   역운동학과 충돌 검사를 참고했습니다. 이 구현은 해당 논문 전체를 재현한 것이 아닙니다.
3. [Sakai의 HybridAStarTrailer](https://github.com/ompugao/HybridAStarTrailer):
   위치와 두 차체 방향을 포함하는 4차원 탐색, 두 차체의 충돌 검사를 참고했습니다.
4. [사용자 제공 Autonomous_Capstone](https://github.com/eriverOoO/Autonomous_Capstone/tree/1c0a19c5cca5db587174343ef0336ed5167d51c4):
   `parking_mission_node.py`, `ver2_parking_control_node.py`, `parking_launch.py`를
   읽었습니다. 상태별 명령과 기어 전환 정지 구조를 참고했으며, 한 대 수직주차의
   시간 시퀀스를 트레일러 평행주차 동작으로 재사용하지 않았습니다.

동작은 고정된 좌/우 조향 시간표가 아닙니다. 현재 위치·목표 칸·장애물에 맞춰
전진 준비 → 후진 진입 → 필요 시 전후진 보정 → 두 차체 평행 정렬을 계산합니다.
공간이나 연결각 조건을 만족하는 경로가 없으면 `STOP_NO_PATH`로 멈춥니다.

## 두 버전의 구성

| 기능 | LiDAR + YOLO | 카메라 + YOLO 전용 |
|---|---|---|
| 입력 영상 | 견인차 전방 + 트레일러 후방 | 동일 |
| YOLO 역할 | 빈 칸·차량 분할 인식 | 동일 |
| 차체 이동 추정 | 지면 특징점 기반 영상 이동 추정 | 동일 |
| 거리 스케일 | 보정된 카메라 높이/자세와 바닥 평면 | 동일 |
| 추가 장애물 입력 | 앞/뒤 360° LaserScan 두 개 | LiDAR 생성/구독 없음 |
| 주차 제어 | 동일한 트레일러 경로계획·추종기 | 동일 |

카메라 전용은 **YOLO 분할 + 기하학적 거리 환산 + 영상 기반 이동 추정 + 제어**입니다.
YOLO 하나가 조향을 직접 출력하는 종단간 학습 모델은 아닙니다. 카메라 보정 없이
YOLO 마스크만으로 미터 단위 거리와 연결각을 정확히 알 수 있다고 가정하지 않습니다.

`best.pt`의 pickle을 실행하지 않고 메타데이터를 읽어 확인한 클래스는
`parking_space`, `car_front`, `car_back`이고, task는 `segment`입니다.
기록된 학습 베이스는 `yolo26m-seg.pt`입니다. 기존 모델의 새 전후방 시점 성능이나
정확도는 아직 측정하지 않았습니다.

### 좌표계와 제어

`State(x, y, yaw, beta)`의 원점은 **견인차 뒤축 중앙**입니다.
진행 방향이 +x, 왼쪽이 +y이고 `beta = trailer_yaw - tractor_yaw`입니다.
원본 모델의 전방은 모델 -Y이므로 변환 시 90° 회전을 적용합니다.

```text
tractor_yaw_rate = v * tan(steering) / wheelbase
trailer_yaw_rate = (-v * sin(beta)
                   - hitch_offset * tractor_yaw_rate * cos(beta)) / trailer_axle
beta_rate = trailer_yaw_rate - tractor_yaw_rate
```

연속 조향/이동거리 최적화를 먼저 시도하고, 실패하면 4차원 Hybrid A*를 사용합니다.
후진 탐색 후보는 연결각의 역운동학으로 생성합니다. 경로는 8–10cm 간격으로
두 차체와 히치의 장애물 충돌을 다시 검사합니다. 추종기는 위치·차체 방향·연결각
오차를 함께 줄이며 기어 전환을 건너뛰지 않습니다.

경로 추종기의 최종 출력은 실차와 같은 `interfaces_pkg/MotionCommand`입니다.
`steering`은 -7~7, 좌·우 속도는 부호 있는 PWM -255~255이며 기본 0.30m/s를
PWM 90에 대응시킵니다. 두 구동륜에는 같은 PWM을 주고 전륜 조향을 별도로 제어합니다.
Gazebo Classic Ackermann 플러그인은 `Twist.angular.z`를 조향각으로 사용하고
후진일 때 부호를 내부에서 뒤집으므로, 이 차이는 시뮬레이션 어댑터에서만 보상합니다.
[플러그인 원문](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp)
의 `OnUpdate`를 기준으로 했습니다. 일반 differential-drive `/cmd_vel`과 의미가 다릅니다.

## 설치와 실행

Ubuntu 22.04 터미널에서 저장소 최상위로 이동합니다. ROS Humble 설치가 전제입니다.
이 작업 중 시스템 Gazebo나 GPU/PyTorch는 설치하지 않았습니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge \
  ros-humble-ackermann-msgs python3-colcon-common-extensions \
  python3-venv python3-numpy python3-scipy python3-opencv python3-serial
python3 -m venv --system-site-packages .venv-parking-linux
source .venv-parking-linux/bin/activate
python -m pip install -r requirements-parking.txt
colcon build --symlink-install --packages-select \
  interfaces_pkg serial_communication_pkg simulation_trailer_plugins \
  simulation_pkg trailer_parking_pkg
source install/setup.bash
```

ROS의 시스템 Python 실행 항목에서도 가상환경 패키지가 보이도록, 같은 터미널에서
아래 변수를 지정합니다. NumPy는 cv_bridge 바이너리와 호환되도록 1.x로 제한합니다.

```bash
export PYTHONPATH="$VIRTUAL_ENV/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
```

두 실행 파일 중 **하나만** 실행합니다. 기존 `driving_sim`, `mission_sim`,
`trailer_sim autonomy:=true`, 수동 `/cmd_vel` 발행기는 동시에 실행하지 않습니다.

```bash
# A. LiDAR + 전후방 YOLO
ros2 launch trailer_parking_pkg parallel_lidar_yolo.launch.py \
  weights:="$(pwd)/best.pt" start_index:=1 device:=cpu

# B. LiDAR 없이 전후방 YOLO
ros2 launch trailer_parking_pkg parallel_camera_yolo.launch.py \
  weights:="$(pwd)/best.pt" start_index:=1 device:=cpu
```

두 시뮬레이션 실행도 내부적으로 실차와 같은 `topic_control_signal`을 만들고,
`simulation_sender_node`만 이를 `/cmd_vel`로 변환합니다.

## 실제 유아차 실행

아두이노에는 [실차 펌웨어](src/control/driving/driving.ino)를 업로드합니다. 원본과 같은
115200bps 및 `s<조향>l<좌PWM>r<우PWM>\n` 형식을 사용하며, 통신이 300ms 끊기면
구동 모터를 정지하도록 fail-safe를 추가했습니다.

실차에서는 시뮬레이션 SDF 보정을 사용하지 않습니다. 전후방 카메라 외부 파라미터,
두 차체 치수와 초기 자세를 실측한 JSON을 반드시 `calibration`으로 전달합니다.

```bash
# A. 실차 LiDAR + 전후방 YOLO
ros2 launch trailer_parking_pkg parallel_lidar_yolo_hardware.launch.py \
  weights:="$(pwd)/best.pt" calibration:="$(pwd)/real_calibration.json" \
  port:=/dev/ttyACM0 tractor_scan:=/scan trailer_scan:=/rear_scan

# B. 실차 전후방 카메라 + YOLO 전용
ros2 launch trailer_parking_pkg parallel_camera_yolo_hardware.launch.py \
  weights:="$(pwd)/best.pt" calibration:="$(pwd)/real_calibration.json" \
  port:=/dev/ttyACM0
```

제어 흐름은 다음과 같습니다.

```text
경로 추종기
  -> /parking/raw_motion_command (MotionCommand)
  -> 250ms wall-clock watchdog / 범위 검사
  -> topic_control_signal
  -> serial_sender_node
  -> 115200bps: s0l90r90\n
  -> Arduino 전륜 조향 + 좌/우 구동 모터
```

`parking_pwm:=90`, `steer_steps:=7`이 원본 유아차 기본값입니다. 실제 속도와
`speed:=0.30`의 대응은 엔코더나 실측 거리로 다시 보정해야 합니다. 최초 시험은
구동륜을 바닥에서 띄우고 `s0l0r0`, 저속 전진, 저속 후진, ±1 조향 순서로 확인한 뒤
진행해야 합니다. 물리 비상정지 장치 없이 자동주차 시험을 시작하면 안 됩니다.

GPU가 구성된 환경은 `device:=cuda:0`를 사용합니다. CPU에서 두 카메라의 추론 지연이
크면 freshness 조건 때문에 정지할 수 있습니다. 지연을 실제로 측정하고 모델 크기나
추론 장치를 조정해야 합니다.

`start_index`는 1~4입니다. 로더와 영상 이동 추정이 같은 시작 위치를 사용합니다.
원본 로더를 단독 실행할 때는 이전과 같이 `0`=무작위가 기본값입니다.
영상 추정은 이 **초기 위치 사전정보**를 사용하지만 실행 중 Gazebo 정답 위치를
읽지 않습니다. 임의 위치에서 시작하는 실차에는 측정한 초기 위치가 필요합니다.

기본 영상 토픽은 `/camera/image_raw`, `/rear_camera/image_raw`, 각각의
`camera_info`입니다. 실행 인자 `front_image`, `front_info`, `rear_image`, `rear_info`로
바꿀 수 있습니다. `ros2 topic list`로 실제 연결을 확인하세요.

```bash
ros2 topic echo /parking/status
ros2 topic hz /parking/front/observation
ros2 topic hz /parking/rear/observation
```

추가 LiDAR는 견인차 앞 범퍼 밖과 트레일러 뒤 범퍼 밖에 생성합니다.
두 스캔의 위치 오프셋은 모델 기준 고정값입니다. 실차에 센서를 다르게 달면
해당 오프셋과 frame convention을 수정해야 합니다.

## 카메라 보정과 재학습

시뮬레이션 기본값은 SDF의 실제 카메라 장착 위치와 방향을 읽고,
내부 보정값은 `CameraInfo`를 사용합니다. 실차에서는 아래 JSON을 예시로 내보내고
`geometry`, 전후방 `rotation`, `translation`, `initial_pose`를 실측값으로 교체합니다.
`rotation`은 optical frame → 해당 차체 뒤축 좌표계 변환입니다.

```bash
python3 tools/inspect_parking_model.py --start-index 1 \
  --output artifacts/parking/calibration.json
```

전후방 영상 모두에서 데이터셋을 확보해야 합니다. 단일 차량 전방 학습 결과를
트레일러 후방에 그대로 적용했을 때도 잘 된다고 가정하지 않습니다.

```bash
python tools/extract_parking_frames.py --video front.mp4 \
  --camera front --output artifacts/dataset/images --every 15
python tools/extract_parking_frames.py --video rear.mp4 \
  --camera rear --output artifacts/dataset/images --every 15
```

이미지마다 YOLO **segmentation polygon** 라벨을 만듭니다.
한 줄 형식은 `class_id x1 y1 x2 y2 ...`이고 좌표는 이미지 크기로 나눈 0~1 값입니다.
주차칸은 바닥의 빈 공간, 차는 해당 클래스의 실제 보이는 영역을 라벨링합니다.
전진/후진, 좌/우 칸, 가림, 조명 변화, 시작점 네 개, 여러 연결각을 포함합니다.
같은 영상의 이웃 프레임이 학습·평가에 섞이지 않도록 **주행 세션 단위**로 나눕니다.

`src/trailer_parking_pkg/config/parking_dataset.yaml`을 복사해 데이터 위치를 지정합니다.

```bash
python tools/train_parking_yolo.py --data /path/to/parking_dataset.yaml \
  --weights best.pt --epochs 100 --device 0
python tools/train_parking_yolo.py --data /path/to/parking_dataset.yaml \
  --weights runs/parking/front_rear_segment/weights/best.pt --validate-only --device 0
```

현재 저장소에 전후방 주차용 정답 라벨 세트가 없어 이번 작업에서 새 학습을 실행하지
않았습니다. 학습/평가 실행 코드와 데이터 형식만 제공하며 학습 완료 모델이나 mAP를
새로 만들었다고 주장하지 않습니다.

## 정지 조건과 현재 한계

- 두 영상 중 하나라도 오래되거나 영상 이동 추정이 실패하면 정지합니다.
- 영상 간 시각 차이, 연결부 위치 불일치, LiDAR 미수신/잘못된 거리값을 검사합니다.
- 여러 프레임에서 안정적으로 검출한 충분히 큰 빈 칸만 목표로 고정합니다.
- YOLO가 주차칸을 본 경우에만 제한된 거리의 저속 접근을 허용합니다.
- 별도 명령 감시 노드가 제어 명령 0.25초 이상 미수신 시 0속도를 발행합니다.
- 주차 완료는 두 차체 전체가 칸 안에 있고 위치·방향·연결각이 맞는지 확인합니다.
- 평평한 바닥, 정확한 보정, 충분한 지면 무늬가 필요합니다. 누적 영상 이동 추정에는
  드리프트가 있으며 글로벌 재위치 추정/루프 폐쇄는 아직 없습니다.
- 카메라 전용의 가려진 차량 깊이는 알려진 차량 크기로 보수적으로 추정합니다.
  일반적인 3D 장애물 인식이나 카메라 사각지대 전체의 안전을 보장하지 않습니다.
- LiDAR 버전의 장애물 융합은 두 센서의 점유 영역을 합치는 방식입니다.
  확률적 깊이 융합이나 LiDAR 기반 위치 추정은 구현하지 않았습니다.
- 정적 주차 차량은 시야에서 사라져도 보수적으로 지도에 남깁니다. 이동 장애물이나
  잘못된 검출이 지도에 오래 남으면 정지/재시작이 필요할 수 있습니다.
- 실제 범퍼/타이어 접촉, 마찰, 구동 PID, 영상 인식 실패를 포함한 Gazebo 전체 검증과
  실제 유아차 치수로의 교체·조향 부호·제동거리 검증은 별도 작업이 남아 있습니다.

## 테스트

```bash
python3 -m pytest src/trailer_parking_pkg/test -q
python3 tools/parking_benchmark.py --x 9 --y -4 --length 11.094831 --timeout 60
```

수치 실험은 이상적인 저속 모델/정확한 위치 입력을 사용하는 제어기 검증이며,
Gazebo 또는 YOLO 인식 성능 검증이 아닙니다. `--length`를 바꾸는 것은 검증용
가상 공간 크기 변경이고 원본 월드나 주차 차량 배치를 바꾸지 않습니다.
최종 검증 결과는 `docs/parking_validation.md`에 기록합니다.
