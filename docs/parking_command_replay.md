# 인식한 주차 공간에서 계획을 저장하고 명령만 재생하기

이 기능은 **정지 상태의 인식 결과 → CPU 경로계획 → 실차 명령 파일 → 미리보기/재생**을 분리합니다.
MVSim, Gazebo, GPU, SLAM을 실행하지 않고도 저장된 인식 결과에서 계획과 명령 파일을 만들 수 있습니다.
명령 파일을 만드는 데 실차를 먼저 주행시키거나 주행 중 명령을 녹화할 필요가 없습니다.

## 차량과 명령 계약

사용자가 지정한 차량은 [대호토이즈 벤츠 NEW GTR AMG](https://www.daehotoys.com/product/detail.html?product_no=854)입니다.
제조사의 [사용 설명서](https://daehotoys.godohosting.com/new_manual/benz_new_gtr.pdf)는 제품 식별 자료이며,
개조된 아두이노의 조향각/PWM 보정 자료로 사용하지 않습니다.

구동 계약은 저장소의 `src/control/driving/driving.ino`를 따릅니다.

- 앞 조향 모터는 A2 포텐셔미터를 읽고 조향 목표 단계 -7~7을 맞춥니다. -7은 좌, +7은 우입니다.
- 좌우 구동 모터는 각각 부호 있는 PWM -255~255를 받습니다. 음수는 후진입니다.
- 아두이노에는 115200bps로 `s<조향>l<좌PWM>r<우PWM>\n`을 20Hz 전송합니다.
- 새 조향 목표와 전후진 전환은 구동륜을 정지한 상태에서 보정표의 대기시간을 둡니다.
- 저장소 펌웨어의 300ms 수신 제한은 구동륜을 멈춥니다. 통신 두절만으로 조향 모터도 꺼지는 것은 아닙니다.

좌우 PWM 차이로 회전을 만드는 differential-drive 모델은 사용하지 않습니다.
`drive_samples`의 좌우 PWM 쌍은 같은 주행 속도를 위한 측정된 출력 쌍입니다.
모터 편차 보정이 필요하면 측정된 서로 다른 PWM 쌍을 넣을 수 있지만,
그 PWM 차이에 따른 추가 회전은 현 운동학 모델이 따로 계산하지 않습니다.

## 실차 보정표

`src/trailer_parking_pkg/config/daeho_gtr_amg.template.json`은 해당 제품명을 넣은 **미측정 템플릿**입니다.
`null`은 실측값으로 채워야 합니다. 제품명만으로 다음 값을 결정할 수 없습니다.

| 항목 | 값의 의미 |
|---|---|
| `geometry.wheelbase` | 견인차 앞축~뒤축 거리(m) |
| `hitch_offset` | 견인차 뒤축에서 뒤쪽 히치까지 거리(m) |
| `trailer_axle` | 히치~트레일러의 모델상 유효 뒤축 거리(m) |
| `front`, `rear`, `width` | 뒤축 기준 차체 앞/뒤 길이와 충돌 검사 너비(m) |
| `max_steer`, `max_beta` | 실제 허용 조향각·연결각(rad) |
| `steering_radians` | -7~7 각 단계에서 측정한 등가 앞바퀴 조향각(rad). 좌회전은 양수 |
| `drive_samples` | 전진/후진 각각 측정한 `speed_mps`, `left_pwm`, `right_pwm` |
| `steering_settle_s` | 정지 중 조향 목표가 바뀌었을 때 필요한 대기시간(s) |
| `gear_settle_s` | 전후진 전환 정지 대기시간(s) |

모든 값을 실차에서 확인한 프로파일만 `calibration_status: "measured"`로 표시합니다.
값이 없는 템플릿과 `example` 보정표로 하드웨어 전송은 할 수 없습니다.
이 표시 자체가 보정의 정확성을 자동으로 입증하는 것은 아닙니다.

현재 운동학은 **앞바퀴 조향 견인차 + 단일 유효 축을 가진 수동 트레일러**이며,
두 차체에 같은 `front/rear/width` 충돌 외곽을 사용합니다.
실제 뒤차의 앞바퀴 고정/자유 회전 상태, 차축 구성, 두 차체의 크기가 이 가정과 다르면
모델을 먼저 수정해야 합니다. 이 프로파일은 다른 차축 구조를 자동 지원하지 않습니다.

## 1. 차량을 움직이지 않고 인식 결과 저장

실제 전후방 카메라 토픽과 카메라 내부/외부 보정 및 초기 자세가 준비된 경우:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select interfaces_pkg trailer_parking_pkg
source install/setup.bash
ros2 launch trailer_parking_pkg capture_parking_space.launch.py \
  weights:=/absolute/path/best.pt \
  calibration:=/absolute/path/real_camera_calibration.json \
  profile:=/absolute/path/filled_vehicle_profile.json \
  output:=/absolute/path/parking_snapshot.json
```

이 launch는 CPU YOLO와 인식 결과 수집 노드만 시작합니다.
시뮬레이터·시리얼 송신 노드·자동 접근 주행은 실행하지 않습니다.
카메라 드라이버는 별도로 필요합니다. 안정적으로 5프레임 인식된 빈 칸이 보이면
차량 초기 자세, 칸, 장애물, 차체 치수를 저장하고 `SNAPSHOT_SAVED` 상태가 됩니다.
원하는 칸이 현재 시야에 없으면 움직여 찾지 않고 대기합니다.
CPU 추론이 늦어 입력 시각/유효성 검사를 통과하지 못하면 저장을 보류합니다.
정확한 카메라 보정 없이 YOLO 마스크만으로 거리나 연결각이 확정되는 것은 아닙니다.

좌표 원점은 견인차 뒤축, +x 전방, +y 좌측이며 `beta = trailer_yaw - tractor_yaw`입니다.
저장 파일과 프로파일의 `geometry`가 다르면 계획을 거부합니다.

## 2. GPU·시뮬레이터 없이 계획과 명령 만들기

```bash
python3 tools/parking_replay.py plan \
  --snapshot /absolute/path/parking_snapshot.json \
  --profile /absolute/path/filled_vehicle_profile.json \
  --output artifacts/parking/commands.json
python3 tools/parking_replay.py preview artifacts/parking/commands.json
```

경로계획은 보정표에 있는 **실제로 명령 가능한 조향각만** 탐색합니다.
연속 조향각 경로를 나중에 임의로 반올림하지 않습니다.
주행속도도 해당 진행 방향에서 측정된 PWM/속도 표 중 요청 속도에 가장 가까운 행을 고릅니다.
계획 이동거리를 그 속도로 나눠 50ms 명령 틱으로 변환하고, 틱 반올림 잔차를 누적 보정합니다.
정지 대기 중에는 모델 위치를 진행시키지 않습니다.

내보낸 명령을 다시 적분해 차체 전체의 칸 진입, 연결각 제한, 장애물 충돌을 확인합니다.
유효한 경로가 없거나 변환된 명령이 모델상 목표를 만족하지 않으면 파일을 내보내지 않습니다.
`prediction`은 운동학 모델 결과이며 실측 또는 실차 주차 완료 판정이 아닙니다.
명령 파일에는 원본 인식 정보, 보정표, 계획 경로와 재생 명령을 함께 저장합니다.

| 파일 필드 | 내용 |
|---|---|
| `snapshot` | 인식 당시 초기 자세, 칸, 장애물, 차량 치수와 출처 |
| `profile` | 명령 생성에 사용한 실차 보정표 전체 |
| `planned_path` | 경로 지점별 차량 상태, 전후진 방향, 조향각 |
| `dt_s` | 명령 주기 0.05초 |
| `frames` | 동일 명령을 묶은 `ticks`, `steering`, `left_pwm`, `right_pwm` |
| `prediction` | 명령 재적분에 따른 모델상 종료 상태와 예상 동작 시간 |

`ticks: 20`은 해당 명령을 20Hz로 1초 동안 반복 전송한다는 뜻입니다.
이는 주차 공간에 따라 계산한 명령의 저장 결과이며, 고정 조향 시간표를 사람이 작성하는 방식이 아닙니다.

소프트웨어 예시만 확인하려면 다음을 실행합니다. **두 입력 파일은 합성 예시이며 실차 측정값이 아닙니다.**

```bash
python3 tools/parking_replay.py plan \
  --snapshot src/trailer_parking_pkg/config/parking_snapshot.example.json \
  --profile src/trailer_parking_pkg/config/ride_on_platform.example.json \
  --output artifacts/parking/replay_example.json
python3 tools/parking_replay.py replay artifacts/parking/replay_example.json
```

`replay`도 기본값은 미리보기입니다. `serial` 모듈을 가져오거나 포트를 열지 않습니다.

## 3. 명시적인 하드웨어 재생

구현된 전송 옵션은 `replay <명령파일> --execute --port /dev/ttyACM0`입니다.
이번 작업에서는 이 옵션을 실행하지 않았고, 차량 모터 명령도 보내지 않았습니다.
측정 보정표가 있는 파일만 전송할 수 있고, 포트를 열기 전에 전체 명령을 다시 검증합니다.
실행 시 CPU 스케줄이 150ms 이상 밀리면 뒤늦은 명령을 몰아서 보내지 않고 정지를 요청합니다.
종료·중단·전송 실패에도 정지 명령을 시도합니다. 다른 명령 송신 노드와 동시 실행하지 않습니다.

재생 중에는 카메라·라이다·SLAM·IMU를 구독하지 않습니다.
따라서 시작 자세와 연결각을 저장 당시 조건에 맞춰야 하며, 새 장애물이나 위치 오차는 자동 보정하지 못합니다.
실제 배터리 전압, 하중, 바닥, 가속/감속, 조향 지연이 보정 조건과 달라지면 이동 결과도 달라집니다.
MVSim 모델과 실차가 많이 다르면 같은 시간 명령이 같은 경로를 만들지 않는다는 점은 그대로 남습니다.
이 기능은 MVSim의 PWM/속도 이력을 복사하지 않고 실차 보정표로 계획을 다시 계산하지만,
저속 운동학이 표현하지 않는 미끄러짐·동역학 차이까지 없애지는 않습니다.

## 검증 범위

CPU 단위/통합 테스트: 조향 단계 제한, 별도 후진 보정, 곡선 경로와 전후진 계획,
방향 전환 정지, 변환 후 충돌/목표 검사, 잘못된 파일 거부, 미리보기의 무포트 동작,
가짜 시계/시리얼을 사용한 20Hz 송신 및 지연 시 정지.
실차 보정, 실제 카메라 인식 저장, 모터 재생 및 실제 주차 성공은 별도 확인 대상입니다.

2026-11-06 검증에서는 관련 테스트 **45개가 통과**했습니다.
합성 공간 예시로 조향·전진·후진이 포함된 **14구간**, 모델상 총 **43.15초** 명령 파일을 생성했습니다.
이 시간은 예상 재생 시간이며 CPU 계산 시간이나 실제 차량 주행 시간이 아닙니다.
예시 파일은 `example` 보정표라 하드웨어 전송이 거부됩니다.

빌드된 ROS 메시지를 사용할 수 있는 환경에서 수행한 검증 명령:

```bash
PYTHONPATH="$PWD/src/simulation_pkg:$PWD/src/trailer_parking_pkg:$PWD/src/serial_communication_pkg:$PYTHONPATH" \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/trailer_parking_pkg/test \
  src/serial_communication_pkg/test/test_protocol.py \
  src/simulation_pkg/test/test_parking_command_adapter.py -q
```

ROS 테스트는 메시지·노드 계약 확인이며 시뮬레이터나 차량 송신 노드를 실행하지 않습니다.
테스트 당시 기존 SciPy/NumPy 버전 조합 경고가 1건 있었지만 45개 테스트는 통과했습니다.
새 보정표 기반 계획 경로는 NumPy/SciPy/ROS/GPU/serial 모듈 없이 수행되는 테스트도 포함합니다.
USB 연결 확인과 라이다의 미해결 이상 보고는 [하드웨어 연결 기록](hardware_connection_notes.md)에 남겼습니다.
