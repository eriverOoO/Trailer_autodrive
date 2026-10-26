# 수동 주행 SLAM 확인

기본 수동조작은 카메라 없는 WASD 창입니다. 기존 센서·제어 프로세스를 종료한 뒤 실행합니다.

```bash
bash tools/run_manual_slam_wasd.sh
```

`parking_data_control.py`는 `ros2_parking/src/data_collection/data_collection.py`와
옆의 `Data_Collect.process` 바이트코드에서 확인한 조작 방식을 사용합니다.
카메라 연결·C 촬영만 제외하고, 0.1초 입력 주기·증감·유지·우선순위를 원본과 맞췄습니다.

- W: 양쪽 PWM +10, 최대 +250. S: 양쪽 PWM -10, 최소 -250.
- A: 조향 -1단계, 최소 -7. D: 조향 +1단계, 최대 +7.
- R: 속도 0·조향 0. F: 속도 0·조향 0 후 전체 종료.
- 키를 떼어도 현재 속도·조향을 유지합니다. 창 포커스와 무관하게 키를 감지합니다.
- 원본과 같은 `W > S > A > D > R > C > F` 우선순위이므로 여러 키를 누르면 앞선 키만 적용합니다.
- C는 촬영 없이 값을 유지합니다. 창의 R 버튼으로도 즉시 초기화할 수 있습니다.

시작 RViz는 두 차체를 일자로 평행하게 표시합니다. 회색 WAIT 표시는 시작 자세 가정이며,
5초 안정화 뒤 첫 READY의 양쪽 회전량·꺾임각은 정확히 0°입니다.
그 이후에는 각 IMU의 시작 대비 변화량을 표시합니다. 실제 차체도 일자로 정렬하고 시작해야 합니다.
상세 준비 조건은 [초기 회전량 안내](RELATIVE_YAW.md)를 참조하세요.

조작 창이 115200 baud로 Arduino에 `s{steering}l{left_speed}r{right_speed}\n`을 직접 보냅니다.
SLAM·READY는 관찰용이며 수동 명령을 제한하지 않습니다. 이전의 고정 PWM·조향·키 해제 정지 방식과
`manual_motor_bridge`는 이 실행 경로에서 사용하지 않습니다. 창에는 송신값을 표시하며 보드 수신 확인값은 아닙니다.
기존 제어기·시리얼 송신기·다른 수동조작 도구와 동시에 실행하지 않습니다.
X11 키 상태를 조회하므로 root 권한·카메라·`keyboard` Python 패키지 없이 동작합니다.

원본 메서드를 가짜 키·시각·캡처 객체로 호출해 128가지 키 조합을 세 번 순회하며 결과를 비교했습니다.
실제 조향 전송과 값 유지·종료 시 초기화는 가상 UART로 검증했습니다. 실제 장비는 이 변경 후 재가동하지 않았습니다.

아래는 기존 터미널 진단 도구 설명입니다.

`manual_slam_check.py`는 ROS 2에서 키보드 입력을 차량용 Twist로 바꾸고 실제 `/scan`, `/map`, `map → laser_frame` 상태를 관찰합니다. 센서·SLAM·Arduino 프로세스를 자동으로 시작하지 않습니다. 지도는 기존 RViz로 확인합니다.

```bash
cd ~/mvsim_slam_real
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/manual_slam_check.py
```

기본 명령 출력은 `/parking/manual_cmd_vel`입니다. 구독하는 제어기가 없는 상태에서 키보드 입력과 SLAM 관찰을 확인할 수 있습니다. 실제 차를 별도 리모컨으로 움직이거나 센서를 손으로 이동하면서 관찰하는 용도로도 사용할 수 있습니다.

초기 센서 안정화를 기다리는 새 실행 경로는 [초기 대비 좌우 회전량 안내](RELATIVE_YAW.md)를 참조하세요.
`--require-ready`를 추가하면 `/vehicle_viz_2d/ready`가 true이며 최근 0.3초 이내 수신되었을 때만 이동 명령을 냅니다.

- `q / w / e`: 전진하면서 좌 / 직진 / 우 조향
- `z / s / c`: 후진하면서 좌 / 직진 / 우 조향 (앞바퀴 기준)
- `Space` 또는 `x`: 정지
- `Q` 또는 `Esc`: 정지 명령을 보낸 뒤 종료

이동 키가 반복 입력될 때만 움직입니다. 기본 0.35초 동안 입력이 없으면 속도 0을 발행합니다. 운영체제의 첫 키 반복 지연 때문에 처음 누를 때 잠시 정지할 수 있습니다. 기본 속도는 전진 0.08m/s, 후진 0.06m/s입니다. `--speed`, `--reverse-speed`, `--steer-deg`, `--wheelbase`, `--key-timeout`으로 바꿀 수 있습니다. 기본 축간거리는 사용자가 실측한 0.55m입니다. Twist angular.z는 `v*tan(steering)/wheelbase`인 yaw rate입니다.

기존 Adapter에 연결하려면 다음처럼 입력 토픽을 지정합니다.

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/manual_slam_check.py \
  --command-topic /parking/request_cmd_vel --wheelbase 0.55
```

이 명령만으로 실차가 구동되지는 않습니다. 기존 `control_adapter`와 Arduino 송신기가 준비되어 있어야 하며, 기존 상태·IMU 신선도, 비상 정지, 실측·모터 변환 확인 조건을 그대로 적용합니다. `auto_parking` 등 같은 입력 토픽에 명령을 발행하는 노드는 함께 실행하지 않습니다. `g6_slam.launch.py`만 실행하면 차량 제어기와 `/parking/state`는 없으므로 이 경로로 실차를 움직일 수 없습니다. 기존 구동 준비 조건은 README의 센서·차량 설정을 참조하세요. 이 도구는 그 조건을 해제하거나 상태를 만들어내지 않습니다.

### ros2_parking 방식의 수동 PWM 연결

`manual_motor_bridge.py`는 `ros2_parking`의 `driving.ino` 및 기존 프로토콜 변환 함수를 사용해
115200 baud로 `s0l50r50\n` 같은 명령을 보냅니다. 자율주차 Adapter의 실측 확인 플래그와는 별개인
수동 PWM 시험 도구입니다. Twist의 속도 크기는 실측 m/s가 아니며, 전후진 부호만 사용해
고정 PWM(기본 50)을 선택합니다. 조향은 축간거리 0.55m, 최대각 0.45rad를 사용해 ±7 단계로 변환합니다.

먼저 SLAM/두 IMU/READY 실행 경로를 켜고, ROS 환경을 source한 별도 터미널에서 각각 실행합니다.
이 연결을 사용할 때 기존 Arduino 송신기·자율주차·Adapter는 함께 실행하지 않습니다.

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/manual_motor_bridge.py \
  --port /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_054433A4937351314608-if00 --pwm 50
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/manual_slam_check.py --require-ready --motor-pwm 50
```

명령이 0.25초 이상 끊기거나 READY·스캔이 오래되면 모터 PWM 0을 반복 전송합니다.
복수 수동 명령 발행기·다른 제어 명령 발행기가 있으면 정지합니다.
진행 방향 ±20°의 유효 스캔이 부족하거나 전방 1.2m/후방 1.5m 이내에 물체가 있으면 정지합니다.
이는 제한적인 스캔 검사이며 차체 전체의 충돌 방지를 보장하지 않습니다.
`/parking/emergency_stop`에 true를 보내면 재시작 전까지 정지를 유지합니다.
기존 Arduino의 수신 watchdog 부재는 그대로이므로 USB 단절·호스트 고장 시 정지는 보장하지 못합니다.

2026-10-24 실차 시험: PWM 50, 1초 전진 입력 후 PWM 0 전송, 초기 SLAM 진행 방향 변화 약 0.143m.
기록은 `log/manual_slam_20261024_front_up/forward_test.json`과 `motor_bridge.log`입니다.
이후 별도 3초 관찰에서 SLAM 위치가 약 22m 변하는 이상과 READY 해제가 관측되어
수동 비상 정지를 걸었습니다. 실제 차량 이동·정지는 사용자 관찰 확인이 필요하며,
초기 지도상 이동량을 검증된 실측 거리·속도로 사용하지 않습니다.

이미 실행 중인 시뮬레이터의 Twist 입력에 연결할 때는 `--command-topic /해당_입력`과 실제 `--wheelbase`를 지정합니다. 시뮬레이터가 `/clock`을 쓰면 `--ros-args -p use_sim_time:=true`를 붙입니다. 차체 좌표를 보고 싶으면 올바른 TF가 있는 경우 `--pose-frame base_footprint`를 사용합니다.

SLAM은 기존 G6 실행 명령으로 별도로 시작하고 `enable_rviz:=true`를 지정하세요. 시작 지점의 벽 모양을 확인한 뒤 천천히 직진·회전·후진하고 원래 위치로 돌아옵니다. RViz에서 벽이 이중으로 벌어지는지, 위치가 튀는지, 복귀 위치가 맞는지 확인합니다.

화면의 `RECEIVING`은 스캔·지도·스캔 시각 TF가 들어온다는 뜻이며, SLAM 정확도 합격 판정이 아닙니다. `match`는 Cartographer의 `/scan_matched_points2` 관찰값입니다. 기본 지도 신선도 기준은 3초이며, 지도 발행 주기가 더 길면 `WAIT / STALE`로 표시될 수 있습니다. 종료 시 최근 최대 10시간의 초당 관찰값이 `log/manual_slam_check.json`에 저장됩니다. `--output`으로 경로를 바꾸세요.

입력 중단·정상 종료 시 소프트웨어 정지 명령을 발행하지만, 기존 Arduino 펌웨어에는 수신 watchdog이 없어 프로세스 강제 종료·USB 단절 시 정지를 보장하지 않습니다.
