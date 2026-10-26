# 시작 자세 기준 좌우 회전량

차량 두 대를 같은 방향으로 일자 정렬하고 정지시킨 뒤 실행합니다.
기존 G6·Cartographer·IMU·시각화 실행은 종료해야 합니다.

```bash
cd ~/mvsim_slam_real
bash tools/run_relative_yaw.sh
```

이 명령은 G6, LiDAR 단독 Cartographer, 두 IMU 수신, 센서 TF, 2D RViz를 실행합니다.
RViz는 시작 전방(map +X)이 화면 위쪽을 향하도록 표시합니다. 카메라만 회전하며 SLAM 좌표와 센서 TF는 그대로 유지합니다.
실행 직후 준비 중에는 두 차체가 일자로 평행한 회색 시작 자세를 표시합니다.
이 표시는 시작 자세 가정이며 실제 센서 측정·구동 준비 완료를 의미하지 않습니다.
모터·Arduino 송신기·자율주차 제어기는 실행하지 않습니다.
장치명은 `/dev/lidar_g6`, `/dev/imu_tractor`, `/dev/imu_trailer`를 사용합니다.

1. RViz에 WAIT / HOLD STILL이 표시되는 동안 일자 정렬을 유지합니다.
2. 신선한 스캔(최근 0.5초, 최근 1초에 5개 이상), 지도(최근 3초), SLAM TF,
   두 IMU와 시각이 맞는 샘플이 확보되어야 합니다.
3. 기본 5초 동안 각 IMU의 전체 각속도 크기가 2°/s 이하이고,
   IMU·SLAM 방향 변화가 1° 이내, SLAM 위치 변화가 2cm 이내여야 합니다.
   움직이거나 입력이 끊기면 대기 시간을 다시 채웁니다.
4. 안정 구간을 확보한 마지막 동시 IMU 샘플을 각각의 회전량 0° 기준으로 저장합니다.
   첫 READY 화면에서는 두 차체의 회전량과 꺾임각이 정확히 0°이며 서로 평행합니다.
   공통 지도 방향은 안정 구간의 SLAM 방향 평균을 사용합니다.
5. READY가 표시되면 수동으로 움직이며 각 차체의 초기 대비 회전량과 꺾임각을 확인합니다.

수신이 시작되는 시간까지 포함하면 전체 준비 시간은 5초보다 깁니다.
Cartographer는 IMU를 구독하지 않습니다. 지도·위치는 LiDAR SLAM에서 얻고,
RViz의 차체 방향은 `시작 지도 방향 + 각 IMU의 초기 대비 yaw 변화량`입니다.
높이·roll·pitch 변화는 화면의 차체 회전으로 표시하지 않습니다.
±180° 경계는 연속적으로 풀어서 누적 좌우 변화량을 계산합니다.
트레일러는 장착 yaw=180°를 반영합니다. 180° 장착은 yaw 변화 부호를 반대로 만드는 것이 아닙니다.

두 센서의 절대 yaw·방위각·초기 숫자는 서로 같을 필요가 없습니다.
단순 gyro 적분을 사용하지 않으므로 이 프로그램이 적분 bias를 누적하지는 않지만,
센서가 출력하는 자세의 드리프트·자기장 영향·장착 기울기 영향까지 제거하는 것은 아닙니다.
기존 진단 sign=-1을 두 센서에 사용하며 실제 좌·우 방향과 각도 정확도는 추가 수동 회전 검증이 필요합니다.
이전 기록의 수정후 CW/CCW 시험은 최종 확정되지 않았습니다.

확인용 토픽:

```bash
source /opt/ros/humble/setup.bash
source ~/mvsim_slam_real/install/setup.bash
ros2 topic echo /vehicle_viz_2d/status
ros2 topic echo /vehicle_viz_2d/ready
```

status의 `degrees.tractor_delta`, `degrees.trailer_delta`는 각자의 시작 대비 회전량,
`degrees.phi`는 `트레일러 변화량 - 트랙터 변화량`을 ±180°로 표시한 상대 꺾임각입니다.
`ready=true`는 센서 표시를 시작할 준비가 되었다는 뜻이며 차량 구동 허가는 아닙니다.

수동조작 실행:

카메라 없이 WASD로 실제 수동조작과 SLAM을 함께 실행하려면 아래 명령을 사용합니다.
원본 `ros2_parking` 데이터 수집기와 같이 `W/S`는 PWM을 ±10씩(한계 ±250),
`A/D`는 조향을 ±1단계씩(한계 ±7) 바꿉니다. 입력 주기는 0.1초이며 키를 떼어도 값을 유지합니다.
`R`은 속도·조향 초기화, `F`는 초기화 후 종료입니다. 동시 입력은 `W > S > A > D > R > C > F` 우선순위입니다.
카메라와 C 촬영 기능은 없습니다. SLAM/READY와 수동 명령은 독립이며 Arduino에 직접 전송합니다.
시작 전에 실제 두 차체도 같은 방향으로 일자 정렬하고 정지시킵니다.
조작 창을 닫거나 F를 누르면 SLAM·RViz·IMU·모터 연결도 함께 종료합니다.

```bash
bash tools/run_manual_slam_wasd.sh
```

장치 경로를 바꾸려면 `LIDAR_PORT`, `TRACTOR_IMU_PORT`, `TRAILER_IMU_PORT`, `ARDUINO_PORT`
환경변수를 지정합니다. 실행 중인 다른 센서/제어 프로세스가 있으면 먼저 종료합니다.

기존 터미널 키보드 도구를 진단 용도로 사용할 경우:

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/manual_slam_check.py --require-ready
```

이 옵션은 READY가 false이거나 0.3초 이상 수신되지 않으면 속도 0만 발행합니다.
실차 제어 연결 조건은 `MANUAL_SLAM_CHECK.md`와 기존 controller 설정을 따릅니다.

센서가 끊기거나 비정상 각도 점프(기본 120°/s 초과)가 생기면 준비가 해제됩니다.
기준이 소실되면 주행 중 자동으로 0°를 다시 잡지 않습니다. 정지 후 두 차체를 같은 방향으로
정렬하고 프로세스를 재시작하거나 다음으로 기준을 비운 뒤 5초 안정화를 다시 기다립니다.

```bash
ros2 service call /vehicle_viz_2d/align_straight std_srvs/srv/SetBool '{data: false}'
```

이 기능은 `relative_yaw.launch.py`의 관찰용 경로에 적용됩니다.
기존 자율주차 상태추정기·LQR의 방향 입력이나 구동 확인 플래그는 변경하지 않습니다.
