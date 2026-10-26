# EBIMU-9DOFV6 독립 UART / RViz 진단

현재 검증 단계: **10% — CP2102와 실제 IMU의 물리 연결 확인 대기**.
분석·진단 도구 작성과 실제 하드웨어 성공률은 별개다. 현재 UART는 열지 않았다.
과거 실측 로그의 재파싱 및 입력 없는 RViz 상태만 시험했다. 실제 회전 성공을 의미하지 않는다.

## 실행 전 조건

- `/dev/ttyUSB0` / CP2102 serial `0001`이 실제 EBIMU에 연결됐음을 먼저 확인한다.
  현재 VID:PID는 `10c4:ea60`이며 이 값만으로 LiDAR와 구분할 수 없다.
- LiDAR 포트를 열지 않는다. 포트 소유자는 한 프로그램뿐이어야 한다.
- 새 도구는 센서에 명령을 전송하지 않는다. DTR/RTS를 low로 요청해 포트를 열지만,
  드라이버의 open 시 모뎀 신호 변동 및 초기 RX 버퍼 비움 가능성은 있다.
- 물리 장치를 구분하지 못하면, 사용자가 동의한 뒤 해당 USB 하나의 분리 전/후 목록을 비교한다.
  배선을 분리하거나 바꿀 필요는 없다. 이번 작업에서 분리·변경은 수행하지 않았다.
- 현재 기본 파싱 형식은 **오늘 15:32의 과거 실제 캡처**에 근거한
  `qz,qy,qx,qw,gx,gy,gz,ax,ay,az,timestamp_ms`이다.
  실제 현재 패킷 형식과 일치하는지 반드시 probe에서 확인한다.
- `--accel`은 `soa1`의 body 기준 중력 포함 가속도만 의미한다.
  global/linear acceleration 또는 velocity 출력을 같은 형식으로 취급하면 안 된다.
- 기본 설정과 다른 필드가 나오면 실행을 멈추고 조사한다. 임의 설정 명령을 보내지 않는다.
  Euler 단독 출력이 실제 확인되면 `--orientation euler --no-gyro --no-accel --no-timestamp`를 쓴다.
  Binary 데이터는 원시 저장만 가능하고 이 도구의 ROS 파서는 지원하지 않는다.

## 1. ROS 환경과 포트 확인

아래 환경 설정을 **각 터미널에서** 실행한다. 기존 workspace overlay는 필요 없다.
ROS domain 86을 독립 진단에 사용하며, 기존 domain 0의 제어·SLAM에 연결하지 않는다.
이미 domain 86을 쓰는 프로그램이 있다면 진단용 다른 domain을 모든 터미널에 동일하게 지정한다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=86
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
cd /home/venezuelachocolatemilk/mvsim_slam_real/verification/ebimu9dofv6_rviz
export ROS_LOG_DIR="$PWD/evidence/ros_logs"
ls -l /dev/ttyUSB* /dev/serial/by-id/* /dev/serial/by-path/*
udevadm info --query=property --name=/dev/ttyUSB0
fuser -v /dev/ttyUSB0
```

`fuser`에 PID가 나오면 사용 중인 프로그램을 먼저 확인한다. 임의 종료하지 않는다.
이 계정은 이미 dialout 그룹이다. `chmod 666`, udev 변경, 드라이버 설치는 필요 없다.

## 2. 실제 UART 수신 (물리 장치 확정 후에만)

```bash
/usr/bin/python3 scripts/serial_probe.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --confirmed-imu-port --seconds 10 \
  --output "evidence/passive_$(date +%Y%m%d_%H%M%S)"
```

`summary.json`의 `opened`, `rx_bytes`, `packets`, `rejected`, `packet_rate_hz`를 구분한다.
RX=0이면 하드웨어 수신 실패다. `opened=true`만으로 성공이 아니다.
RX>0인데 packets=0이면 출력 형식·통신 설정을 조사한다. 다음 단계를 성공으로 진행하지 않는다.
초당 RX, 패킷 길이 분포, 원시 hex, 원시 bytes를 함께 보존한다.
ASCII에는 제조사 정의 CRC/checksum이 없으므로 checksum 합격을 주장하지 않는다.
매 실행은 새 결과 디렉터리를 요구한다.

## 3. 실제 데이터 Publisher

probe에서 실제 유효 패킷을 확인하고 probe가 종료된 후 실행한다.

```bash
/usr/bin/python3 scripts/imu_publisher.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --confirmed-imu-port \
  --output "evidence/publisher_$(date +%Y%m%d_%H%M%S)"
```

발행: `/ebimu_diag/tractor/imu/data` (`sensor_msgs/msg/Imu`),
`/ebimu_diag/tractor/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`).
frame: `ebimu_diag_tractor_sensor`. 실제 패킷당 한 메시지를 발행한다.
수신이 끊기면 IMU 메시지를 새로 만들지 않는다. 마지막 Quaternion을 재발행하지 않는다.

Quaternion은 z,y,x,w 순서를 ROS x,y,z,w로 옮겨 정규화한다.
각속도는 DPS→rad/s, 가속도는 g→m/s² (표준중력 9.80665 사용)로 환산한다.
없는 각속도·가속도는 covariance[0]=-1이고, 수신된 항목의 공분산은 실측하지 않아 0(unknown)이다.
시각은 host parsing/publication 시각이며 센서와 동기화된 시간이나 실제 end-to-end 지연이 아니다.
같은 read에 여러 패킷이 들어오면 호스트 발행 시각이 몰릴 수 있다. 센서 ms 간격도 로그에 남긴다.
센서 timestamp 역행은 reset/rollover 가능성이 있어 별도 집계하며 누락과 구분한다.

```bash
ros2 topic info /ebimu_diag/tractor/imu/data --verbose
ros2 topic echo /ebimu_diag/tractor/imu/data --once --qos-reliability best_effort
ros2 topic hz /ebimu_diag/tractor/imu/data
```

## 4. RViz (다른 터미널 두 개)

환경 설정을 각 터미널에 적용한 뒤 실행한다.

```bash
# 터미널 B: Publisher와 독립적인 수신 중단 감시
/usr/bin/python3 scripts/imu_visualizer.py \
  --output "evidence/visualizer_$(date +%Y%m%d_%H%M%S)"
```

```bash
# 터미널 C
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d /home/venezuelachocolatemilk/mvsim_slam_real/verification/ebimu9dofv6_rviz/imu_rviz_live.rviz
```

Fixed Frame: `ebimu_diag_reference`. 이 frame은 제조사 AHRS native Quaternion을
ROS Quaternion 수학 관례로 표시하기 위한 독립 기준이다. map, odom 또는 지리 ENU가 아니다.
장착 회전·절대 방위·제조사 Quaternion의 물리 회전 의미는 실제 동작 시험이 필요하다.
초기 영점 설정, yaw 적분, ENU 자동 보정은 하지 않는다. 화면 RPY는 Quaternion에서 계산한 값이며
센서의 Euler 출력과 동시에 비교해 검증한 값이 아니다. 제조사 9·10쪽의 Euler/gyro 축 도면도 다르므로
단순 부호 반전이나 ENU 변환을 임의 적용하지 않았다.

유효한 orientation이 있어야 직육면체와 X=빨강/Y=초록/Z=파랑 화살표가 나타난다.
유효 데이터 전에는 NO DATA만 표시한다. 마지막 유효 데이터가 0.5초 이상 오래되면
STALE, 경과 ms, LAST POSE - NOT LIVE와 회색 본체·흐린 축을 표시한다.
Publisher를 꺼도 visualizer는 실행 상태여야 이 감시가 동작한다.
visualizer까지 종료되면 Marker는 1초 후 만료한다.
TF를 발행하지 않으므로 RViz Global Status에 No TF 경고가 보일 수 있으나,
Marker는 Fixed Frame 자체 좌표이므로 이 경고만으로 수신 실패를 판단하지 않는다.

## 5. 사용자와 한 단계씩 실제 동작 시험

현재 이 시험들은 수행하지 않았다. Publisher·visualizer·RViz를 켠 상태에서 진행한다.
한 시험 결과를 확인한 뒤 다음 시험으로 이동한다. 손으로 센서만 움직이며 차량을 작동시키지 않는다.
다음 명령의 test는 시험별로 stationary, roll, pitch, yaw, return 중 하나다.

```bash
/usr/bin/python3 scripts/record_motion.py --test stationary --seconds 15 \
  --output "evidence/stationary_$(date +%Y%m%d_%H%M%S)"
```

1. 정지: 평평한 표면에서 15초. RPY span/stddev, gyro stddev, 실제 topic Hz와 RViz 상태를 기록한다.
2. Roll: 제조사 축 도면에 맞춰 30~45도 기울였다 유지 후 복귀한다. `--test roll`로 기록한다.
3. Pitch: 같은 방식으로 `--test pitch`를 기록한다.
4. Yaw: 수평을 유지해 수직축 기준 약 90도 회전·유지·복귀한다. `--test yaw`로 기록한다.
5. 각 시험에서 실제 축, 회전 부호, 모델 방향, 복귀를 사용자가 확인한다.
   출력 수치만으로 물리 방향 일치 또는 절대 각도 정확도를 합격 처리하지 않는다.
   span은 왕복 운동의 크기, end-minus-start는 복귀 잔차를 참고할 수 있다.
   기준 각도 계측 없이 제조사 정확도 수치를 실측 정확도로 쓰지 않는다.
6. 중단: Publisher 터미널에서 Ctrl+C. visualizer·RViz는 유지하고 0.5초 이후 STALE을 확인한다.
   `visualizer_.../status.jsonl`에 LIVE→STALE이 남아야 한다. USB나 배선 변경 없이 가능하다.
   처음부터 데이터가 없으면 NO DATA이며 STALE 전환 시험을 했다고 보고하지 않는다.

각 측정의 summary는 `MEASURED_AWAITING_PHYSICAL_RVIZ_OBSERVATION`이며 자동 합격이 아니다.
정지 안정성과 절대 정확도는 다른 측정이다. Yaw는 주변 금속·전자장비 영향도 따로 조사한다.

## 종료 / 확장

각 터미널 Ctrl+C로 종료한다. 자신이 실행한 프로세스만 종료한다.
probe와 Publisher를 동시에 실행하지 않는다. 포트 점유 검사를 통과해도 다른 프로그램의 동시 open
경쟁을 완전히 배제할 수는 없으므로 실행 중인 시리얼 프로그램을 함께 확인한다.

두 번째 실제 센서가 확인된 이후에만 `--namespace /ebimu_diag/trailer`,
`--frame ebimu_diag_trailer_sensor` (visualizer는 `--sensor-frame`)를 지정해 별도 실행할 수 있다.
RViz에는 별도 MarkerArray 표시를 추가해야 한다. 현재 두 번째 센서는 생성·검증하지 않았다.
장착축과 공통 yaw 기준이 확인되기 전 yaw 차이를 articulation 또는 jackknife 측정으로 쓰지 않는다.

상세 결과: `reports/IMU_RVIZ_FINAL_REPORT.txt`.
