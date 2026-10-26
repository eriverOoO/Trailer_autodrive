# 트랙터 IMU 고정 이름 등록

현재 결과: 실제 트랙터 UART 수신 확인. **udev 설치는 전용 USB 포트 지정 대기 중**이다.

실측 2026-10-19 21:34 KST:

- CP2102 한 개, VID/PID `10c4:ea60`, serial `0001`, interface `00`.
- 현재 ID_PATH: `pci-0000:80:14.0-usb-0:8:1.0`.
- 115200bps, 5.00099초, RX37,481바이트, 정상499패킷, 오류0, 약99.99486Hz, TX0.
- 11필드 ASCII Quaternion/gyro/accel/timestamp 패킷. 시작할 때 걸친56바이트는 부분 패킷으로 버림.

`evidence/g6_identity_collision.json`을 확인하면 과거 실제 G6도 VID/PID/serial/제품명/interface뿐 아니라
**현재와 동일한 ID_PATH까지 사용했음**을 알 수 있다. 시리얼 번호가 고유하다고 주장할 수 없다.
EEPROM/USB serial을 바꾸거나 자동으로 G6 포트를 열어 패킷을 조사하는 udev PROGRAM은 사용하지 않는다.

## 선택과 제한

안전하게 구분할 고유 USB 정보가 현재 없으므로, 어느 USB 포트에서도 트랙터만 식별하는 등록은 할 수 없다.
물리 포트 규칙은 **역할을 전용 소켓에 배정**하는 방식이다. 지정 소켓에 G6를 꽂으면 G6도 매칭한다.
따라서 사용자가 이 소켓을 트랙터 전용으로 지정하고 G6/다른 CP2102를 꽂지 않는 조건을 확인해야 한다.
다른 소켓에서는 `/dev/imu_tractor`가 생성되지 않는 것이 이 방식의 정상 동작이다.
두 번째 IMU의 식별자는 아직 미검증이다.

후보 규칙: `99-ebimu-tractor.rules.candidate`. `.candidate`는 시스템 규칙 폴더에 설치되지 않았다.
현재 `device_selection.json`의 operator_confirmation은 PENDING이며 설치 프로그램도 실행을 거절한다.
다른 USB 소켓을 선택하면 실제 그 소켓으로 옮긴 후 UART와 ID_PATH를 다시 확인하고 후보를 갱신해야 한다.

## 조건 확인 후 설치

소켓 지정과 실제 연결 확인이 끝난 경우에만 device_selection.json을 갱신한 뒤 설치한다.
sudo 비밀번호는 **사용자의 정상 터미널**에서 입력한다. 코드/보고서/대화에 비밀번호를 적지 않는다.

```bash
cd /home/venezuelachocolatemilk/mvsim_slam_real/verification/imu_persistent_device_names
cat 99-ebimu-tractor.rules.candidate
sudo /usr/bin/python3 scripts/install_rule.py --dedicated-port-confirmed
```

설치 프로그램은 기존 같은 이름의 규칙 또는 imu_tractor 규칙/별칭이 있으면 중단한다.
확인한 장치 인스턴스와 ID_PATH가 달라졌거나 포트가 점유되어도 중단한다.
새 `/etc/udev/rules.d/99-ebimu-tractor.rules`를 O_EXCL로 만들고, 규칙 reload 및 해당 tty 한 개의
change 이벤트만 적용한다. 권한은 기존 dialout 0660을 유지하며 chmod666 등은 하지 않는다.
현재 sudo 무암호 인증은 불가능한 것으로 확인했다.

## 별칭·실제 수신 검증

```bash
ls -l /dev/imu_tractor
readlink -f /dev/imu_tractor
udevadm info -q property -n /dev/imu_tractor
```

설치가 된 다음에만 별칭으로 다음 수신 시험을 한다. 확인된 장치만, 점유 프로그램 없이 실행한다.

```bash
cd /home/venezuelachocolatemilk/mvsim_slam_real
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  verification/ebimu9dofv6_rviz/scripts/serial_probe.py \
  --port /dev/imu_tractor --confirmed-imu-port --seconds 5 \
  --output "verification/imu_persistent_device_names/evidence/alias_uart_$(date +%Y%m%d_%H%M%S)"
```

위 진단 파일 경로와 실행 인자는 실제 존재·실행을 확인했다. 이번 현재 UART 측정에도 사용했다.
센서 설정·캘리브레이션·EEPROM 명령을 보내지 않는다.
재연결 시험은 사용자가 실제 USB를 분리/재연결한 다음 확인해야 한다. 이번에는 아직 하지 않았다.
다른 USB 소켓 시험과 G6 동시 연결 시험도 아직 하지 않았다.

## 기존 ROS 2 드라이버 사용 예 (나중에 별칭 확인 후)

기존 파일은 변경하지 않는다. 실제 기존 launch에 `imu_port` 인자가 있다.

```bash
source /opt/ros/humble/setup.bash
source /home/venezuelachocolatemilk/mvsim_slam_real/install/setup.bash
ros2 launch mvsim_slam_real_bringup ebimu.launch.py imu_port:=/dev/imu_tractor
```

이 명령은 이번에 실제로 실행하지 않았다. 기존 launch/source/install 파일과 인자를 조사해 확인한 사용 예다.
원래 ROS 토픽 `/tractor/imu/data`를 발행하므로 시리얼 raw 진단과 동시에 실행하지 않는다.
`/dev/imu_trailer`는 이름만 예약하고 규칙/별칭을 만들지 않는다.

## 원복 (실제 설치했을 때만)

먼저 설치한 파일과 후보가 같은지 확인한다. 기존 타인의 파일을 삭제하지 않는다.

```bash
cd /home/venezuelachocolatemilk/mvsim_slam_real/verification/imu_persistent_device_names
sudo cmp -- 99-ebimu-tractor.rules.candidate /etc/udev/rules.d/99-ebimu-tractor.rules
```

위 비교가 성공하고 이번에 만든 파일임을 확인한 경우에만:

```bash
sudo rm -- /etc/udev/rules.d/99-ebimu-tractor.rules
sudo udevadm control --reload-rules
```

트랙터 USB만 재연결하면 별칭이 사라진다. ttyUSB 노드와 다른 udev 규칙·프로젝트는 유지된다.
비교 실패/다른 파일이면 삭제하지 않는다. 현재는 미설치이므로 원복할 시스템 변경도 없다.
