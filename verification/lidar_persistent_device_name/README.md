# G6 전용 USB 소켓 등록

2026-10-24 사용자가 G6만 연결했다고 확인한 상태에서 `/dev/ttyUSB0`를 확인했습니다.
VID/PID `10c4:ea60`, serial `0001`, interface `00`은 과거 IMU와 동일합니다.
따라서 장치 자체 식별이 아닌 **현재 물리 USB 소켓을 G6 전용으로 배정**합니다.
허브를 사용한다면 PC 연결 소켓과 허브의 연결 위치를 모두 유지하세요.
이 소켓에 IMU를 꽂으면 IMU도 `/dev/lidar_g6`로 이름이 붙으므로 다른 소켓을 사용해야 합니다.

현재 상태: 사용자 관리자 설치 후 `/dev/lidar_g6 → ttyUSB0` 생성, 규칙 일치, 읽기·쓰기 권한 및 포트 미점유를 확인했습니다. USB 재연결 후 지속 여부는 아직 미검증입니다.
아래는 설치 명령 기록입니다.

```bash
cd ~/mvsim_slam_real
sudo /usr/bin/python3 tools/register_lidar_port.py --install
ls -l /dev/lidar_g6
```

설치 도구는 충돌 규칙과 별칭을 검사하고, 다른 내용의 기존 파일은 덮어쓰지 않습니다.
규칙 reload 후 해당 tty에만 change 이벤트를 적용하며, 장치 권한이나 센서 설정을 변경하지 않습니다.
동일 규칙으로 재실행할 수 있습니다. 재연결 후 별칭 지속 여부는 추가 확인이 필요합니다.

설치 후 G6 실행 인자는 `lidar_port:=/dev/lidar_g6`를 사용합니다.
설치 전에도 현재 소켓의 기존 안정 경로를 사용할 수 있습니다:
`/dev/serial/by-path/pci-0000:80:14.0-usb-0:7.1:1.0-port0`.

등록 점검만 실행 (시스템 변경 없음):
```bash
/usr/bin/python3 tools/register_lidar_port.py
```
