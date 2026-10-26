# 트랙터 IMU 전용 USB 소켓 등록 (2026-10-24)

현재 사용자가 연결한 IMU는 `/dev/ttyUSB1`, 물리 경로는
`pci-0000:80:14.0-usb-0:7.2:1.0`입니다. 라이다의 `7.1`과 구분됩니다.
VID/PID `10c4:ea60`, serial `0001`이 라이다와 동일하므로 현재 소켓을 전용으로 배정합니다.
이 소켓에 다른 CP2102를 꽂으면 그 장치도 별칭을 받습니다. 허브/PC 연결 위치를 유지하세요.

현재 상태: 사용자 관리자 설치 후 `/dev/imu_tractor → ttyUSB1`, 라이다 `/dev/lidar_g6 → ttyUSB0`를 확인했습니다. 두 규칙 일치·읽기/쓰기 권한·미점유 확인 완료. USB 재연결 시험은 아직 미검증입니다.
아래는 설치 명령 기록입니다.
센서 수신·설정·보정 명령은 이 도구에서 수행하지 않습니다.

```bash
cd ~/mvsim_slam_real
sudo /usr/bin/python3 tools/register_tractor_imu_port.py --install
ls -l /dev/imu_tractor /dev/lidar_g6
```

기존 이름의 다른 내용 규칙/별칭은 덮어쓰지 않습니다. 동일 규칙 재실행은 가능합니다.
등록 후 라이다와 함께 별칭 및 USB 재연결을 확인하세요.

과거 `verification/imu_persistent_device_names/`의 후보는 다른 소켓(`8`)을 위한
미설치 기록입니다. 현재 등록에는 이 폴더의 후보와 위 명령을 사용하세요.
등록 전에도 기존 by-path를 쓸 수 있습니다:
`/dev/serial/by-path/pci-0000:80:14.0-usb-0:7.2:1.0-port0`.
