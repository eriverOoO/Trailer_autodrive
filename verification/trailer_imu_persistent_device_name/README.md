# 트레일러 IMU 전용 USB 소켓 등록 (2026-10-24)

사용자가 연결한 트레일러 IMU: `/dev/ttyUSB2`.
전용 물리 USB 경로: `pci-0000:80:14.0-usb-0:7.3:1.0`.
기존 라이다(`7.1`)·트랙터 IMU(`7.2`)와 구분됩니다.
세 장치 모두 VID/PID `10c4:ea60`, serial `0001`이므로 물리 소켓별로 역할을 지정합니다.
현재 소켓과 허브 연결 위치를 유지해야 하며, 장치를 바꿔 꽂으면 역할도 바뀝니다.

현재 상태: 사용자 관리자 설치 후 세 별칭의 장치 구분·소켓 일치·규칙 일치·접근 권한·미점유 확인 완료. USB 재연결 시험은 아직 미검증입니다.
아래는 설치 명령 기록입니다.

```bash
cd ~/mvsim_slam_real
sudo /usr/bin/python3 tools/register_trailer_imu_port.py --install
ls -l /dev/lidar_g6 /dev/imu_tractor /dev/imu_trailer
```

다른 내용의 기존 규칙이나 별칭은 덮어쓰지 않습니다. 해당 tty에만 change 이벤트를 적용합니다.
센서 통신·설정·보정은 수행하지 않습니다. 등록 후 별칭 및 실제 USB 재연결 확인이 필요합니다.
