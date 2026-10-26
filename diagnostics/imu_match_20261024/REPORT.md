# 트랙터·트레일러 IMU 설정 일치 확인 (2026-10-24)

- 사용자 요청: 트랙터 IMU 기존 설정을 기준으로 트레일러를 동일하게 맞추고, 앞뒤 반대 장착만 반영.
- 실제 장치: `/dev/imu_tractor`, `/dev/imu_trailer`, 모두 버전 `<imu9dofv612b>`.
- `<ver>`, `<cfg>`, cfg 종료 `>`만 송신. 두 센서의 조회 가능한 설정 30개가 모두 동일하여 설정 쓰기는 0회.
- 115200bps / ASCII / Quaternion z,y,x,w / gyro DPS / 중력 포함 accel g / sensor timestamp ms / 10ms 주기.
- 필터·범위·자기장 처리·자동 gyro 보정 설정 등 전체 cfg 비교: `settings_comparison.json`.
- 기존 트랙터의 `sem2`, `agc_e0`, `cmo0` 등 그대로 유지. 센서별 캘리브레이션·bias·영점을 복사하거나 보정하지 않음.

ROS 설정 변경:

- `config/imu.yaml` 두 프로파일의 형식·주기·단위·공분산을 기존 `ebimu_slam.yaml`과 통일.
- 장치명·frame·topic만 tractor/trailer로 분리. 기본 enable_device=false 유지.
- `ebimu_slam.yaml`의 트랙터 포트는 충돌 가능한 공통 by-id 대신 `/dev/imu_tractor` 사용.
- 드라이버 mounting_rpy는 둘 다 0. 트레일러 yaw=π는 측정 위치와 함께 `vehicle_geometry.yaml`에 보존.
- 통합 launch에서 실제 wire format과 충돌하던 include_acceleration 강제 덮어쓰기 제거.
- `dual_imu.launch.py` 추가: 두 실제 IMU + 선택적 센서 정적 TF. 제어·모터·자동 보정 노드 없음.

검증:

- bringup 패키지 빌드 성공, 테스트 44개 통과.
- 독립 ROS domain 225에서 실제 두 IMU 동시 수신, 각각 767개, 약 100.03Hz.
- 정규화 quaternion, 증가하는 host timestamp, 중력 포함 가속도 확인.
- 실제 /tf_static의 트랙터 장착 yaw=0°, 트레일러=180° 확인.
- 드라이버와 TF를 둘 다 180° 회전시키지 않도록 프로파일·벡터 변환 테스트.
- 확인 완료 후 이번 검증 프로세스 종료.

현재 축 방향의 절대 기준(트랙터 yaw=0 및 두 IMU roll/pitch=0)은 아직 물리 회전으로 검증하지 않음.
서로 다른 센서의 절대 yaw·영점·gyro bias가 동일하다는 의미는 아님.
geometry_confirmed / initial_alignment_confirmed / trailer_bias_confirmed는 false 유지.

실행:

```bash
cd ~/mvsim_slam_real
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mvsim_slam_real_bringup dual_imu.launch.py
```

다른 센서 TF 노드가 실행 중이면 `publish_sensor_tf:=false`를 지정한다.
