# 왼쪽 두 구조물 주차 테스트

최종 동작은 **시작 명령 → 자동 전진 탐색 → 카메라/LiDAR로 두 구조물 확인 →
공간 확정 → 기존 Final B 경로계획·주차 제어**입니다. 손으로 미는 방식은
`manual_survey:=true` 중간 검증 옵션입니다. 기존 `real_parking.launch.py`의
U자 공간 검출 기본 동작은 그대로 유지됩니다. 두 실행을 동시에 띄우지 않습니다.

## 공간 정의

- 초기 진행방향의 왼쪽(+Y)에 있는 별도 LiDAR 점유 군집 두 개가 필요합니다.
- 구조물의 도로 쪽 표면 위치로 입구와 방향을, 서로 마주 보는 경계로 길이를 계산합니다.
- 깊이는 `config/two_structure_test.yaml`의 `slot_width: 1.10` m입니다.
  벤츠 NEW GTR AMG 표기 크기 0.99 × 0.55 × 0.41 m의 폭을 기준으로 한 초기값이며
  실제 진입 가능 여부는 기존 전체 견인차·트레일러 충돌 검사와 경로계획으로 판단합니다.
- `slot_width:=1.20`처럼 실행 시 바꾸거나 YAML을 수정하고 재시작할 수 있습니다.
  자동 탐색/주차가 시작된 뒤에는 검출 파라미터 변경을 거부합니다.
- 뒤쪽 벽을 요구하지 않으며 가상의 벽이나 빈 셀을 만들지 않습니다.
  고정 깊이 내부(경계에서 0.10 m 안쪽)의 미관측/점유 셀은 공간 확정을 막습니다.
  실제 LiDAR 광선으로 내부를 관측해야 하며 무반사·가림 때문에 부족하면 후보로만 표시됩니다.
- 기본 최소 간격 2.60 m, 최대 6.0 m이며 실제 차체 길이+0.36 m보다 좁으면 거부합니다.
  현재 카메라 검출은 주황색 라바콘/흰색 차체 모양 후보용입니다. 임의 재질·모양 인식은
  보장하지 않으므로 구조물 결정 후 영상 임계값(`detector_params`)을 맞춥니다.

## 카메라 설치

카메라를 고정한 뒤 기존 [카메라/LiDAR 보정 형식](../README.md#라바콘흰색-전동차-후보-및-라이다-겹침-확인)으로
내부 파라미터·왜곡·LiDAR→카메라 회전/이동값을 실측합니다.
왼쪽 한뼘 반/아래 10 cm라는 대략값을 실측 변환으로 저장하지 않습니다.
장착 각도가 바뀌면 다시 보정해야 합니다. LiDAR 스캔 높이와 카메라 시야 모두에
구조물이 잡히는지 먼저 확인합니다.

보정 파일이 없으면 영상 검출과 LiDAR 공간 후보는 표시하지만 결합 확인과 자동 주행 시작은
거부합니다. 이미지/스캔 시각 차이는 최대 0.10 s이며 스캔 중 움직임의 deskew는 없습니다.
윤곽 안의 연속적인 전경 LiDAR 반사점을 지도 군집과 다시 대응시킵니다. 배경/가림 오대응은
실물로 확인해야 합니다. 두 구조물의 카메라 확인은 서로 다른 프레임이어도 되며, odom에
저장했다가 최신 지도 변환을 적용합니다. 기본 유효기간은 180 s입니다.

## 실행

이 워크스페이스의 기존 빌드/장치 준비 절차를 완료한 후 사용합니다. Cartographer와
G6 드라이버/SDK도 필요합니다. 기본 `enable_actuation:=false`는 모터 송신 없이 명령만 확인합니다.

```bash
cd /home/heven/Trailer_autodrive/mvsim_slam_real
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1

# 하드웨어 전체 실행 예시: 실제 확인한 장치 경로로 바꿉니다.
# 기본은 주행 시작 명령 대기입니다. IMU와 SLAM도 함께 시작됩니다.
ros2 launch mvsim_slam_real_bringup two_structure_parking_test.launch.py \
  development_mode:=false start_g6_driver:=true \
  lidar_port:=/dev/lidar_g6 \
  tractor_imu_port:=/dev/imu_tractor trailer_imu_port:=/dev/imu_trailer \
  camera_path:=/dev/v4l/by-id/실제_카메라 \
  calibration_file:=/절대경로/실측_보정.json \
  enable_rviz:=true

# 동일한 환경의 다른 터미널: 센서가 준비되고 1초간 정지한 뒤 자동 탐색 시작
ros2 service call /parking_test/start_search std_srvs/srv/Trigger '{}'

# 테스트 중 정지. 다시 실행하려면 launch를 재시작합니다.
ros2 service call /parking_test/stop std_srvs/srv/Trigger '{}'
```

실제 모터 구동은 기존 geometry/scale/polarity 확인 조건을 충족한 환경에서만
`enable_actuation:=true arduino_port:=실제_포트`를 추가합니다. 기존 Arduino 송신·감시 한계는
그대로이며 이번 작업에서 물리 주행을 시작하지 않았습니다.

이미 SLAM·상태추정·TF·드라이버가 실행 중이면 `start_stack:=false`와 외부
`image_topic:=/실제_영상`을 사용합니다. 이때 기존 자동주차 노드는 종료하고,
기존 control_adapter의 구동 설정이 적용된다는 점에 유의합니다.

손으로 미는 중간 테스트는 같은 launch에 `manual_survey:=true`를 추가합니다.
이 모드에서는 주행 명령을 0으로 유지합니다. 공간 확인 후 멈춰서
`/parking_test/start_parking` Trigger 서비스를 호출하면 10초 안정화와 새 지도 3회
재확인을 거쳐 기존 주차 동작으로 넘어갑니다. `start_search`는 자동 탐색 경로입니다.

보정 전 LiDAR 후보만 확인하려면 `require_camera:=false`를 명시할 수 있습니다.
이 경우 자동 주행/주차 시작 서비스는 거부합니다.

## 확인 화면과 검증

- `/parking_test/observation`: LiDAR 두 구조물, 카메라 확인, 내부 자유공간, 중심·방향·길이·깊이.
- `/parking_test/markers` (RViz MarkerArray): 노랑=미확정 후보, 초록=확정 공간, 파랑=두 구조물 점.
- `/parking_vision/objects_overlay`: 영상 후보와 투영한 LiDAR 반사점.
- `/parking_vision/associations`: 이미지와 대응된 실제 스캔 XY 좌표.
- `/parking/status`: 기존 검색·안정화·계획·주차 상태. 기존 staging 거리/종료 조건은 유지합니다.

검증은 뒤쪽 벽 없는 지도, 순차 카메라 관측, 좌표 회전, 미관측/장애물 거부,
깊이 변경, 무보정 주행 거부, 수동/자동 시작 및 정지 명령을 합성 입력으로 확인합니다.
이는 카메라 부착·실측 보정·실제 Cartographer 지도 생성·실차 주차 완료의 증거는 아닙니다.
