# G6 전방/RViz 화면 방향 작업 기록

진행률: **80% — 실제 목표물 배치 확인 대기**. 물리적 전방과 좌우 방향 검증은 미완료이며, 100% 완료로 판정하지 않는다.

## 변경

- `src/mvsim_slam_real_bringup/rviz/g6_front_up.rviz`: 기존 Grid/Scan/Map/TF 유지, LiDAR Axes 및 Views 패널 추가. Fixed Frame 및 Target Frame은 `map`, TopDownOrtho Angle은 **−π/2 rad**. 빨강 +X, 초록 +Y. 카메라만 회전하며 ROS 스캔과 TF에는 회전을 적용하지 않음.
- `src/mvsim_slam_real_bringup/mvsim_slam_real_bringup/g6_launch.py`: 선택 인자 `rviz_config` 추가. 생략 시 기존 `g6.rviz` 사용. 그 외 노드와 TF 동작 보존.
- `src/mvsim_slam_real_bringup/config/g6.yaml`: 직전 0.15~1m 시험 설정을 **기존 0.28~15m**로 복원. 12Hz/18K, inverted=true, reversion=false 유지.
- `src/mvsim_slam_real_bringup/config/g6_lidar_only.lua`: 기존 0.28~15m 및 missing_data_ray_length=12m 복원. 좌표계 및 알고리즘 보존.

기존 `rviz/g6.rviz` 파일은 바이트 단위로 동일하다. 다른 소스 244개 파일 SHA256 비교에서 변경 없음. 드라이버/SDK, 차량 제어, Hybrid A*, LQR, IMU, Arduino 및 별도 시뮬레이터는 수정하지 않았다. 구동 관련 노드는 실행하지 않았으며 commit/push는 하지 않았다.

## 좌표계 근거

제조사 G6 Data sheet V1.6 2.6절(본문 4쪽, PDF 6쪽)은 케이블 출구 방향을 0°, 위에서 본 시계 방향을 양의 각도로 정의하며 개체별 ±3° 차이를 명시한다.

출처: <https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20G6%20Data%20sheet%20V1.6%28211230%29.pdf>

이번에는 웹 도구에 제공된 공식 문서 본문을 확인했다. 원본 PDF 직접 다운로드는 404였으므로 로컬 PDF를 확보했다고 주장하지 않는다.

현재 SDK `CYdLidar.cpp`는 센서 각도와 장치 zero-offset을 합한 뒤 reversion=true인 경우에만 π를 더하고, inverted=true이면 2π−angle로 변환한다. 따라서 현재 설정에서 제조사 정의와 ROS +X/+Y는 부합할 것으로 예상된다. 장치/실물 방향의 검증 결과를 대체하지 않는다.

RViz Humble TopDownOrtho 구현은 카메라를 Z축 주위로 Angle만큼 회전한다. 원래 Angle=0에서는 +X가 오른쪽, +Y가 위쪽이다. Angle=−π/2에서는 지도 +X가 위쪽, +Y가 왼쪽이다. 이는 센서의 90° 보정이 아니다.

출처: <https://raw.githubusercontent.com/ros2/rviz/humble/rviz_default_plugins/src/rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.cpp>

Cartographer tracking/published frame은 `laser_frame`, fixed/map frame은 `map`. 실제 TF는 `map → odom → laser_frame`. sensor 모드에는 `base_link` 또는 장착 static TF가 없으며 임의의 차체 프레임을 만들지 않았다. /scan frame_id는 laser_frame, 각도는 −π~+π, angle_increment는 양수이다.

## 실제 관측 결과

- 35초 관측: 스캔 408개, 실제 수신 11.647Hz. 실행 파라미터 및 드라이버 표시 주파수는 12Hz, 샘플링 18K.
- /scan range_min=0.28m, range_max=15m; 각도 간격은 0.2329~0.2338°.
- 지도 35회, scan-matched cloud 408회 수신. /scan, /map, /tf 각각 발행자 하나. /tf_static 발행자 없음.
- 스캔 시각 기준 TF 조회 성공 349회. 관측기 초기 버퍼 시점에 past extrapolation 1회, RViz 시작 시 future extrapolation 약 0.9ms 1회. 이후 지속 오류는 관찰하지 않았고 RViz Global Status는 Ok였다. TF 오류 0건이라고 보고하지 않는다.
- 관측 구간 지도 기준 센서 yaw는 -0.783~0.613°, 최초 관측 pose 대비 최대 이동 추정 0.0164m. 사용자에게 물리적 고정 여부를 확인받은 결과는 아니다.
- RViz 화면 캡처에서 빨강 +X는 거의 위쪽, 초록 +Y는 왼쪽이다. 정확한 물리적 케이블 정렬은 목표물로 추가 확인해야 한다.
- ROS 0° ±5° 관측 중앙값 약 1.18m. 사용자 배치 확인이 없어 이 물체가 요청한 목표물인지는 미확인이다.
- 지도 출력은 확인했다. 실제 공간과의 정밀 비교나 이동 SLAM을 수행하지 않아 지도 왜곡이 전혀 없다는 판정은 보류한다.

## 남은 실물 검증

센서를 회전/이동시키지 않고, 센서 스캔 높이에 걸리는 판/상자를 중심에서 약 1m 떨어진 케이블 전방에 놓은 후 배치 완료를 알려야 한다. 각 단계의 물리적 배치 확인 뒤 원시 /scan을 기록한다. 이후 전방 물체를 옮겨 후방, 좌측, 우측을 차례로 확인한다.

예상 대응은 전방 0°(+X), 후방 ±180°(−X), 전방을 바라볼 때 좌측 +90°(+Y), 우측 −90°(−Y)이다. 기존 배경 물체를 목표물로 오인하지 않도록 필요하면 물체를 놓기 전/후 스캔을 비교한다. 측정 전 회전 오프셋을 추가하지 않는다. 데이터시트의 ±3° 개체 차이, 목표물 폭, 실제 배치 오차를 고려하며 오프셋을 추측하지 않는다.

## 실행

이미 실행 중이면 먼저 기존 Launch 터미널에서 Ctrl+C로 종료한다. 포트를 점유한 드라이버를 중복 실행하지 않는다.

```bash
cd ~/mvsim_slam_real
source tools/g6_env.bash
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py \
  lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  enable_rviz:=true \
  rviz_config:="$PWD/src/mvsim_slam_real_bringup/rviz/g6_front_up.rviz"
```

필요한 설치 파일을 갱신할 때는 `colcon build --symlink-install --packages-select mvsim_slam_real_bringup` 후 `source install/setup.bash`를 실행한다. 이번 빌드는 완료했다.

이 설정은 지도 기준 화면 회전이다. 정지 상태에서 초기 센서 +X와 map +X가 거의 일치하므로 전방이 위쪽으로 보인다. 센서 yaw가 바뀌면 빨간 센서 축도 화면에서 돌아간다. 이동/회전 중에도 항상 센서 전방을 위쪽으로 유지하려면 별도 yaw 추종 카메라가 필요하며 Fixed Frame을 센서로 바꾸어 이를 대체하지 않는다.

## 원복

화면만 이전으로 되돌리려면 실행을 중지하고 위 명령에서 `rviz_config` 인자를 빼고 다시 실행한다. 원본 g6.rviz는 그대로 남아 있다. 새 RViz 파일도 보존 가능하다.

Launch 코드까지 원복하려면 실행 종료 후 다음 명령을 사용한다.

```bash
cd ~/mvsim_slam_real
cp backups/g6_front_up_20261018/g6_launch.py \
  src/mvsim_slam_real_bringup/mvsim_slam_real_bringup/g6_launch.py
source tools/g6_env.bash
colcon build --symlink-install --packages-select mvsim_slam_real_bringup
source install/setup.bash
```

백업 경로는 `backups/g6_front_up_20261018/`. 이 백업의 g6.yaml 및 g6_lidar_only.lua는 **이번 작업 시작 시점의 0.15~1m 시험값**이다. 이번 요청의 15m 보호 조건을 유지하려면 이 두 파일을 덮어쓰지 않는다. 원본 성공 RViz 파일의 백업은 `backups/g6_front_up_20261018/g6.rviz`이다.

검증 기록: `verification/g6_front_up_20261018/`의 launch.log, live_35s.json, live_35s.log, rviz_before.png, rviz_front_up.png, progress.json. direction_probe.py는 구독만 수행하며 센서/제어 메시지를 발행하지 않는다.
