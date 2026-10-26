# YDLIDAR G6 실차 SLAM 통합 결과

작성일: 2026-10-17, Ubuntu 22.04 / ROS 2 Humble, `~/mvsim_slam_real`.

**실제 G6 → `/scan` → Cartographer → 지도·센서 위치추정 → RViz 연결을 정지 상태에서 검증했다. 전체 목표 진행률은 90%로 기록한다.** 차량 장착 위치·방향, 차량 기준 TF, 실물 장애물과 스캔 방향 대조, IMU fusion, 이동 SLAM 및 자율주차는 미검증이다. 결과 문서 작성까지 완료했어도 이 항목들을 100% 성공으로 처리하지 않는다.

실제 센서를 사용했으며 이번 검증에서 합성 scan, 가짜 odometry, 가짜 IMU를 생성하지 않았다. 차량 모터·조향·Serial Sender·자율주차 노드는 실행하지 않았다. LiDAR의 스캔 모터만 요청 범위 내에서 시작·정지했다. 종료 후 G6 포트 점유와 이번 SLAM/RViz 프로세스가 남지 않은 것을 확인했다.

## 1. 기존 구조와 작업 전 기준

| 기존 ROS 패키지 | 확인한 역할·이번 처리 |
|---|---|
| `camera_perception_pkg` | 비전 인식, 보존 |
| `debug_pkg` | 시각화, 보존 |
| `decision_making_pkg` | 기존 의사결정, 보존 |
| `interfaces_pkg` | `MotionCommand.msg` 등 메시지, 보존 |
| `launch_pkg` | 기존 전체 실행, 제어 노드를 포함하므로 이번에는 실행하지 않음 |
| `lidar_perception_pkg` | 기존 RPLidar UART 라이브러리·publisher·processor, 보존 |
| `ebimu_driver` | tractor/trailer EBIMU UART 수신, 수정·장치 실행 없음 |
| `real_vehicle_bridge` | LaserScan gateway, 센서 TF, 상태추정, 제어 adapter, 모두 보존 |
| `real_parking` | 최종 B, Hybrid A*, LQR 및 원형 vendor 알고리즘, 보존 |
| `mvsim_slam_real_bringup` | 기존 YAML/Lua/Launch/RViz 재사용, G6 전용 구성을 추가 |
| `serial_communication_pkg` | Arduino Serial Sender, 수정·실행 없음 |

`src/control`의 Arduino 스케치와 `src/data_collection` 등 비패키지 파일도 확인·보존했다. 기존 LiDAR 하위 라이브러리는 RPLidar 프로토콜로 G6를 지원하지 않는다. 이를 G6용으로 개조하지 않고, 공식 YDLIDAR 드라이버만 추가했다. 새 사용자 정의 드라이버 패키지를 만들지 않았다.

기존 데이터 흐름:

```text
RPLidar 또는 외부 LaserScan → /lidar_raw → real_vehicle_bridge/lidar_gateway → /scan
                                                        → Cartographer → /map, /tf
기존 lidar_processor_node: /lidar_raw → /lidar_processed (별도 회전·반전 처리)
```

기존 processor는 애플리케이션 전용 반전 처리를 포함하므로 G6 SLAM 입력에 추가하지 않았다. 기존 RPLidar publisher에는 포트 하드코딩과 별도 TF 발행이 있어 G6와 동시에 실행하지 않는다.

기존 `slam.launch.py`, `real_parking.launch.py`, `reference_lidar_only.lua`, `reference_imu_optional.lua`, 공통 Cartographer Lua, `slam.yaml`, `lidar.yaml`, `vehicle_geometry.yaml`는 그대로다. 기존 B Lua는 2D, `num_laser_scans=1`, `tracking_frame=published_frame=base_footprint`, `map_frame=map`, `odom_frame=odom`, `provide_odom_frame=true`, `use_odometry=false`이다. 기본 IMU는 꺼져 있다. optional Lua만 `tractor_imu_link`와 IMU를 사용한다.

기존 TF는 Cartographer의 `map → odom → base_footprint`이며, sensor_tf가 `base_footprint → base_link/laser_frame/tractor_imu_link`, `trailer_link → trailer_imu_link`, `map → local_map`을 제공한다. estimator가 `base_footprint → trailer_link`를 제공한다. 기존 LiDAR 장착값 `[0,0,0.35,0,0,0]`은 **PROVISIONAL**이므로 이번에 실측값으로 사용하지 않았다.

SLAM 결과는 기존 estimator가 `/map`, `/scan`, `/scan_matched_points2`, `/tf`를 받아 rear-axle 기준 `/parking/state`, `/parking/detection_map`, `/tractor/localization/local_map`, 차체 pose·tracking path로 변환한다. 이 상태추정은 실제 IMU, 정렬·bias 확인도 필요하므로 독립 G6 SLAM 성공만으로 전체 주차 상태가 유효해지지는 않는다. `/odom` 메시지를 새로 만들지 않았으며 `odom`은 Cartographer TF 프레임이다.

작업 전 재검증:

- `colcon build --symlink-install`: **11 packages finished [2.84s]**.
- 기존 `pytest tests`: **20 passed [0.79s]**.
- 원본 174개·보호 복사본 103개 SHA-256: **PASS**.
- 이전 `verification/tests.log`, `VERIFICATION.md`의 11개/20개 성공 기록과 일치.
- 증거: `verification/g6/baseline_{packages.txt,build.log,tests.log,preservation.log}`.

## 2. 백업과 변경 파일

변경 전에 `src` 전체를 `backups/g6/src_before_g6_20261017_202555.tar.gz`로 백업했고, 기존 소스 180개 해시를 `verification/g6/baseline_source_hashes.json`에 기록했다. 원본 `~/mvsim_slam`은 읽기·해시 비교만 수행했다. Git commit/push는 하지 않았다.

| 파일·경로 | 변경 이유 |
|---|---|
| `src/ydlidar_ros2_driver/` | 공식 Humble 드라이버 추가 |
| 위 패키지 `CMakeLists.txt` | SDK의 빈 library dir 문제를 해결하도록 SDK prefix에서 실제 라이브러리를 찾는 imported target 추가 |
| `third_party/YDLidar-SDK/` | 공식 SDK 소스, ROS 패키지와 분리 |
| `third_party/COLCON_IGNORE` | SDK가 별도 ROS 패키지로 중복 검색되는 것을 방지 |
| `src/mvsim_slam_real_bringup/package.xml` | G6 드라이버, tf2_ros, YAML, 지도 저장 도구 실행 의존성 추가 |
| `.../config/g6.yaml` | G6 전용 실제 센서 파라미터 |
| `.../config/g6_mount.yaml` | 미측정 장착값을 null로 분리; `measured: false` |
| `.../config/g6_lidar_only.lua` | 기존 B Lua를 include하고 센서 원점·G6 최소거리 적용 |
| `.../config/g6_vehicle.lua` | 실측 후 기존 base_footprint 인터페이스 사용 |
| `.../config/g6_imu_optional.lua` | 향후 검증된 IMU용 기존 optional Lua 재사용 |
| `.../mvsim_slam_real_bringup/g6_launch.py` | 포트·장착값 검증, LiDAR/SLAM/RViz 전용 실행 |
| `.../launch/g6_lidar.launch.py` | LiDAR 단독 또는 LiDAR+RViz |
| `.../launch/g6_slam.launch.py` | G6+Cartographer+선택 RViz, 제어 노드 없음 |
| `.../rviz/g6.rviz` | Best Effort scan, transient-local map 표시 |
| `tools/g6_env.bash` | 현재 셸에만 ROS·로컬 SDK 경로 설정 |
| `tools/g6_probe.py` | 실제 scan/map/TF 수신 계측, 센서·제어 데이터 발행 없음 |
| `verification/g6/` | 빌드·하드웨어·지도·화면·종료 증거 |

위 `...`는 `src/mvsim_slam_real_bringup`이다. 드라이버 CMake 수정 전 파일은 `backups/g6/ydlidar_ros2_driver.CMakeLists.txt.upstream`에 있고 diff는 `verification/g6/driver_local.patch`에 있다. 기존 소스에서 바뀐 파일은 bringup `package.xml` **하나**이다(`source_changes.json`). 펌웨어, MotionCommand, serial 패키지, IMU, bridge, Hybrid A*, LQR, B 알고리즘, 기존 SLAM 설정은 변경하지 않았다.

## 3. SDK·드라이버와 의존성

| 항목 | 실제 확인 버전·위치 |
|---|---|
| YDLidar SDK | **1.2.20**, commit `42a82ed10d2304094c111fc63dee8e4a229b79b7` |
| SDK 설치 prefix | `~/mvsim_slam_real/third_party/ydlidar_sdk_install` |
| SDK 링크 | 로컬 정적 `lib/libydlidar_sdk.a`; 시스템 ldconfig 불필요 |
| 공식 ROS 2 드라이버 | **humble** branch, commit `4ef70d3f32a85704ade0be54b214f3763b1ab3e8` |
| 드라이버 package.xml 버전 | **1.0.2** (실행 로그 내 하드코딩 버전 문자열은 1.0.1) |
| Cartographer ROS | 기존 설치 `2.0.9002-1jammy.20260915.225849` 재사용 |
| nav2_map_server | 기존 설치 `1.1.20-1jammy.20251228.102023` 재사용 |
| RViz2 | 기존 설치 `11.2.25-1jammy.20251228.130645` 재사용 |

초기 `/usr`, `/usr/local`, `/opt`, 사용자 로컬 경로와 홈 상위 경로 검색 및 `ros2 pkg prefix`에서 기존 YDLIDAR 설치를 찾지 못했다. SDK와 드라이버만 작업공간에 추가했고 apt/pip/sudo, 전역 라이브러리 교체, `.bashrc` 수정은 수행하지 않았다. SDK Python/SWIG API는 필요 없어 빌드하지 않았다. 기존 ROS/Python과 `psmisc`의 `fuser`를 사용한다.

첫 드라이버 링크는 `cannot find -lydlidar_sdk`로 실패했다. SDK 1.2.20이 `YDLIDAR_SDK_LIBRARY_DIRS`를 빈 값으로 내보내는 원인을 확인하고 **해당 드라이버의 CMake만** 수정했다. 수정 후 전체 빌드가 통과했다. 초기 실패 기록은 `driver_build.log`, 최종 성공은 `integrated_build.log`와 `final_build.log`에서 구분한다.

## 4. 실제 장치 식별

- USB: `10c4:ea60`, Silicon Labs CP2102, cp210x, bridge serial `0001`.
- 이번 연결: `/dev/ttyUSB0`.
- 사용 경로: **`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`**.
- by-path: `/dev/serial/by-path/pci-0000:80:14.0-usb-0:8:1.0-port0`.
- 현재 사용자는 `dialout` 소속이고 장치는 `root:dialout`, `0660`; 권한 변경 불필요.
- 시작 전 다른 프로세스 점유 없음, 종료 후에도 포트 미점유 확인.
- USB-UART는 하나만 보였고 `/dev/ttyACM*`는 없었다. UART 식별만으로 G6라고 단정하지 않고 SDK 질의로 **G6 / model code 13 / firmware 3.2 / hardware 3 / sensor serial 2022061000071139**를 확인했다.

`0001`은 범용 bridge 식별자이므로 향후 동일 CP2102가 추가되면 by-id 충돌 가능성이 있다. 그때 by-path와 SDK의 실제 G6 모델·시리얼을 다시 대조한다. Launch는 포트를 자동 추측하지 않으며 `lidar_port:=...`를 요구한다. IMU/Arduino 포트는 열지 않았다. USB_DATA/USB_PWR 배선을 변경하지 않았다.

## 5. 최종 G6 파라미터

기준 파일: `src/mvsim_slam_real_bringup/config/g6.yaml`.

| 파라미터 | 값 |
|---|---|
| `baudrate`, `sample_rate`, `frequency` | **512000**, **18**, **10.0** |
| `lidar_type`, `device_type` | **1 TRIANGLE**, **0 SERIAL** |
| `isSingleChannel`, `intensity`, `support_motor_dtr` | 모두 **false** |
| `frame_id` | `laser_frame` |
| `range_min`, `range_max` | **0.28m**, **16.0m** |
| `angle_min`, `angle_max` | **-180.0°, +180.0°** (LaserScan에서는 rad) |
| `inverted`, `reversion` | **true**, **false** |
| `auto_reconnect` | **true**; 실제 USB 재연결 시험은 하지 않음 |
| `fixed_resolution` | **false**; 고정 개수 맞추기 위한 SDK 점 복제·절단을 피함 |
| `ignore_array` | 빈 문자열; 차체·주변 환경 임의 마스킹 없음 |
| `intensity_bit`, `abnormal_check_count`, `debug` | 0, 4, false |
| `invalid_range_is_inf`, `use_sim_time` | false, false |

18K 모드의 최소거리는 공식 SDK 표·데이터시트의 **0.28m**를 적용했다. 공식 SDK 표에는 최대 25m, 현재 제품 페이지와 V1.6 데이터시트에는 16m가 있어 확인 가능한 보수적인 16m를 사용했다. Cartographer 최대 입력거리는 기존 B의 **12m**를 유지한다. 따라서 12~16m는 scan에 존재할 수 있어도 지도에는 기존 정책에 따라 제한된다.

공식 드라이버의 방향 정의에 따라 `inverted=true`로 counterclockwise 좌표를 선택하고, 임의 180° 회전인 `reversion`은 끈다. 실제 장착 yaw는 별도 TF에서 다룬다. **실물 앞/왼쪽 장애물과의 일치는 아직 검증하지 않았다.** 값이 잘못되었을 때 기존 processor의 추가 반전으로 우회하지 말고 센서 자세와 원시 방향을 대조한다.

공식 드라이버가 읽는 `invalid_range_is_inf`는 현재 소스에서 출력 처리에 사용되지 않는다. 미수신·범위 밖 bin은 0일 수 있으며 Cartographer 최소거리에서 제외된다. 이를 최대거리의 정상 free-space 측정으로 변환하지 않았다. LaserScan은 SDK의 host 기반 수집 타임스탬프와 beam 간격을 사용하며 외부 하드웨어 시각 동기화는 검증하지 않았다.

공식 근거: [ROS 2 Humble 드라이버](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble), [SDK 모델 표](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/Dataset.md), [G6 제품](https://www.ydlidar.com/product/ydlidar-g6), [G6 데이터시트 V1.6](https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20G6%20Data%20sheet%20V1.6%28211230%29.pdf). 구형 PDF 직접 다운로드는 HTTP 실패로 로컬 파일을 얻지 못했고, 공식 SDK 표·공식 제품 페이지와 검색에 노출된 공식 데이터시트 내용을 대조했다.

## 6. 토픽과 TF

실제 검증한 독립 모드:

```text
G6 → ydlidar_ros2_driver_node → /scan → cartographer_node
                                          ├ /scan_matched_points2
                                          ├ /submap_list → cartographer_occupancy_grid_node → /map
                                          └ /tf: map → odom → laser_frame
                                                        (센서 원점, 차량 원점 아님)
```

| 인터페이스 | 메시지·연결 |
|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan`, Best Effort / Volatile |
| `/map` | `nav_msgs/msg/OccupancyGrid`, Reliable / Transient Local |
| `/scan_matched_points2` | `sensor_msgs/msg/PointCloud2`, 실제 scan matching 결과 |
| `/tf` | `tf2_msgs/msg/TFMessage`, Cartographer 단독 발행 확인 |
| `/submap_list` | `cartographer_ros_msgs/msg/SubmapList` |
| `/start_scan`, `/stop_scan` | `std_srvs/srv/Empty`, 공식 드라이버 서비스 |
| `/finish_trajectory`, `/write_state` | Cartographer trajectory 종료·상태 저장 서비스 |

검증 시 scan publisher 1개, map publisher 1개, dynamic TF publisher 1개였다. `/tf_static` 발행자는 없었고 센서 원점 테스트에는 별도 장착 TF가 필요하지 않았다. Cartographer, RViz, probe의 scan QoS가 모두 Best Effort로 호환되는 것을 endpoint 정보로 확인했다. `/odom` 메시지나 IMU 토픽을 요구하거나 생성하지 않았다.

실측 후 차량 모드:

```text
map → odom → base_footprint → base_link
                         └→ laser_frame  (실측 xyz/rpy)
```

기존 프레임 이름을 존중해 `base_footprint`를 차량 기준으로 유지한다. `g6_mount.yaml`의 `measured=true`, `xyz[3]`, `rpy[3]`를 실측값으로 채우기 전 기본 vehicle 모드는 실행을 거부한다. 현재 값은 null이며 임의 높이·오프셋을 넣지 않았다.

기존 sensor_tf를 별도로 쓰는 경우 G6 Launch의 `publish_mount_tf:=false`로 중복을 피하되, **외부 TF가 실측된 값인지 먼저 확인해야 한다.** 기존 provisional sensor_tf를 그대로 신뢰하라는 뜻은 아니다. 같은 TF edge와 Cartographer를 두 번 실행하지 않는다.

## 7. Cartographer 구성과 호환성

- 기본 `frame_mode=sensor`: `g6_lidar_only.lua`, tracking/published=`laser_frame`.
- `frame_mode=vehicle`: `g6_vehicle.lua`, tracking/published=`base_footprint`, 실측 TF 필요.
- 공통: 2D, scan 1개, point cloud 입력 0개, `use_odometry=false`, `use_imu_data=false`, `provide_odom_frame=true`, 실시간 clock, map=`map`, odom=`odom`.
- 기존 B의 scan matching·motion filter·min_score·global_localization_min_score를 유지. G6의 최소거리만 0.28m로 바꿨다.
- 기본 모드에는 IMU 드라이버가 없고 실제 IMU를 기다리지 않는다. optional IMU Lua는 로딩만 검증했으며 실제 fusion은 미검증이다.
- `use_imu:=true`는 vehicle 모드에서만 허용한다. 실제 gravity 포함 가속도(`/tractor/imu/data`), 각속도, 타임스탬프와 `tractor_imu_link` TF를 외부에서 검증해야 한다. 이 옵션으로 IMU가 자동 실행·검증되는 것은 아니다. Odometry도 미래에 필요하면 실제 데이터부터 확인해야 한다.

현재 센서 원점 pose는 차량 rear-axle pose가 아니다. 미측정 extrinsic 상태에서 기존 주차 estimator나 제어 입력으로 연결하지 않는다. 실측 후 기존 `/scan`, `/map`, `/tf`, `/scan_matched_points2`, base_footprint 구조와 인터페이스가 맞는다. 기존 전체 Launch에 G6를 넣는 방법은 외부 드라이버 출력 `/lidar_raw`와 기존 gateway를 사용하는 방식 또는 직접 `/scan`과 gateway 비활성화 방식이며, 둘을 동시에 발행해서는 안 된다. 이번에는 전체 자율주차 Launch를 실행하거나 변경하지 않았다. 전체 연동은 별도 검증 대상이다.

## 8. 빌드·정적 검사와 회귀

- SDK configure/build/install: **PASS** (로컬 prefix).
- 최종 `colcon build --symlink-install`: **12개 패키지 PASS**.
- 기존 테스트: **20 passed [0.49s]**, 기존 20개 회귀 없음.
- 패키지 XML 12개, bringup/검사 Python 9개, YAML/RViz 11개 파싱: **PASS**.
- 두 G6 Launch `--show-args` 로딩·설치 패키지 검색: **PASS**.
- `g6_lidar_only.lua`: 실제 하드웨어로 실행·처리 성공.
- `g6_vehicle.lua`, `g6_imu_optional.lua`: 별도 domain에서 무입력 프로세스 시작·Lua 로딩·정상 SIGINT 종료 확인. **하드웨어 fusion/차량 TF 성공이 아님**.
- 포트 미지정, 점유 포트, 미측정 vehicle 모드: 실행 거부 **PASS**.
- 원본 174개·보호 복사본 103개 및 변경 전 소스 해시: **PASS**; 기존 소스 삭제 없음.

기존 전체 패키지의 `colcon test` lint 전체 통과를 새로 주장하지 않는다. 여기서 회귀 기준은 이전에 보고된 작업공간 `tests/`의 20개 테스트다. 초기 회귀 실행은 검증용 ROS_DOMAIN_ID=233이 DDS 포트 범위를 벗어나 중단되었고, 정상 범위 231에서 변경 없이 재실행해 20개 통과했다. 권장 실행 domain은 아래의 220이다.

## 9. 실제 센서·SLAM 검증 결과

| 검사 | 실제 결과 |
|---|---|
| G6 식별·health·init | G6/model 13, health good, init success |
| scan start | SDK `Lidar has started!`, 18K/10Hz 설정 확인 |
| LiDAR 단독 연속 수신 | **39.882초 / 387 scan / 9.679Hz**, 최대 간격 0.1059초 |
| 통합 SLAM 연속 수신 | **44.956초 / 436 scan / 9.676Hz**, 최대 간격 0.1072초 |
| scan stamp | 양수·단조 증가, angle/beam count 유효 |
| 통합 수신 지연 | 첫 beam stamp 기준 중앙값 0.1040초, 최대 0.1078초 |
| scan beam 수 | 1853~1867, 유효 거리 beam 835~1085/scan |
| 실제 matching | `/scan_matched_points2` **436개** |
| 지도 업데이트 | `/map` **46개**, 서로 다른 내용 SHA-256 **46개** |
| 실제 시각 TF 조회 | **449회 성공**, 관측 시작 직후 과거 시각 버퍼 부족 1회 |
| 지도 셀 | probe 마지막 204×355, 해상도 0.05m; 최종 저장 시 206×355 |
| 위치추정 | map→laser_frame 실제 TF; 마지막 관측 x=-0.00148m, y=-0.00690m, yaw 약 -0.00397rad |
| RViz GUI | 실제 scan+map 및 `Global Status: Ok` 화면 직접 확인·PNG 저장 |
| 지도 저장 | PGM/YAML 성공, FinishTrajectory/WriteState 응답 code=0 |
| 종료 | LiDAR stop, Cartographer/grid/RViz 모두 clean exit, 포트 해제 |

위 pose는 SLAM 시작 센서 원점에 대한 정지 추정치이며 실측 절대 정확도가 아니다. 실제 회전 지속은 연속 360° 스캔 수신으로 확인했다. 물리 모터의 육안 관찰·회전계 측정은 하지 않았다. 차량을 이동시키지 않았으므로 이동 deskew·누적오차·loop closure·주행 중 위치추정 정확도를 평가하지 않았다.

주요 증거:

- [실제 LiDAR 계측 JSON](verification/g6/lidar_live.json)
- [실제 SLAM 계측 JSON](verification/g6/slam_live.json)
- [실제 통합 실행 로그](verification/g6/slam_hardware.log)
- [RViz 실제 화면](verification/g6/rviz_live.png)
- [최종 RViz 화면](verification/g6/rviz_live_final.png)
- [최종 지도 YAML](verification/g6/maps/g6_stationary_final.yaml), [PGM](verification/g6/maps/g6_stationary_final.pgm)
- [Cartographer 상태](verification/g6/maps/g6_stationary.pbstream)
- [기존 테스트 회귀](verification/g6/regression_tests.log), [보존 검사](verification/g6/final_preservation.log)

## 10. 남은 문제와 검증 범위

1. **장착 실측 없음.** base_footprint 대비 x/y/z[m], roll/pitch/yaw[rad], 센서 앞 방향이 필요하다. 트랙터/트레일러 어느 곳에 설치했는지도 최종 확인해야 한다. 트레일러 장착이면 현재 tractor 고정 TF 가정을 그대로 적용할 수 없다.
2. **방향·자차 반사 미검증.** 실제 앞·왼쪽 장애물과 RViz를 대조하고 트랙터/트레일러가 센서 시야에 들어오는지 확인해야 한다. 현재 임의 마스킹은 없다.
3. **시작 시 checksum 경고.** 두 번의 실제 실행 모두 시작 직후 6회의 checksum 로그와 SDK intensity 자동 판별(16→8→0)이 기록됐다. 이후 반복 checksum/scan 실패는 관측되지 않았다. SDK 해당 코드가 checksum 실패 시 intensity 포맷을 바꾸는 것을 확인했지만 최초 오류의 물리 원인까지 입증한 것은 아니다. 전력·케이블 상태를 소프트웨어만으로 완전히 검증했다고 주장하지 않는다.
4. **일부 포인트 시간 경고.** Cartographer 로그에 `Dropped 2/3/2/8/11/2 earlier points`가 총 6회(28개 포인트) 있었다. 시작 시 velocity queue 경고도 1회 있었다. scan 전체의 stamp는 단조 증가했고 지도·matching은 계속됐다. beam 끝 시각과 다음 scan의 시작 시각이 일부 겹칠 가능성이 있으며, SDK timestamp 및 각도 binning/획득 순서의 이동 중 정확도는 별도 분석·측정이 필요하다. 경고를 숨기거나 timestamp를 임의 조작하지 않았다.
5. **TF 관측 초기 1회 실패.** probe가 들어온 직후 요청한 scan 시각이 새 Buffer의 최초 TF보다 약 0.093초 과거였다. 이후 449회 조회 성공. 지속 TF lookup 오류로 보지는 않지만 무오류라고 기록하지 않는다.
6. **실측 scan 약 9.68Hz.** 설정·장치 보고는 10Hz이나 실측 수신·stamp 간격은 약 9.68Hz였다. 설정값과 측정값을 구분한다.
7. **IMU·odometry·이동·자율주차 미검증.** 차량 구동/조향/이동은 수행하지 않았다. 최종 이동 검사는 별도 승인된 안전한 실험이나 수동 이동 시험으로 남긴다.

## 11. 실행 명령

아래 명령은 **각 터미널에서 공통 준비를 먼저 실행**한다. 모든 프로세스의 ROS_DOMAIN_ID가 같아야 한다. 이번 계측은 232에서 했으며, 이후 독립 실행 예시는 DDS 포트 여유가 있는 220을 사용한다. 다른 시스템과 연결하려면 domain을 명시적으로 맞춘다.

### 공통 준비

```bash
cd ~/mvsim_slam_real
source tools/g6_env.bash
export ROS_DOMAIN_ID=220
export ROS_LOCALHOST_ONLY=1
export G6_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
ls -l "$G6_PORT"
```

### 필요할 때 재빌드

현재 SDK와 ROS 패키지는 이미 빌드되어 있다. 재빌드 시:

```bash
cd ~/mvsim_slam_real
source tools/g6_env.bash
cmake -S third_party/YDLidar-SDK -B third_party/ydlidar_sdk_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/third_party/ydlidar_sdk_install" \
  -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonInterp=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonLibs=TRUE
cmake --build third_party/ydlidar_sdk_build -j 4
cmake --install third_party/ydlidar_sdk_build
colcon build --symlink-install
source install/setup.bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /usr/bin/python3 -m pytest tests -q
```

### LiDAR 단독 / LiDAR+RViz

```bash
# LiDAR만 실행
ros2 launch mvsim_slam_real_bringup g6_lidar.launch.py lidar_port:="$G6_PORT"
```

```bash
# 위 실행을 Ctrl+C로 끝낸 뒤 LiDAR+RViz 실행
ros2 launch mvsim_slam_real_bringup g6_lidar.launch.py \
  lidar_port:="$G6_PORT" enable_rviz:=true
```

### 실제 LiDAR 데이터 확인 — 다른 터미널

```bash
ros2 topic info /scan --verbose
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --qos-reliability best_effort
ros2 topic hz /scan
/usr/bin/python3 tools/g6_probe.py --duration 40 \
  --output verification/g6/manual_lidar_probe.json
```

마지막 명령은 관측만 한다. 30초 이상 연속 수신, 실제 유효 거리, 시각 조건 등이 실패하면 exit 1이다. LiDAR 단독에서 map/TF가 없다는 것은 예상된 상태이며 `--require-slam`을 주지 않는다.

### RViz2만 별도 실행

```bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix mvsim_slam_real_bringup)/share/mvsim_slam_real_bringup/rviz/g6.rviz" \
  -f laser_frame
```

SLAM을 보고 있으면 마지막 인자를 `-f map`으로 바꾼다. 이 명령은 드라이버를 추가 실행하지 않는다.

### G6+Cartographer SLAM — 독립 실행

LiDAR 단독 Launch를 먼저 종료한다. 장착값 없이 가능한 **센서 원점 기준** 구성:

```bash
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py \
  lidar_port:="$G6_PORT" enable_rviz:=true
```

이미 LiDAR 단독 실행 중이라면 드라이버를 중복 실행하지 않고 다음을 사용할 수 있다:

```bash
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py \
  start_driver:=false enable_rviz:=true
```

다른 터미널에서 실제 SLAM 계측:

```bash
/usr/bin/python3 tools/g6_probe.py --duration 45 --require-slam \
  --output verification/g6/manual_slam_probe.json
ros2 run tf2_ros tf2_echo map laser_frame
```

차량 프레임은 **실측값을 채운 이후에만**:

```bash
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py \
  lidar_port:="$G6_PORT" frame_mode:=vehicle \
  mount_file:="$HOME/mvsim_slam_real/src/mvsim_slam_real_bringup/config/g6_mount.yaml" \
  enable_rviz:=true
```

현재 `measured=false`이므로 위 vehicle 명령이 거부되는 것이 정상이다. vehicle 전환은 자율주차 제어를 실행하지 않는다.

### 지도 저장 — SLAM 실행 중 다른 터미널

```bash
mkdir -p "$HOME/mvsim_slam_real/maps"
G6_MAP_PREFIX="$HOME/mvsim_slam_real/maps/g6_$(date +%Y%m%d_%H%M%S)"
ros2 run nav2_map_server map_saver_cli -f "$G6_MAP_PREFIX" \
  --ros-args -p save_map_timeout:=10.0
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${G6_MAP_PREFIX}.pbstream', include_unfinished_submaps: true}"
```

PGM/YAML은 occupancy 지도이고 pbstream은 Cartographer 내부 상태다. 둘은 다른 산출물이다. 이번에 실제로 두 종류 모두 저장했다.

### 안전한 종료

필요한 지도를 먼저 저장한 후 **실행한 Launch 터미널에서 Ctrl+C**. 드라이버 종료 코드가 `turnOff()`와 disconnect를 호출하고 `Lidar has stopped!`를 기록하는 것을 실제 확인했다. 별도 RViz 터미널도 Ctrl+C로 종료한다. 선택적으로 종료 전 스캔 중지:

```bash
ros2 service call /stop_scan std_srvs/srv/Empty '{}'
```

`/stop_scan` 응답만으로 물리 회전 정지까지 확정하지 말고 Launch 종료·포트 해제도 확인한다. 다른 ROS 프로세스를 일괄 kill하는 명령은 사용하지 않는다. 검증 종료 후 이번에는 모든 프로세스를 정리했으므로 센서/SLAM이 계속 실행 중이지 않다.

## 12. 다음 실측·검증 항목

필요한 사용자 정보는 LiDAR의 장착 대상(트랙터/트레일러), base_footprint 대비 xyz/rpy, 실물 앞 방향이다. 이를 확보한 뒤 차량 기준 TF와 장애물 방향을 확인한다. 이후 실제 IMU 수신·축·가속도와 필요시 진짜 odometry를 검증하고, 별도로 승인된 수동 이동/주행 SLAM을 실시한다. 전체 B 자율주차와의 인터페이스 검증·구동은 그 다음 단계이며 이번 작업에서 승인 없이 실행하지 않았다.
