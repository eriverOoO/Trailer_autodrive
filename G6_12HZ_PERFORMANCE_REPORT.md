# G6 12Hz·18K·15m 설정 및 Cartographer 검증

2026-10-17 / Ubuntu 22.04 / ROS 2 Humble / `~/mvsim_slam_real`

**이번 요청의 설정 변경·60초 이상 시험·비교·보고서 작성은 완료했다(100%). 최종 설정은 12Hz·18K·15m를 유지한다.** 실측 스캔 주파수는 **11.666Hz**이며 설정값 12Hz와 구분한다. 정지 상태 SLAM은 통과했지만 일부 포인트 시간 경고가 증가했다. 이동 SLAM, 차량 장착 TF, IMU fusion, 15m 실물 표적 정확도까지 검증한 결과는 아니다.

## 1. 기존 설정 추적과 백업

실제 실행 경로:

```text
g6_slam.launch.py
  → mvsim_slam_real_bringup/g6_launch.py
    → config/g6.yaml                         [실제 드라이버 파라미터]
    → config/g6_lidar_only.lua              [기본 sensor 모드]
      → reference_lidar_only.lua           [기존 B 설정]
        → map_builder.lua / trajectory_builder.lua / pose_graph.lua 등
```

`install/.../config/g6.yaml` 및 Lua는 `src/mvsim_slam_real_bringup/config/`로 연결되는 symlink임을 확인했다. 공식 드라이버의 별도 `src/ydlidar_ros2_driver/params/G6.yaml`은 이 Launch에서 사용하지 않으므로 수정하지 않았다.

기존 10Hz 성공 설정·소스·도구·보고서·검증 결과를 다음 경로에 백업했다:

```text
backups/g6_10hz_before_12hz_20261017_204438/
  g6.yaml
  g6_lidar_only.lua
  g6_vehicle.lua
  g6_imu_optional.lua
  successful_10hz_sources_and_evidence.tar.gz
```

기존 `G6_INTEGRATION_REPORT.md`와 `verification/g6/`는 그대로 두었다. 백업 archive와 대조하여 해당 보고서·검증 파일의 바이트 보존도 확인했다. 원본 `~/mvsim_slam`과 Hybrid A*, LQR, Arduino, IMU, 차량 제어 코드는 변경하지 않았다. Git commit/push, 드라이버 재설치, 패키지 생성·삭제, sudo·pip·apt 설치는 수행하지 않았다.

## 2. 변경 파일과 파라미터

| 파일 | 변경 |
|---|---|
| `src/mvsim_slam_real_bringup/config/g6.yaml` | `frequency: 10.0 → 12.0`, `range_max: 16.0 → 15.0`, 설명 주석 |
| `src/mvsim_slam_real_bringup/config/g6_lidar_only.lua` | 기존 include 이후 `TRAJECTORY_BUILDER_2D.max_range = 15.0` 추가 |
| `tools/g6_performance_probe.py` (추가) | 기존 관측 도구를 재사용해 성능·CPU·pose 기록; 센서/제어 데이터를 발행하지 않음 |
| `verification/g6_12hz/` (추가) | 원시 관측 기록, 로그, 설정 확인, 비교, 지도, 화면 |
| 이 보고서 (추가) | 결과·해석·실행·복구 명령 |

기존 소스/도구의 변경 전 해시 231개를 비교해 **기존 파일 수정은 위 YAML과 Lua 2개뿐**임을 확인했다. Launch, TF, ROS 토픽, 패키지 의존성, SDK와 드라이버 소스는 이번에 변경하지 않았다.

| 설정 | 기존 | 최종 |
|---|---|---|
| frequency | 10.0Hz | **12.0Hz** |
| sample_rate | 18K | **18K 유지** |
| baudrate | 512000 | **512000 유지** |
| LiDAR range_min / range_max | 0.28 / 16m | **0.28 / 15m** |
| Cartographer max_range | 12m | **15m** |
| missing_data_ray_length | 12m | **12m 유지** |
| lidar_type / device_type | 1 / 0 | 유지 |
| isSingleChannel / intensity / support_motor_dtr | false / false / false | 유지 |
| inverted / reversion | true / false | 유지 |
| frame_id / auto_reconnect | laser_frame / true | 유지 |
| fixed_resolution | false | 유지; 고정 개수 보간·복제 없음 |
| IMU / odometry | false / false | 유지 |

유효 반사점을 15m까지 쓰는 것과 누락 광선의 처리 길이는 별개이므로 missing-data 설정은 바꾸지 않았다. `g6_vehicle.lua`는 G6 기본 Lua를 include하므로 15m 설정을 상속한다. 실제 사용하지 않는 optional IMU Lua와 기존 reference Lua는 수정하지 않았다.

## 3. 공식 지원 범위와 측정 의미

[공식 Humble 드라이버 파라미터 문서](https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/details.md)는 G6의 512000baud, 5~12Hz, 8/16/18K를 명시한다. [공식 SDK 모델 표](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/Dataset.md)도 대조했다. 기존 SDK 1.2.20과 ROS 드라이버를 그대로 사용했다.

- `frequency`는 SDK를 통해 장치의 회전 주파수를 설정한다. 실제 장치 응답에서 **10.00→12.00Hz**, 재시험에서 **12.00Hz**를 확인했다. 출력 간격으로 잰 값은 약 11.66Hz이다.
- `range_max: 15.0`은 SDK의 거리 유효성 필터 및 LaserScan 메타데이터다. 탐지 반경을 줄여도 18K 측정을 가까운 영역에 재배분하지 않는다.
- 설정상 이론은 `18000 / 12 = 1500 points/회전`이다. 실측 회전 수가 약 11.66Hz이므로 약 1543개 규모의 bin이 나오는 것은 이론적 1500과 모순되지 않는다. bin 개수는 실제 유효 반사점 수와 다르다.
- SDK 로그의 `Fixed Size: 1525`는 내부 계산값이다. `fixed_resolution=false`여서 실제 메시지 개수를 1525개로 맞추지 않으며, 보고서는 LaserScan 배열을 직접 세었다.
- 현재 SDK의 `hasSampleRate()`는 G6를 샘플링 레지스터 질의·변경 대상에서 제외한다(`third_party/YDLidar-SDK/core/common/ydlidar_help.h`). 따라서 `sample_rate=18` ROS 파라미터, SDK **18.00K** 판정, 약 **17,986 bins/s** 출력은 확인했지만 이를 독립 하드웨어 샘플 카운터 측정 또는 샘플링 레지스터 readback으로 주장하지 않는다. SDK를 바꿔 우회하지 않았다.
- `/point_cloud`도 관측하여 SDK가 거리 필터 후 출력한 포인트 수를 별도로 셌다. 이 수 역시 ADC/레이저 펄스 전체 개수가 아니다.

## 4. 장치·토픽·TF와 시험 구분

실제 장치: G6, model 13, firmware 3.2, hardware 3, serial `2022061000071139`.

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
    → /dev/ttyUSB0 (이번 연결)
```

권한과 포트 미점유를 확인한 뒤 단일 드라이버만 열었다. 기존 USB_DATA/USB_PWR 배선은 변경하지 않았다. ROS_DOMAIN_ID=220, ROS_LOCALHOST_ONLY=1로 실행했다.

```text
G6 → 기존 공식 드라이버 → /scan (LaserScan, Best Effort)
                         → Cartographer 2D
                           → /scan_matched_points2
                           → /submap_list → occupancy_grid → /map
                           → /tf: map → odom → laser_frame
```

IMU·odometry 입력과 차량 구동 노드는 사용하지 않았다. 센서 원점 모드이므로 차량 장착 extrinsic을 임의로 만들지 않았다.

시험은 다음과 같이 구분한다:

1. **LiDAR 단독 65초 관측:** `lidar_65s.json`, 실제 수신 구간 64.844초.
2. **첫 SLAM 65초 관측:** `slam_65s.json`, 실제 수신 구간 64.863초. 이후 사용자가 LiDAR 본체를 움직였다고 확인했다. 큰 회전 pose를 정지 드리프트로 해석하지 않으며, **이 결과의 정지 안정성 판정을 철회**했다. 숫자는 보존하고 이동 사실만 메타데이터에 추가했다. `manual_movement_note.json` 참고.
3. **본체 고정 요청 후 SLAM 65초 재시험:** `stationary_repeat_65s.json`, 실제 수신 구간 **64.887초**. **이 재시험을 최종 정지 SLAM 결과와 10Hz 비교 기준으로 사용한다.** 위치·각도 추정의 시간 변화도 함께 저장했다.

첫 시험을 조사할 때 저장된 pbstream을 별도 domain에서 무입력으로 열었으며, 하드웨어 센서는 사용하지 않았다. 해당 오프라인 프로세스도 종료했다. 이번에 자동 차량 모터·조향·자율주차 명령을 실행하지 않았다.

## 5. 실제 LiDAR 수신 결과

| 측정 | LiDAR 단독 | 최종 고정 상태 SLAM 재시험 |
|---|---:|---:|
| 실제 연속 수신 구간 | 64.844초 | **64.887초** |
| LaserScan 수 | 757 | **758** |
| 평균 주파수 | 11.6587Hz | **11.6664Hz** |
| 인접 stamp 간 주파수 최소~최대 | 11.326~12.034Hz | **11.399~11.913Hz** |
| 수신 시각 간 주파수 최소~최대 | 별도 JSON 기록 | **11.233~12.193Hz** |
| 최대 수신 간격 | 0.08890초 | **0.08902초** |
| scan당 bin 최소/중앙/최대 | 1537 / 1543 / 1549 | **1537 / 1541 / 1547** |
| scan당 유효 반사점 최소/중앙/최대 | 652 / 701 / 741 | **755 / 781 / 805** |
| 전체 유효 LaserScan 비율 | 45.416% | **50.618%** |
| 추정 bin 출력 속도 | 17,986.3/s | **17,986.1/s** |
| 추정 유효 LaserScan 반사점 속도 | 8,168.8/s | **9,104.5/s** |
| SDK 필터 후 point cloud 속도 | 8,316.1/s | **9,282.4/s** |
| SDK cloud당 포인트 중앙값 | 714 | **795.5** |
| 15m 초과 유효 scan/cloud 포인트 | 0 / 0 | **0 / 0** |
| 드라이버 scan 수신 실패 | 0 | **0** |
| 재연결 로그 | 0 | **0** |

두 시험의 유효 반사점 비율 차이는 장면·본체 방향 차이의 영향도 받는다. 이를 설정만으로 인한 성능 차이로 해석하지 않는다. bin에는 0으로 채워진 미수신·무효 슬롯이 포함된다. bin/s를 전부 실제 반사 측정 수로 세거나 RViz 보간점을 센서 측정으로 세지 않았다. 속도 추정은 첫 메시지를 제외한 누적 개수를 첫~마지막 수신 시간으로 나눴다.

최종 재시험의 첫 beam 시각 대비 수신 나이는 중앙값 **86.34ms**, 최대 **89.77ms**였다. 한 회전 수집 시간이 포함되므로 이것을 USB 통신 지연만으로 해석하지 않는다.

모든 메시지의 `range_max=15.0`과 SDK의 범위 필터 소스를 확인했다. 최종 재시험 최대 실제 유효 반사는 약 **9.42m**였고, 전체 시험에서도 약 10.93m였다. **15m 및 15m 초과 실물 표적이 없었으므로 15m 감지 정확도나 실물 초과 거리를 잘라내는 실험 자체를 완료했다고 주장하지 않는다.**

## 6. 최종 Cartographer 정지 검증

| 검사 | 결과 |
|---|---|
| 실제 scan 수신·matching | **758 / 758** |
| `/map` 발행 | **66회**, 내용 SHA-256 **62종** |
| 해상도·마지막 관측 지도 | **0.05m**, 307×157셀 |
| 스캔 시각 TF 조회 | **649회 성공**, 시작 직후 Buffer 부족 1회 |
| RViz | 실제 지도·scan 및 **Global Status: Ok** 확인·PNG 보존 |
| 관측 시작 pose 대비 최대 xy 변화 | **0.01831m (1.83cm)** |
| 관측 시작 pose 대비 yaw 변화 | **-0.4165~+0.6906°** |
| 마지막 yaw 변화 | **+0.1026°** |
| Cartographer ERROR/FATAL | **0건** |
| 모니터링 대상 프로세스 이탈 | **0회** |
| 지도·상태 저장 | 새 PGM/YAML/pbstream, 서비스 code=0 |
| 종료 | 드라이버·Cartographer·grid·RViz 모두 clean exit, 포트 해제 |

pose 수치는 관측 시작 이후 상대적인 **추정 안정성**이며, 측정된 ground truth 대비 절대 정확도가 아니다. 초기 Buffer가 채워지기 전 1회 요청 시각보다 약 33.5ms 이후부터 TF가 있어 과거 외삽 오류가 발생했고 이후 조회는 성공했다. 무오류라고 표현하지 않는다.

CPU는 `/proc/PID/stat`를 약 1초 간격으로 읽었으며 **한 CPU 코어 사용률 100% 기준**이다. 메인 관측 구간의 평균/최대는 다음과 같다. 지도 저장과 종료 최적화 시점은 이 구간 밖이다.

| 프로세스 | 평균 CPU | 최대 CPU | 최대 RSS |
|---|---:|---:|---:|
| G6 driver | 0.57% | 2.00% | 22.69MiB |
| Cartographer | **7.28%** | **25.96%** | **48.24MiB** |
| occupancy_grid | 0.37% | 2.00% | 32.44MiB |
| RViz | 5.46% | 7.00% | 181.41MiB |

## 7. 경고·데이터 손실 분석

**checksum:** 각 G6 시작 시 **6개 로그 항목**, 이후 반복 없음. 첫 드라이버 실행과 정지 재시험을 합하면 12개 로그다. SDK의 intensity 자동 포맷 판별(16→8→0)과 동시에 발생했다. 기존 10Hz 실행도 시작 시 6개였다. 로그 중복이 있을 수 있으므로 이 숫자를 독립 물리 패킷 불량 개수로 단정하지 않는다. USB 전력·케이블을 완전 검증했다는 의미도 아니다.

**시간 경고:** 최종 재시험 전체 실행에서는 `Dropped … earlier points`가 **14회·97포인트**, 실제 계측 구간에서는 **13회·95포인트**였다. 시작 시 velocity queue 준비 경고도 1회 있었다. 첫 사용자 수동 이동 시험은 별도로 전체 10회·74포인트, 계측 구간 6회·67포인트였으며 최종 정지 비교에는 사용하지 않는다.

연속 메시지의 `이전 scan stamp + (bin 수-1) × time_increment`와 `다음 scan stamp`를 대조했다. 최종 재시험에서 양수 시간 중첩이 **18회**, 최대 약 **0.919ms**였다. 첫 시험의 경고와 해당 scan을 대조했을 때도 36포인트 경고가 약 2.005ms 중첩과 일치하는 등 시간 중첩을 확인했다(`timing_warning_analysis.json`). Cartographer가 이전 끝 시각보다 이른 포인트를 버리는 조건과 일치한다. 물리 clock/USB/SDK 각도 binning 중 근본 발생 원인을 완전히 분리한 것은 아니다.

계측 구간의 유효 LaserScan 반사점 591,525개 대비 95개는 단순 비율 약 **0.0161%**이다. **이 비율은 전체 시리얼 패킷 손실률이 아니다.** 각도 bin 중복·무효 반사·필터링과 패킷 손실을 혼동하지 않았다. 최대 수신 간격 약 89ms, stamp 단조 증가, scan/matching 758개, 재연결 0회로 전체 scan 중단의 징후는 관찰되지 않았다.

포인트 시간 경고는 해결하지 않았다. 경고를 숨기거나 timestamp를 임의로 바꾸거나 드라이버를 재작성하지 않았다. 이동 중 deskew·정확도는 별도 검증이 필요하다.

## 8. 기존 10Hz와 비교 및 채택 판단

| 항목 | 기존 기록 | 최종 고정 상태 재시험 |
|---|---:|---:|
| 설정 주파수 | 10Hz | **12Hz** |
| 실제 평균 주파수 | 9.6761Hz | **11.6664Hz** |
| 관측 시간 / scan 수 | 44.956초 / 436 | **64.887초 / 758** |
| 샘플링 설정·SDK 판정 | 18K | **18K** |
| LiDAR / Cartographer 최대거리 | 16m / 12m | **15m / 15m** |
| bin 수 중앙값 | 1859 | **1541** |
| bin 수 최소~최대 | 1853~1867 | **1537~1547** |
| 유효 bin 수 중앙값 | 902 | **781** |
| 전체 유효 비율·SDK cloud 속도 | 기존 원시 기록 없음 | **50.618% / 9282.4/s** |
| checksum 로그/시작 | 6 | **6** |
| 시간 경고/계측 구간 | 3회 / 7포인트 | **13회 / 95포인트** |
| 시간 경고 폐기 포인트/s | 0.156 | **1.464** |
| 실제 matching·지도 | 성공 | **성공** |
| CPU 기준값 | 미기록 | 위 표의 실측값 |

기존 로그 전체의 시간 경고는 6회·28포인트였으나 계측 구간은 3회·7포인트다. 시간 범위가 다른 전체 로그 횟수만 직접 비교하지 않았다. 기존에 없는 원시 반사점 개수·유효 비율·CPU를 추정해서 채우지 않았다.

스캔 수는 **초당 20.57% 증가**, 스캔당 중앙 bin은 **17.11% 감소**했다. 초당 약 18K의 bin 출력은 유지되므로 시간 해상도와 한 회전의 각도 밀도를 교환하는 설정이다. 15m 제한으로 점 밀도가 증가한 것이 아니다.

**최종 채택: 12Hz + 18K + 15m.** 정지 상태 연속 수신·scan matching·지도·TF·pose 변화·CPU·종료가 정상이고 시간 갱신이 빨라 사용 가치가 있다. 다만 시간 경고 폐기량은 증가했으므로 10Hz보다 모든 면에서 우월하거나 이동 정확도까지 동등하다고 단정하지 않는다. 환경·방향 차이도 있어 경고 증가의 원인을 12Hz 하나로 확정하지 않는다. 시간 경고를 보수적으로 줄이고 싶을 때는 아래 방법으로 검증된 10Hz 설정을 복구할 수 있다.

## 9. 빌드·보존 결과

- `colcon build --symlink-install`: **12 packages finished [2.93s]**.
- 기존 `pytest tests`: **20 passed [0.52s]**.
- YAML 값·기존 Launch 해시·symlink 실행 경로·Launch 인자 로딩 검사: 통과.
- 새 관측 도구는 실제 LiDAR/SLAM 시험에서 실행했고, scan·cloud·CPU·pose 원시 기록을 저장했다.
- 기존 알고리즘·제어·IMU·펌웨어 변경 없음. 원본 174개·보호 복사본 103개 SHA-256 최종 검사 통과.
- 이전 10Hz 보고서·검증 자료 보존, 새 지도는 `verification/g6_12hz/maps/`에 별도 저장.

## 10. 최종 실행 명령

각 터미널에서 공통 준비:

```bash
cd ~/mvsim_slam_real
source tools/g6_env.bash
export ROS_DOMAIN_ID=220
export ROS_LOCALHOST_ONLY=1
export G6_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

최종 LiDAR+Cartographer+RViz 실행(차량 제어 없음):

```bash
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py \
  lidar_port:="$G6_PORT" enable_rviz:=true
```

LiDAR만 실행하려면 위 실행을 종료한 뒤:

```bash
ros2 launch mvsim_slam_real_bringup g6_lidar.launch.py lidar_port:="$G6_PORT"
```

다른 터미널에서 확인·65초 계측:

```bash
ros2 param get /ydlidar_ros2_driver_node frequency
ros2 param get /ydlidar_ros2_driver_node sample_rate
ros2 param get /ydlidar_ros2_driver_node range_max
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --qos-reliability best_effort
ros2 topic hz /scan
# 본체 위치·방향을 고정하고, SLAM 실행 중에 사용
/usr/bin/python3 tools/g6_performance_probe.py --duration 65 --require-slam \
  --output verification/g6_12hz/manual_stationary_probe.json
```

LiDAR 단독 계측은 `--require-slam`을 뺀다. 위 간단 계측 명령은 PID를 지정하지 않아 CPU 집계는 비어 있다. 이번 실제 시험에서는 각 실제 PID를 지정해 CPU를 기록했다. 관측 도구의 통과값만으로 물리 정지나 절대 정확도를 증명하지는 않는다.

새 지도 저장(SLAM 실행 중 다른 터미널):

```bash
mkdir -p "$HOME/mvsim_slam_real/maps"
G6_MAP_PREFIX="$HOME/mvsim_slam_real/maps/g6_12hz_$(date +%Y%m%d_%H%M%S)"
ros2 run nav2_map_server map_saver_cli -f "$G6_MAP_PREFIX" \
  --ros-args -p save_map_timeout:=10.0
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${G6_MAP_PREFIX}.pbstream', include_unfinished_submaps: true}"
```

필요한 지도를 저장하고 **실행한 Launch 터미널에서 Ctrl+C**로 종료한다. 드라이버 `Lidar has stopped!`와 각 프로세스 clean exit를 확인한다. 이번 모든 실제·오프라인 실행은 종료했고 포트도 해제했다.

## 11. 기존 10Hz 설정 복구

먼저 모든 G6/SLAM Launch를 Ctrl+C로 종료한 상태에서 실행한다. YAML만 복구하면 SLAM 거리는 15m로 남으므로 **두 파일 모두** 복구한다.

```bash
cd ~/mvsim_slam_real
source tools/g6_env.bash
G6_10HZ_BACKUP="$HOME/mvsim_slam_real/backups/g6_10hz_before_12hz_20261017_204438"
cp -a "$G6_10HZ_BACKUP/g6.yaml" src/mvsim_slam_real_bringup/config/g6.yaml
cp -a "$G6_10HZ_BACKUP/g6_lidar_only.lua" src/mvsim_slam_real_bringup/config/g6_lidar_only.lua
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py \
  lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  enable_rviz:=true
```

이렇게 하면 10Hz·18K·LiDAR 16m·Cartographer 12m의 기존 성공 설정으로 돌아간다.

12Hz로 다시 돌아갈 때는 실행을 종료한 뒤 보존한 최종 설정을 복사한다:

```bash
cd ~/mvsim_slam_real
cp -a verification/g6_12hz/tested_12hz_g6.yaml src/mvsim_slam_real_bringup/config/g6.yaml
cp -a verification/g6_12hz/tested_12hz_g6_lidar_only.lua src/mvsim_slam_real_bringup/config/g6_lidar_only.lua
source tools/g6_env.bash
colcon build --symlink-install
```

## 12. 결과 파일

- [최종 고정 상태 계측](verification/g6_12hz/stationary_repeat_65s.json), [원시 scan·CPU·pose 기록](verification/g6_12hz/stationary_repeat_65s.samples.json)
- [최종 비교·경고 집계](verification/g6_12hz/stationary_comparison.json)
- [실제 통합 로그](verification/g6_12hz/stationary_repeat_hardware.log), [실행 파라미터](verification/g6_12hz/stationary_live_driver_parameters.yaml)
- [최종 RViz 화면](verification/g6_12hz/stationary_rviz_final.png)
- [최종 고정 지도 YAML](verification/g6_12hz/maps/g6_12hz_stationary_repeat.yaml), [PGM](verification/g6_12hz/maps/g6_12hz_stationary_repeat.pgm), [pbstream](verification/g6_12hz/maps/g6_12hz_stationary_repeat.pbstream)
- [수동 이동 확인·첫 시험 분리](verification/g6_12hz/manual_movement_note.json)
- [빌드](verification/g6_12hz/build.log), [기존 20개 테스트](verification/g6_12hz/tests.log), [변경 파일](verification/g6_12hz/source_changes.json)

첫 시험의 `g6_12hz_stationary.*` 파일명은 이동 사실을 알기 전에 저장한 이름이며, 실제로는 사용자 수동 이동이 포함됐다. **정지 지도 증거는 `_stationary_repeat.*`를 사용한다.** 기존 10Hz 지도는 `verification/g6/maps/`에 그대로 있다.
