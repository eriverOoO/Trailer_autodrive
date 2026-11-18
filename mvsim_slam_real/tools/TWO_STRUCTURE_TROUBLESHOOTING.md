# 두 구조물 주차 테스트 실패 원인 정리 (실차 진단용)

`two_structure_parking_test.launch.py` 실행이 실패했을 때 원인을 빨리 좁히기 위한 문서입니다.
사람과 다른 코딩 에이전트가 함께 읽는 것을 전제로, **증상(상태·reason 문자열) → 코드 위치 →
확인 방법 → 흔한 원인** 순서로 적었습니다. 동작 설명은 [TWO_STRUCTURE_TEST.md](TWO_STRUCTURE_TEST.md)를 봅니다.

에이전트 주의: 이 문서의 확인 명령은 모두 **읽기 전용**(topic echo, 로그, 저장 파일)입니다.
모터 구동(`enable_actuation:=true`)이나 하드웨어 launch는 사람이 직접 실행합니다.
`vendor/`, `optimizer/`의 알고리즘 파일은 수정하지 않는 것이 원칙입니다.

## 0. 먼저 볼 것 (순서대로)

테스트 창(`tools/run_two_structure_gui.sh`)을 쓰면 아래 1~5번이 한 화면에 나오고, 실패 이유가
한국어 설명과 함께 표시됩니다. 설명 문구는 `tools/two_structure_gui_core.py`의 `EXPLANATIONS`에 있습니다.

1. `/parking/status` – 현재 단계(stage)와 마지막 전이 사유. FAILED / EMERGENCY_STOP이면 사유 문자열로 아래 표를 찾습니다.
2. `/parking_test/observation` – 공간 검출 결과(JSON). `reason`, `lidar_pair_seen`, `camera_confirmed`, `free_fraction`.
3. `/parking/state`, `/tractor/localization/status` – 위치 추정이 살아 있는지(`valid`, `reason`).
4. `/lidar/status` – LiDAR 게이트웨이 상태(`valid`, `reason`).
5. RViz: `/parking/detection_map`(검출 격자), `/parking_test/markers`(노랑=후보, 초록=확정, 파랑=구조물 점), `/scan`.
6. `blocked_debug_directory`(launch의 `output_directory`)에 저장되는 파일:
   `planning_input_NNN.npz`(계획 입력 격자·출발·목표), `observed_stop_audit.jsonl`, 차단 이벤트 기록.

```bash
ros2 topic echo --once /parking/status
ros2 topic echo --once /parking_test/observation
ros2 topic echo --once /parking/state
ros2 topic echo --once /lidar/status
ros2 topic hz /scan
```

## 1. 전체 흐름과 단계별 상태

```
WAIT_FOR_START_COMMAND (또는 MANUAL_SURVEY)
  └ /parking_test/start_search
WAIT_FOR_LOCALIZATION → CALIBRATE_TRAILER → SEARCH_FORWARD → FIRST_OBSTACLE_SEEN → GAP_SEEN
  → SECOND_OBSTACLE_SEEN → STOP_AND_STABILIZE(10 s) → DETECT_TARGET → PLAN → READY → EXECUTE → PARKED
```

코드: 테스트 노드 [two_structure_test.py](../src/real_parking/real_parking/two_structure_test.py),
공간 검출 [two_structure_gap.py](../src/real_parking/real_parking/two_structure_gap.py),
기존 주차 상태기계 `vendor/tractor_parallel_parking_auto.py`(`tick_checked`),
위치 추정 [estimator.py](../src/real_vehicle_bridge/real_vehicle_bridge/estimator.py),
LiDAR [lidar.py](../src/real_vehicle_bridge/real_vehicle_bridge/lidar.py),
무반사 빈 공간 [free_space.py](../src/real_vehicle_bridge/real_vehicle_bridge/free_space.py).

## 2. 시작 전 (start_search가 거부됨)

| 응답 메시지 | 원인 후보 | 확인 |
|---|---|---|
| `Need current SLAM/IMU/scan/camera and a stationary rig for one second` | 지도·스캔·위치 상태 중 하나가 늦음(`streams_ready`), 카메라 스트림 없음(`camera_ready`, `require_camera:=true`일 때만), 1초간 정지 아님(이동 < 2.5 cm, 각도 < 2°) | `/parking/state` valid, `/scan` 주기, `/parking_vision/associations` 주기, 차가 흔들리지 않는지 |
| `LiDAR-only diagnostics cannot start automatic motion; ...` | `require_camera:=false`인데 `allow_lidar_only_motion:=true`가 없음 | LiDAR만으로 시험하려면 둘 다 지정. 아니면 카메라 보정 후 `require_camera:=true` |
| `Restart the test before starting another search` | 이미 시작했거나 FAILED/EMERGENCY_STOP/PARKED 상태 | launch 재시작 |

## 3. 센서·위치 추정

| 증상 | 코드 위치 | 원인 후보 / 확인 |
|---|---|---|
| `/lidar/status` `no scan received` | `LidarGateway.receive` | G6 드라이버 미실행, 포트(`lidar_port`) 오류, `/lidar_raw` 토픽 없음 |
| `scan timestamp stale/backward` | `LidarGateway.receive` | 스캔 시각이 0.5 s 이상 늦거나 뒤로 감. PC 시계, 드라이버 타임스탬프. `G6_TIMESTAMP_DIAGNOSTICS_REPORT.md` 참고 |
| `no finite scan returns` | `normalize_scan` | 모든 빔이 무반사(완전히 열린 곳/센서 가림). 무반사(+inf)는 유한값이 아니어서 이 스캔은 버려짐 |
| `invalid scan geometry`, `angle/beam count mismatch` | `normalize_scan` | 드라이버 설정(각도 범위·빔 수) 불일치 |
| `/parking/state` `trailer IMU missing/stale`, `tractor IMU missing/stale` | `StateEstimator.evaluate` | IMU 포트, frame_id(`trailer_imu_link`/`tractor_imu_link`), 100 Hz 수신 여부 |
| `scan missing/stale` | 〃 | `/scan` 0.4 s 이상 끊김 |
| `Cartographer actual scan matching missing/stale` | 〃 | Cartographer가 `/scan_matched_points2`를 안 냄 → SLAM 미동작/설정 오류 |
| `SLAM map unavailable` | 〃 | `/map` 없음 또는 점유 셀이 하나도 없음(완전히 빈 공간에서 시작) |
| `SLAM pose stale`, `SLAM pose jump`, `SLAM clock reversed` | 〃 | Cartographer TF 지연·급변. 빠른 회전/진동, 특징 없는 환경 |
| `gyro/SLAM timestamp pairing unavailable` | 〃 | IMU와 SLAM 시각 차이가 30 ms 넘음 |
| `initial articulated alignment and trailer bias not confirmed` | 〃 | `vehicle_geometry.yaml`의 `initial_alignment_confirmed`/`trailer_bias_confirmed`가 false |
| `articulation emergency limit` / EMERGENCY_STOP `articulation hard safety limit` | estimator / vendor | 꺾임각 35°(추정) 또는 `emergency_angle_deg` 초과. 트레일러 IMU 부호·오프셋 의심 |

## 4. 전진 탐색 중 (SEARCH_FORWARD ~ SECOND_OBSTACLE_SEEN)

| 증상 | 원인 후보 / 확인 |
|---|---|
| EMERGENCY_STOP `localization/map stream stale` | 지도 1.5 s, 스캔 1 s, 위치 상태 1.5 s 중 하나라도 끊김 |
| EMERGENCY_STOP `forward search left road corridor` | 시작 위치 기준 옆으로 1.0 m 이상 또는 `search_heading_limit_deg` 이상 벗어남. 조향 부호(`search_steer_sign`), 바퀴 정렬, 바닥 경사 |
| EMERGENCY_STOP `search footprint collides with observed map` | 차체가 지도상의 점유 셀과 겹침. 경로 옆 벽·물체가 너무 가깝거나 self filter 범위 부족(자기 차체가 점으로 찍힘) |
| FAILED `automatic gap search limit reached` | 22 m 또는 220 s 안에 공간을 확정하지 못함 → 아래 5절의 검출 실패 원인 확인 |
| 공간을 지나쳤는데 계속 전진 | 확정(`observation_hits >= 3`)이 안 됐거나, 확정 후 정지 위치(목표보다 약 7.1 m 앞)에 아직 도달 안 함. legacy 정지 정책은 공간을 지나 약 7 m를 더 가서 멈춤. 앞쪽 공간이 부족하면 벽에 걸림 |

## 5. 공간 검출 실패 (`/parking_test/observation`의 reason)

검출은 `/parking/detection_map`(임계값 50, 미관측 -1 유지 + 무반사 빈 공간 보강)에서 합니다.

| reason | 의미 | 원인 후보 / 확인 |
|---|---|---|
| `waiting for measured local_map detection grid` | 검출 격자 미수신 | state_estimator가 `/map`을 못 받음 |
| `detection map timestamp stale/future` | 검출 격자가 2.5 s 이상 묵음 | Cartographer 지도 발행 주기, 시계 |
| `waiting for two observed left-side structures (found N)` | 구조물 후보가 N개(2개 미만) | 구조물이 LiDAR 높이에 걸리는지, 왼쪽 0.55~6 m 안인지, 벽에 붙어 있으면 0.15 m 이상 튀어나왔는지 |
| `N structures found but no usable pair: gap X m outside ...` / `pair heading ...` | 후보는 있으나 간격·방향 조건 불만족 | 간격 3.30~6.0 m, 진행방향과 10° 이내. 버린 쌍 목록이 `/parking_test/observation`의 `rejected_pairs`에 있음 |
| `... no usable pair: an object in front sits between them` | 두 구조물 사이 앞쪽(입구 쪽)에 다른 후보가 있음 | 두 구조물 사이의 물체 |
| (이전 문자열) `waiting for two observed left-side structures` | 조건에 맞는 쌍이 없음 | 아래 "구조물 후보 조건". 두 구조물 사이에 점유 셀 3개 이상인 물체가 있으면 그것도 구조물 후보가 되어 쌍이 깨지고, 내부 검사까지 가지 않고 이 메시지가 나옴 |
| `gap found; interior blocked or unknown (free=…, unknown=a/b, occupied_group_cells=c/d)` | 두 구조물은 찾았지만 내부 조건 불만족 | 아래 "내부 조건". `unknown` a가 허용치 b를 넘으면 미관측이 넓음, `occupied_group_cells` c가 d를 넘으면 공간 안에 큰 점유 덩어리(물체·벽) |
| `waiting for calibrated camera confirmation of both LiDAR structures` | LiDAR 공간은 OK, 카메라 확인 대기 | 아래 "카메라 조건" |
| `camera unavailable: ...` | 카메라 스트림이 끊기거나 시각 불일치 | `/parking_vision/associations`의 reason |
| `LiDAR-only test: camera confirmation bypassed` | `require_camera:=false` | 공간 확정. 자동 주행은 `allow_lidar_only_motion:=true`일 때만. `camera_confirmed`는 참고용으로 계속 계산됨 |
| (확정됐다 풀렸다 반복) | 3회 연속 같은 공간이어야 확정: 중심 0.15 m, 방향 2°, 길이 0.20 m 안 | 지도 흔들림, 구조물 끝 점이 들쑥날쑥 |

구조물 후보 조건(`observe_two_structure_gap`, 기본값은 `config/two_structure_test.yaml`):

- 처음 진행방향 기준 **왼쪽**, 옆 0.55~6 m, 앞뒤 -5~35 m 안의 점유 셀만 사용합니다. 오른쪽 물체는 무시합니다.
- 점유 셀을 약 0.10 m 간격으로 묶어 덩어리를 만듭니다. 덩어리 크기가 3 m를 넘으면 벽으로 보고,
  벽에서 도로 쪽으로 `pair_structure_protrusion_m`(0.15 m) 이상 튀어나온 부분만 구조물로 분리합니다
  (`split_wall`). 벽이 휘었거나 진행방향과 10° 넘게 기울면 분리하지 않습니다. 벽 위치는 같은 덩어리에서
  벽 높이의 점이 더 많아야 제대로 추정됩니다.
- 두 구조물 사이 뒤쪽에 벽(긴 벽, 또는 1 m 이상 벽 조각)이 입구에서 `slot_width` 안에 있으면 공간 안쪽 끝을
  벽에 맞춥니다(`rear_depth_m`, 창의 "뒤 벽까지"). 이때 공간은 도로 쪽으로 나오므로 도로 폭이 충분해야 합니다.
- 덩어리는 점유 셀 3개 이상이어야 합니다. 구조물이 낮거나 LiDAR 높이에 안 걸리면 안 보입니다.
- 앞뒤 순서로 이웃한 두 덩어리가 쌍이 됩니다. 두 구조물보다 0.15 m 이상 뒤에 있는(오목한) 후보는 건너뛰고,
  입구 쪽에 있는 물체가 사이에 있으면 쌍을 만들지 않습니다.
- 두 구조물 도로 쪽 면을 이은 방향이 진행방향과 10° 안이어야 합니다.
- 간격은 `pair_min_gap_length`(3.30)~`pair_max_gap_length`(6.0) m이고, 차체 길이 + 0.36 m보다 좁으면 거부합니다.

내부 조건(`observed_interior`):

- 공간 사각형(길이 = 간격, 깊이 = `slot_width` 1.10 m)에서 경계로부터 0.10 m 안쪽 셀을 검사합니다.
  - 점유 셀: 그 셀이 속한 점유 덩어리(8방향 연결, 공간 밖 부분 포함)가 `pair_max_speck_cells`(2) 이하면 노이즈로 허용, 넘으면 거부.
  - 미관측 비율이 `pair_max_unknown_fraction`(0.10)을 넘으면 두 구조물의 간격은 확정하고
    미관측 셀을 막힌 상태로 두어 경로계획에서 검사합니다.
  - 허용된 점·미관측이 주차 자세 차체 + 0.18 m 안에 있으면 검출은 통과해도 6절의
    `no bay-contained goal has a safe reverse entry`로 실패합니다(계획 격자는 미관측을 막힘으로 봄).
  - `parking_stop_policy: observed_safe`를 쓰면 vendor의 별도 내부 검사(`observed_stop.observed_interior`,
    미관측·점유 0개 요구)가 정지 조건에 추가됩니다. 기본값은 `legacy`라 해당 없음.
- 흔한 원인:
  - 벽 없는 열린 공간: 반사가 없으면 미관측(-1)으로 남습니다. `/scan`의 열린 방향이 `inf`인지 확인합니다.
    `lidar.yaml`의 `zero_range_is_no_return: true`와 state_estimator의 `no_return_free_range`(3.0 m)가 필요합니다.
    `nan`으로 나오면 드라이버가 0.0이 아닌 다른 값으로 무반사를 표시하는 것이니 `normalize_scan`을 확인합니다.
  - 뒤 벽이 짧은 조각(1 m 미만)으로만 보임: 벽으로 인정되지 않아 공간 내부 점유로 거부됩니다.
    차가 공간 정면을 지나며 벽을 길게 관측하게 합니다.
  - 바닥 반사·노이즈 점, 공간 안의 작은 물체(콘, 케이블 등).
  - 구조물이 LiDAR를 가려 공간 안쪽에 그림자(미관측)가 생김. 차가 공간 정면을 지나며 관측해야 채워집니다.
  - SLAM 지도 보정으로 누적된 무반사 빈 공간이 어긋남(state_estimator 재시작으로 초기화).

카메라 조건(`camera_confirms_pair`):

- 보정 파일(`calibration_file`)이 있어야 하며, 영상과 스캔의 시각 차이는 0.10 s 이하여야 합니다.
- 영상 후보 하나와 대응된 스캔 점의 80% 이상이 **한 구조물에만** 0.20 m 안으로 붙어야 그 구조물이 확인됩니다.
  한 후보가 두 구조물에 걸치면 확인이 안 됩니다. 두 구조물이 모두 확인되어야 합니다(다른 프레임이어도 됨, 180 s 유효).
- 현재 영상 검출은 주황 라바콘/흰 차체 모양 임계값 기반입니다(딥러닝 아님). 다른 재질·색은 `detector_params` 조정이 필요합니다.
- 확인: `/parking_vision/objects_overlay`(영상 위 후보와 투영 점), `/parking_vision/associations`.

## 6. 정지·재확인 (STOP_AND_STABILIZE, DETECT_TARGET)

| 증상 | 원인 후보 |
|---|---|
| FAILED `observed stop not stationary after existing stabilization interval` | (observed_safe 정책) 10 s 뒤에도 속도 0.06 m/s 이상. 모터가 0 명령에도 밀림 |
| EMERGENCY_STOP `unsafe observed-stop current footprint` | 정지 위치 차체가 지도 점유와 겹침 |
| FAILED `gap revalidation timed out; no fixed target fallback` | 정지 후 30 s 안에 같은 공간이 3회 다시 확정되지 않음. 정지 위치에서 공간이 가려지거나 카메라 시야를 벗어남 |
| `Waiting for map target: no bay-contained goal has a safe reverse entry` (로그) | 공간 안에 차체를 넣을 목표 자세가 없음. 목표 차체 + 0.18 m 여유 안에 점유/미관측 셀이 있거나 공간 앞쪽 7 m 도로가 막힘 |

## 7. 경로 계획 (PLAN)

계획 격자는 미관측을 막힘으로 봅니다. 깊이 관측이 허용치 안인 목표에 한해 **공간 사각형 안의 미관측은 빈 공간으로 보고**
(`pair_bay_unknown_as_free`), **공간 안쪽 끝보다 깊은 곳을 가상 벽으로 막습니다**
(`pair_block_behind_bay`, `pair_rear_block_margin_m`; `TwoStructureParkingTest.grid`).

| 증상 | 원인 후보 / 확인 |
|---|---|
| FAILED `candidate search exhausted: ...` | 모든 목표 후보에서 경로 없음. 간격이 좁음(합성 시험: 3.0 m 실패, 3.3 m 전환 7회, 3.6 m 전환 1회), 도로 폭·앞쪽 공간 부족, 미관측 영역이 넓음 |
| `latest RAW candidate precheck rejected: minimum=...` | 경로가 최신 지도에서 0.18 m 여유를 못 지킴 |
| FAILED `planned swept path failed collision/articulation audit` | 계획 결과가 재검사에서 충돌 |
| FAILED `planned path exceeds 30-degree articulation safety limit` | 꺾임각 30° 초과 경로 |
| 계획이 오래 걸림 | `planner_timeout`(parking.yaml 90 s) × 후보 수. 경로가 없으면 수 분 걸릴 수 있음 |

확인: `blocked_debug_directory/planning_input_NNN.npz`를 불러 `data`(가상 벽 포함 격자), `start`, `goal`을 그려 봅니다.
오프라인 재현은 `tests/test_two_structure_rear_block.py`의 `plan_with_wall`과 같은 방식으로 `install_b()` → `HybridAStar(...).parking_plan(...)`을 호출합니다.
`libnative.so`(optimizer/native.cpp)가 빌드돼 있어야 합니다.

## 8. 실행 직전·실행 중 (READY, EXECUTE)

| 증상 | 원인 후보 |
|---|---|
| READY에서 멈춤 | `auto_execute`/`execute_parking` false, 또는 위치 시각이 0.6 s 이상 늦음 |
| EMERGENCY_STOP `streams stale before execution` | 실행 직전 센서 끊김 |
| EMERGENCY_STOP `current actual footprint overlaps latest RAW map before execution` | 현재 차체가 최신 지도와 겹침(지도 변화, self filter 부족) |
| FAILED `final planned footprints outside measured target` | 경로 끝 자세가 공간 사각형을 벗어남 |
| `latest RAW full-path/current clearance audit failed` | 실행 직전 최신 지도로 전체 경로 재검사 실패 → 후보 재시도/재계획 |
| EMERGENCY_STOP `pose too old for closed-loop parking` | 실행 중 위치 시각 지연 |
| EMERGENCY_STOP `current actual footprint overlaps latest RAW occupied/unknown cell` | 실행 중 차체가 점유/미관측 셀에 닿음. 가상 벽도 여기에 포함됨. 경로 추종 오차가 크면 공간 안쪽 끝을 넘으려다 멈출 수 있음 |
| EMERGENCY_STOP `{name} collision reported` | 충돌 토픽 보고 |
| FAILED `replan budget exhausted: ...` | 재계획 `max_replans`(3)회 초과. 추종 오차, 지도 변화 |
| 실제로 움직이지 않음 | `enable_actuation:=false`(기본값, 명령만 발행), 테스트 노드의 `test_motion_enabled` false, Arduino 포트/스케일/극성 |

## 9. 테스트 창 관련

| 증상 | 원인 후보 |
|---|---|
| `ROS 연결 실패: ...` | ROS가 source되지 않은 채 창을 엶 → `tools/run_two_structure_gui.sh` 또는 바로가기로 실행 |
| 창에는 아무 값도 안 나오는데 터미널에서는 토픽이 보임 | 창과 터미널의 `ROS_DOMAIN_ID`/`ROS_LOCALHOST_ONLY`가 다름. 창 입력칸 값 확인 후 창 재시작 |
| 한글이 네모로 보임 | `fonts-noto-cjk` 미설치 |
| `서비스가 없습니다` | launch가 아직 안 떴거나 죽음. 창 아래 로그의 `process has died`/Traceback 확인 |
| `[창 표시 오류, 계속 동작: ...]` | 창 그리기 오류. 버튼·정지는 계속 동작. 메시지를 그대로 보고 |

## 9-1. 빌드·환경

- mvsim_slam_real_bringup의 `setup.py`가 `launch/*`를 통째로 복사하기 때문에, pytest를 돌린 뒤 생긴
  `src/mvsim_slam_real_bringup/launch/__pycache__`가 있으면 colcon build가 실패합니다. 빌드 전에 지웁니다.
- `ydlidar_ros2_driver`는 `third_party/ydlidar_sdk_install`(README의 SDK 빌드)과 `source tools/g6_env.bash`가 필요합니다.
- 계획기는 `optimizer/libnative.so`가 필요합니다. symlink-install이면 src 쪽 `native.py` 옆에서 찾습니다.
- 테스트: `PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 -m pytest tests -q`.
  `test_manual_motor_bridge`는 `ROS_DOMAIN_ID=223`을 요구하므로 221에서는 실패합니다(기존 동작).
  scipy, tkinter(python3-tk)가 필요합니다.

## 10. 알려진 한계

- 3 m 안의 검은색·광택·투명 물체는 무반사로 빈 공간이 될 수 있습니다.
- 무반사 빈 공간은 map 좌표로 누적되므로, SLAM이 크게 보정되면 어긋납니다(노드 재시작으로 초기화).
- 내부 허용치(작은 점, 미관측 10%) 안에 실제 작은 물체가 숨을 수 있습니다. 최종 안전은 계획·실행의 차체 충돌 검사에 의존합니다.
- `allow_lidar_only_motion:=true`에서는 카메라의 구조물 오인 방지가 없습니다. 왼쪽의 기둥·박스 같은 물체 쌍도 공간으로 확정될 수 있습니다.
