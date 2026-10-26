# G6 12Hz Cartographer 포인트 시간 경고 진단

2026-10-17 / Ubuntu 22.04 / ROS 2 Humble / YDLidar SDK 1.2.20

**최종 결과: C — 실제 측정 시각을 보존하는 근본 수정은 미확정.** 경고가 발생하는 소프트웨어 조건은 확인했고, 실제 경고 18회·제외 포인트 85개를 레이별로 정확히 재현했다. 그러나 현재 출력에는 진짜 레이별 측정 시각이 없고, 획득 순서와 각도 bin 시간 모델도 다르다. 안전한 시간 복원을 입증하지 못했으므로 SDK·드라이버·Cartographer 동작 수정은 적용하지 않았다. 경고는 해결되지 않았다.

진단·125초 시험·회귀 검사·보고서는 완료했다. **근본 해결 목표 기준 진행률은 95%로 남긴다.** 단순 경고 감소를 성공으로 처리하지 않는다. 기존 12Hz·18K·15m, IMU/odometry 미사용 설정과 지도·알고리즘·제어 코드를 보존했다. 마지막 LiDAR/SLAM 프로세스는 모두 정상 종료했고 포트를 해제했다.

## 1. 기존 자료 확인과 보존

요청한 `G6_12HZ_PERFORMANCE_REPORT.md`와 다음 파일이 실제로 존재함을 확인하고 먼저 분석했다:

```text
verification/g6_12hz/stationary_repeat_65s.json
verification/g6_12hz/stationary_repeat_65s.samples.json
verification/g6_12hz/stationary_repeat_hardware.log
verification/g6_12hz/timing_warning_analysis.json
verification/g6_12hz/stationary_comparison.json
```

주의: 기존 `timing_warning_analysis.json`은 이전 수동 이동 포함 시험의 6회·67포인트 분석이다. 최종 고정 재시험 13회·95포인트의 직접 증거로 혼용하지 않았다. 고정 재시험은 해당 `stationary_repeat_*` 파일들에서 다시 분석했다.

변경 전에 다음 archive에 성공 소스·SDK·도구·보고서·기존 검증 자료를 백업했다:

```text
backups/g6_timestamps/20261017_211112/successful_12hz_sources_sdk_and_evidence.tar.gz
```

기존 소스·도구·SDK 425개 파일 SHA-256은 `verification/g6_timestamps/baseline_hashes.json`에 기록했다. SDK Git 상태도 기존 로컬 수정이 없음을 확인했다. 이번에는 기존 파일을 수정하지 않았다.

## 2. 경고의 정확한 의미와 출력 위치

실제 설치 패키지:

- `ros-humble-cartographer`: `2.0.9004-1jammy.20260226.010834`
- `ros-humble-cartographer-ros`: `2.0.9002-1jammy.20260915.225849`

해당 릴리스의 소스와 다운로드 provenance SHA-256을 검증해 `verification/g6_timestamps/sources/`에 보존했다. ROS 1 master만 보고 결론을 내리지 않았다.

정확한 출력은 [range_data_collator.cc](verification/g6_timestamps/sources/range_data_collator.cc)의 `RangeDataCollator::CropAndMerge()`, 69~84행이다. 조건은 다음과 같다:

```text
현재 포인트의 절대 시각 < 이미 처리한 구간의 끝(current_start_)
```

이 조건에 해당하는 앞쪽 유효 포인트를 제거하고, 그 개수를 경고에 출력한다. **거리값 중복·USB 패킷 중복을 직접 검출한 경고가 아니다.** 같은 거리라도 시각이 정상이라면 이 경고의 원인이 되지 않는다.

이번 구성은 range sensor 1개이므로 이전 메시지의 마지막 유효 포인트 시각이 다음 처리 구간의 시작이 된다. 현재 유효 포인트가 그 시각보다 이르면 제거된다. 등호인 시각은 이 `<` 조건으로 제거하지 않는다.

공식 릴리스 근거: [Cartographer 2.0.9004 collator](https://github.com/ros2-gbp/cartographer-release/blob/release/humble/cartographer/2.0.9004-1/cartographer/mapping/internal/range_data_collator.cc), [ROS bridge 2.0.9002 변환](https://github.com/ros2-gbp/cartographer_ros-release/blob/release/humble/cartographer_ros/2.0.9002-1/src/msg_conversion.cpp). 로컬 복사본의 검증 결과는 `verified_source_provenance.json`이다.

## 3. G6 원시 패킷과 SDK 시간 경로

[SDK 통신 문서](third_party/YDLidar-SDK/doc/YDLidar-SDK-Communication-Protocol.md)의 일반 측정 패킷은 header, CT, LSN, 시작/끝 각도, checksum, 거리 샘플로 구성된다. 일반 패킷에는 각 레이의 하드웨어 측정 시각이 없다. SDK에는 다른 장치 등을 위한 별도 PH3 timestamp 패킷 파서도 있으므로 “SDK 전체가 타임스탬프를 전혀 지원하지 않는다”는 뜻은 아니다.

실제 코드 흐름:

1. `YDlidarDriver::parseData()`가 USB-UART bytes를 읽고 패킷을 파싱한다(`src/YDlidarDriver.cpp`, 약 540~724행).
2. 별도 timestamp가 없으면 `Pack` 생성자가 `getTime()`을 호출한다(`src/YDlidarDriver.h`, 455~464행).
3. `parsePoints()`는 패킷의 모든 포인트에 같은 `data.stamp`를 복사한다(`src/YDlidarDriver.cpp`, 1622행). **포인트별 개별 하드웨어 시각이 아니다.**
4. Linux `getTime()`은 `CLOCK_REALTIME` 또는 `gettimeofday`의 host wall clock이다(`core/base/timer.cpp`). timeout용 `getms()`의 monotonic clock과 구분해야 한다.
5. 이번 1,458개 header는 모두 millisecond 경계에 정렬되지 않았다. 별도 PH3 시각을 ms→ns로 직접 쓴 값과 다르며 위 host fallback 경로와 일치한다. 레이별 하드웨어 timestamp를 관측했다는 주장은 하지 않는다.

12Hz는 장치 회전 제어에 반영됐고 18K는 샘플링/point-time 관련 코드에 반영된다. 그러나 이것만으로 패킷 처리 지연이나 각 레이의 절대 획득 시각이 알려지는 것은 아니다. UART serialization, USB buffering, 스레드 scheduling, 파싱 시간이 host 시각에 섞일 수 있다. 개별 기여분은 이번에 독립 측정하지 못했다.

## 4. SDK의 스캔 시간 계산 문제

`CYdLidar::doProcessSimple()`를 추적했다. `F_n`을 현재 스캔 첫 패킷의 시각, `L_n`을 마지막 패킷 시각, `N_n`을 SDK 원시 포인트 수로 두면 현재 출력은 다음과 같다:

```text
header.stamp_n = F_n
scan_time_n = L_n - L_(n-1)
time_increment_n = scan_time_n / N_n
```

관련 위치: `third_party/YDLidar-SDK/src/CYdLidar.cpp` 약 599~618행.

`scan_time` 자체는 회전 간 간격의 추정치일 수 있다. 하지만 **현재 첫 패킷 기준 시각에, 이전·현재 끝 패킷 사이 간격을 현재 포인트 수로 나눈 값**을 붙이면 현재 프레임 내부 레이 시각과 일치한다는 보장이 없다. 회전별 packet count·USB 수신/처리 지터가 달라지면 현재 프레임의 추정 끝이 다음 프레임 첫 시각을 넘어갈 수 있다.

SDK는 앞부분에서 `m_PointTime`, 수신 지연과 연속성을 이용해 `tim_scan_start/end`도 계산하지만, 현재 `outscan.stamp`와 `time_increment`는 그 값 대신 위 패킷 시각 경로를 사용한다. 해당 보조 계산에는 연속성 clamp도 있으므로 이를 무조건 출력으로 연결하는 것은 사용자가 금지한 인위적 시각 이동이 될 수 있다. 적용하지 않았다.

기존 자료의 `scan_time_n - (F_n-F_(n-1))`는 약 **-3.806~+2.983ms**로 변했다. 이번 자료도 약 **-3.940~+3.150ms**였다. 두 시간 구간이 실측상 동일하지 않다. 다만 이 차이가 곧 실제 레이의 물리 시각 오차라는 뜻은 아니다.

header 시각은 두 시험 모두 단조 증가했고 `use_sim_time=false`였다. ROS `/clock` 오용이나 관측된 시스템 시각 역행으로 설명할 증거는 없다.

## 5. ROS LaserScan의 의미와 획득 순서 손실

[LaserScan 정의](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/LaserScan.msg)의 header는 **첫 레이 측정 시각**이며 각 레이는 `header + index × time_increment`로 해석된다.

공식 ROS 드라이버는 SDK 포인트를 다음처럼 각도 bin에 넣는다:

```text
bin = ceil((point.angle - angle_min) / angle_increment)
ranges[bin] = point.range
```

반면 `/point_cloud`는 SDK의 원래 포인트 순서로 발행하며 `stamps` channel은 SDK ordinal index 기반 시간 모델을 담는다. 이번에는 둘을 변경하지 않고 함께 관측했다.

32개 같은 stamp의 cloud/scan을 비교한 결과:

- 획득 순서에서 bin index 감소: **24,591회**
- bin index 증가: **53회** (wrap·각도 보정 등의 영향 포함)
- bin index 시간 모델과 SDK ordinal 시간 모델 차이: **-67.309~+55.438ms** 범위

즉 현재 `inverted=true` 등의 변환 뒤에는 획득 순서와 양의 bin index 순서가 대체로 반대이다. 일정 각도 간격의 LaserScan으로 재배열하면서 획득 순서 정보가 그대로 보존되지 않는다. **이 수십 ms는 두 모델 사이의 차이이며, 외부 기준으로 측정한 실제 시각 오차가 아니다.** SDK의 `/point_cloud` channel도 하드웨어 ground truth가 아니다.

최소한 이 문제까지 해결하지 않고 `time_increment`만 줄여 경고를 없애면 올바른 motion compensation을 보장할 수 없다. `inverted/reversion`을 바꿔 시간 문제를 우회하면 이미 검증한 좌표 방향을 바꾸므로 적용하지 않았다.

## 6. Cartographer의 변환·정렬·누적 과정

실제 릴리스 소스 [msg_conversion.cc](verification/g6_timestamps/sources/msg_conversion.cc), [sensor_bridge.cc](verification/g6_timestamps/sources/sensor_bridge.cc), [local_trajectory_builder_2d.cc](verification/g6_timestamps/sources/local_trajectory_builder_2d.cc)를 확인했다.

1. 범위 밖/무효 ray를 제외한다.
2. 남은 ray의 상대 시각을 **원래 배열 index × time_increment**로 계산한다.
3. 마지막 **유효** ray 시각을 메시지 끝 시각으로 삼고 상대 시각을 음수~0으로 옮긴다. 전체 bin의 마지막 시각과 다를 수 있다.
4. `num_subdivisions_per_laser_scan=1`. subdivision 끝 시각이 이전보다 증가해야 한다.
5. 메시지 끝 시각은 내부 ordered queue로 정렬된다. 끝 시각이 증가해도 메시지 내부 앞쪽 포인트가 이전 구간에 들어갈 수 있다.
6. `RangeDataCollator`가 위의 이전 구간 포인트를 제거한다. 이번 경고가 발생하는 지점이다.
7. 그 뒤 extrapolator·거리 처리·누적·voxel filtering·scan matching으로 진행한다. `num_accumulated_range_data=1`이다.

따라서 경고의 **직접 발생 위치는 Cartographer 내부 collator**, 입력 시각이 만들어지는 경로는 **SDK 패킷 시각과 ROS LaserScan 재구성**이다. 복수 LiDAR 충돌이나 IMU 대기 때문은 아니다. 누적 개수를 바꾸는 것은 경고 뒤 단계여서 근본 해결이 되지 않는다. subdivision을 바꾸어도 원래 포인트의 절대 시각은 그대로이므로 해결을 보장하지 않는다.

## 7. 기존 자료 분석과 신규 레이별 검증

기존 고정 64.887초 자료:

- 양수 bin 시간 중첩 **18회**, 정수 ns 계산 최대 **918,412ns ≈ 0.9184ms**.
- 정상 운용 경고 **13회·95포인트**. 13개 경고 모두 중첩 경계의 수신 시점과 일치.
- 기존 요약에 있는 약 0.918627ms와의 미세 차이는 epoch 전체를 float 초로 계산하는 것과 정수 ns 계산의 차이다. 측정 데이터를 보정한 것이 아니다.
- 원래 기록에는 ranges 전체와 유효 index가 없어 나머지 5개 경계의 정확한 유효 포인트 판정은 소급 확정할 수 없다.

신규 시험은 전체 ranges를 압축 JSONL로 보존했다. 관측 노드가 Cartographer의 float32 연산·100ns 단위 변환을 재현한 결과, **18개 경고 각각의 포인트 수가 실제 로그와 모두 일치했다(합계 85개)**.

예: 신규 zero-based scan **23**:

| 값 | epoch ns |
|---|---:|
| 이전 전체 bin 끝 추정 | 1792239311701440190 |
| 이전 마지막 유효 ray 시각 | 1792239311701440100 |
| 다음 header 시각 | 1792239311701334000 |
| 다음 첫 유효 ray(index 1) 시각 | 1792239311701389700 |

첫 유효 ray가 이전 유효 끝보다 **50,400ns** 이르므로 포인트 1개가 제거된다. 실제 로그도 `Dropped 1 earlier points`이다. 이는 원시 패킷이 중복됐다는 증거가 아니라 **재구성한 시각이 이전 처리 구간에 들어갔다는 직접 증거**다.

전체 경고의 시점·스캔 번호·이전 끝·다음 시작·유효 index·예측/실제 제외 개수는 [validated_timestamps.csv](verification/g6_timestamps/validated_timestamps.csv), 기존 18개 경계는 [before_boundary_analysis.csv](verification/g6_timestamps/before_boundary_analysis.csv)에 있다.

## 8. 검토한 수정 후보와 채택하지 않은 이유

| 후보 | 검증·판단 |
|---|---|
| `time_increment=1/18000` 고정 | 기존 기록 중첩 **18→323회**, 신규 기록에서는 603회. 실제 호스트 anchor와 일치하지 않아 기각 |
| 다음 scan 시작을 이전 끝 뒤로 clamp | 경고는 피할 수 있어도 근거 없이 시각을 이동하므로 금지·미적용 |
| 다음 header까지 span을 줄이는 lookahead | 다음 수신 시각에 맞추는 것만으로 실제 ray 시각이 되지 않으며 획득 순서도 해결하지 못함 |
| time_increment=0 | 모든 레이를 같은 시각으로 만들어 deskew 정보를 없애므로 금지·미적용 |
| 초기 ray 제거, 각도/거리 마스킹 | 유효 주변 데이터를 잘라 경고를 회피하므로 미적용 |
| subdivision/누적 개수 변경 | 원래 시간 모델을 고치지 못하며 경고 수만 달라질 수 있어 미적용 |
| SDK의 보조 tim_scan_start/end 사용 | 새 파서의 패킷 시각 의미와 실제 시간 검증 없이 기존 연속성 clamp를 채택할 수 없어 미적용 |
| 획득 순서의 timed PointCloud2 사용 | 방향/비균일 각도와 레이별 시각을 함께 담는 데 적합한 후보. 그러나 현재 SDK ordinal 시간도 모델값이어서 신뢰할 시간 근거부터 필요. 이번 인터페이스는 변경하지 않음 |

적용한 코드는 **read-only 관측·오프라인 분석 도구 3개뿐**이다:

```text
tools/g6_timestamp_analysis.py
tools/g6_timestamp_probe.py
tools/g6_timestamp_validate.py
```

ROS 센서/제어 데이터를 발행하지 않고 timestamp를 변경하지 않는다. 원본 SDK, ROS 드라이버, Cartographer, Launch, YAML, Lua 변경은 **0개**다. 따라서 이하 결과는 “수정 후 개선 성능”이 아니라 **동일 설정의 확장 진단 결과**다.

## 9. 실제 12Hz 125초 시험과 경고 비교

기존 by-id 경로를 사용하고 포트 동시 접근을 방지했다. 장치 모델/serial을 다시 확인했다. 사용자가 알려준 USB_DATA 전원 상태를 유지했고 USB_PWR 보조 전원을 추가하거나 배선을 바꾸지 않았다. 전압·전류를 외부 계측한 것은 아니다. LiDAR 본체 고정을 요청한 상태에서 시험했으며 차량 구동·조향·자율주차는 실행하지 않았다.

| 항목 | 기존 고정 기록 | 동일 설정 확장 진단 |
|---|---:|---:|
| 실수신 구간 | 64.887초 | **124.865초** |
| 실제 평균 주파수 | 11.6664Hz | **11.6686Hz** |
| LaserScan | 758 | **1,458** |
| 전체 bin 수 최소/중앙/최대 | 1537/1541/1547 | **1535/1541/1547** |
| 유효 ray 수 최소/중앙/최대 | 755/781/805 | **719/782/832** |
| 유효 비율 | 50.618% | **50.794%** |
| bin/s 추정 | 17986.1 | **17986.2** |
| 유효 LaserScan ray/s 추정 | 9104.5 | **9135.9** |
| 최대 수신 간격 | 89.02ms | **90.30ms** |
| 양수 bin 중첩(엄격한 >0) | 18회 | **34회** |
| 1µs 초과 중첩 | 18회 | **30회** |
| 최대 중첩(정수 ns) | 0.918412ms | **0.836203ms** |
| 정상 계측 경고/제외 포인트 | 13회/95개 | **18회/85개** |
| 경고/s | 0.20035 | **0.14416** |
| 제외 포인트/s | 1.46408 | **0.68074** |
| 드라이버 scan 실패/재연결 | 0/0 | **0/0** |

기존 performance probe의 `positive_scan_time_overlap_count`는 **1µs 초과**를 센다. 이번 엄격한 정수 ns 재계산은 양수 경계 34개이며, 그중 1µs 이하 4개를 제외하면 기존 기준의 30개와 일치한다. 서로 다른 기준을 숨기거나 혼용하지 않았다. 이전 18개는 모두 1µs보다 커 차이가 없다.

신규 34개 중 **18개**는 실제 유효 포인트가 이전 구간에 들어가 경고가 났고, **16개**는 전체 bin의 모델상 경계가 겹쳐도 실제 유효 포인트 제거는 없었다. 레이별 예측과 로그가 일치한다.

전체 실행 로그는 경고 **21회·111포인트**다. 관측 전 crop 경고는 0회, 정상 계측 구간 18회·85개, 계측 뒤 저장/종료 구간 3회·26개로 구분한다. 초기 velocity queue 준비 경고는 crop 경고와 별개로 1회다. checksum은 시작 시 6개 로그이며 이후 반복 없음, 연결은 1회였다.

정규화한 경고율·제외율은 낮아졌지만 **동작 수정이 없으므로 개선 효과로 주장하지 않는다.** 패킷 scheduling·관측 부하·장면·발생 변동 등이 남아 있다. 이 숫자를 B 판정이나 시간 동기화 정확도 향상의 근거로 사용하지 않았다.

## 10. SLAM 품질·CPU·회귀 검사

| 항목 | 기존 | 확장 진단 |
|---|---:|---:|
| matching 출력 | 758 | **1,459** |
| 지도 발행 / 서로 다른 내용 | 66/62 | **126/96** |
| 지도 해상도 | 0.05m | **0.05m 유지** |
| 스캔 시각 TF 조회 성공/실패 | 649/1 | **1249/0** |
| 최대 정지 xy 추정 변화 | 1.83cm | **1.65cm** |
| 상대 yaw 범위 | -0.42~+0.69° | **-0.54~+0.56°** |
| Cartographer 평균 CPU(1코어=100%) | 7.28% | **6.40%** |
| Cartographer 최대 CPU | 25.96% | **32.00%** |
| 비정상 종료 | 0 | **0** |

신규 matching 출력 1개는 첫 수신 scan stamp보다 앞선 stamp로, 비동기 관측 시작 경계에 포함됐다. scan 1,458개와 matching 메시지 1,459개를 중복 측정 포인트 증가로 해석하지 않는다.

기존처럼 유효 측정·matching·지도·TF·정지 pose 추정은 유지됐다. RViz에서 실제 지도·scan과 `Global Status: Ok`를 직접 확인했고 [화면](verification/g6_timestamps/rviz_final.png)을 저장했다. 외부 pose ground truth 대비 절대 정확도나 이동 deskew 검증은 아니다. CPU peak는 관측 길이도 다르므로 단순 열화로 단정하지 않는다. 상세 관측 도구 자체도 CPU 부하가 있으므로 정밀한 무부하 성능 비교로 주장하지 않는다.

- `colcon build --symlink-install`: **12 packages finished [2.78s]**.
- 기존 `pytest tests`: **20 passed [0.51s]**.
- SDK·기존 소스·도구 425개 hash 일치. 원본 174개·보호 복사본 103개 최종 hash 검사 통과.
- 새 도구의 포인트 제거 예측: 실제 18회·85개와 개별·합계 모두 일치.
- 새 PGM/YAML/pbstream 저장 성공. 기존 지도·보고서·검증 파일은 보존했다.

## 11. 확정 사항·미확정 사항과 다음 검증

**확정:** 경고의 직접 조건·코드 경로, 기존 경고와 시간 중첩의 대응, 신규 모든 제외 포인트 수 재현, host 패킷 시각에 기반한 시간 재구성, SDK 획득 순서와 각도 bin 시간 모델의 불일치.

**미확정:** 레이저 실제 발광/측정 시각과 host 파싱 시각 사이 지연, 모델별로 검증된 sample-clock 기준, 레이별 정확한 상대 시각 복원, USB/스레드/회전 변동 각각의 기여, 실제 시간·geometry를 동시에 보존하는 수정의 효과. raw packet 중복이나 손실률을 입증할 sequence counter 기록도 없다.

근본 해결을 주장하려면 제조사로부터 패킷/샘플 시각의 정의와 신뢰할 sample-clock/latency 근거를 확보하거나 외부 기준으로 시간 모델을 검증해야 한다. 그 후 획득 순서·각도 보정을 함께 보존하는 native timed 데이터 경로를 별도 시험할 수 있다. 현재 LaserScan의 단일 header와 균일 time_increment만으로 이미 사라진 레이별 시각을 정확히 복원했다는 주장은 불가능하다.

따라서 **A 아님, 수정 효과가 없으므로 B 아님, 보수적으로 C**다. 경고의 소프트웨어 직접 원인이 전혀 모른다는 뜻은 아니며, 실제 측정 시각을 보존하는 근본 원인 모델·수정은 미확정이라는 구분이다.

## 12. 복구 및 보존

동작 코드를 바꾸지 않아 **현재 복구 조치는 필요 없다.** 기본 `g6_slam.launch.py`는 그대로 12Hz·18K·15m를 실행한다. 원본 시뮬레이션, Hybrid A*, LQR, Arduino·serial, IMU, 구동 코드 변경·실행 없음. Git commit/push 없음. 진단 자료와 새 지도는 `verification/g6_timestamps/`에만 추가했다.

향후 변경으로 설정 복구가 필요한 경우, 모든 관련 Launch를 종료한 뒤 백업에서 두 설정 파일만 추출하여 복구할 수 있다:

```bash
cd ~/mvsim_slam_real
G6_TS_RESTORE=$(mktemp -d "$HOME/mvsim_slam_real/backups/g6_timestamps/restore_XXXXXX")
tar -xzf backups/g6_timestamps/20261017_211112/successful_12hz_sources_sdk_and_evidence.tar.gz \
  -C "$G6_TS_RESTORE" \
  src/mvsim_slam_real_bringup/config/g6.yaml \
  src/mvsim_slam_real_bringup/config/g6_lidar_only.lua
cp -a "$G6_TS_RESTORE/src/mvsim_slam_real_bringup/config/g6.yaml" \
  src/mvsim_slam_real_bringup/config/g6.yaml
cp -a "$G6_TS_RESTORE/src/mvsim_slam_real_bringup/config/g6_lidar_only.lua" \
  src/mvsim_slam_real_bringup/config/g6_lidar_only.lua
source tools/g6_env.bash
colcon build --symlink-install
```

이번에는 이 복구를 실행하지 않았다. 현재 파일은 이미 백업과 동일하다.

## 13. 실행·재분석 명령과 산출물

터미널 1 — 원래 SLAM 실행:

```bash
cd ~/mvsim_slam_real
source tools/g6_env.bash
export ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1
ros2 launch mvsim_slam_real_bringup g6_slam.launch.py \
  lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  enable_rviz:=true > verification/g6_timestamps/manual_hardware.log 2>&1
```

터미널 2 — 본체를 고정하고 read-only 125초 관측:

```bash
cd ~/mvsim_slam_real
source tools/g6_env.bash
export ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 tools/g6_timestamp_probe.py --duration 125 \
  --output verification/g6_timestamps/manual_125s.json
/usr/bin/python3 tools/g6_timestamp_analysis.py \
  --samples verification/g6_timestamps/manual_125s.samples.json \
  --log verification/g6_timestamps/manual_hardware.log \
  --output verification/g6_timestamps/manual_boundary_analysis.json
```

터미널 1의 출력은 새 시험용 `manual_hardware.log`에 기록하고 터미널 2는 같은 파일을 분석한다. 저장된 `hardware.log`는 이번 보고서 시험용으로 보존한다. 새 manual 시험을 반복할 때는 이전 manual 자료를 먼저 별도 보존한다. CPU도 기록하려면 `--pid label:PID`로 실제 프로세스 PID를 지정한다. 위 간단 명령에는 CPU PID를 넣지 않았으므로 CPU 항목이 비어 있다.

이번 자료를 그대로 재검증하는 명령:

```bash
cd ~/mvsim_slam_real
source tools/g6_env.bash
/usr/bin/python3 tools/g6_timestamp_analysis.py \
  --samples verification/g6_timestamps/live_125s.samples.json \
  --log verification/g6_timestamps/hardware.log \
  --output verification/g6_timestamps/recheck_boundaries.json
/usr/bin/python3 tools/g6_timestamp_validate.py \
  --samples verification/g6_timestamps/live_125s.samples.json \
  --summary verification/g6_timestamps/live_125s.json \
  --log verification/g6_timestamps/hardware.log \
  --output verification/g6_timestamps/recheck_valid_points.json
```

실제 실행했던 PID와 CPU 기록은 원시 samples에 들어 있다. 새 시험의 지도 저장은 기존 보고서의 `map_saver_cli`/`write_state` 명령을 사용할 수 있다. 필요한 지도를 저장한 뒤 **실행한 Launch 터미널에서 Ctrl+C**로 종료한다. 이번 실행에서는 네 프로세스 clean exit와 `Lidar has stopped!`, `fuser` 미점유를 확인했다.

주요 증거:

- [최종 통계](verification/g6_timestamps/live_125s.json), [metadata·CPU·pose](verification/g6_timestamps/live_125s.samples.json), [전체 ranges 압축 기록](verification/g6_timestamps/live_125s.ranges.jsonl.gz)
- [경고 개별 재현](verification/g6_timestamps/validated_timestamps.json), [CSV](verification/g6_timestamps/validated_timestamps.csv), [전후 비교](verification/g6_timestamps/comparison.json)
- [실제 실행 로그](verification/g6_timestamps/hardware.log), [실제 파라미터](verification/g6_timestamps/live_parameters.yaml)
- [새 지도 YAML](verification/g6_timestamps/maps/g6_timestamp_diagnostics.yaml), [PGM](verification/g6_timestamps/maps/g6_timestamp_diagnostics.pgm), [pbstream](verification/g6_timestamps/maps/g6_timestamp_diagnostics.pbstream)
- [빌드](verification/g6_timestamps/build.log), [기존 테스트](verification/g6_timestamps/tests.log), [소스 provenance](verification/g6_timestamps/verified_source_provenance.json)
