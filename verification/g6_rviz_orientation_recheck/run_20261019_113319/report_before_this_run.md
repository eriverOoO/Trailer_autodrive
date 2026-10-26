# YDLIDAR G6 좌표계·RViz 시각화 재진단 보고서

**상태: 진행률 25% / 사용자 센서 고정 확인 대기. 새 정지 시험은 아직 시작하지 않았다.**

이 문서는 준비 및 기존 자료 분석 단계의 중간 기록이다. 새 정지 SLAM 또는 표적 시험 성공을 보고하는 최종 결과가 아니다.

## 1. 현재 확인한 실행 환경과 보존 상태

- G6 USB 장치는 `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`로 보인다. 전원/케이블 구성을 변경하지 않았다.
- 재진단 시작 시 G6 드라이버, Cartographer, RViz 및 이전 진단 계측 프로세스는 모두 종료된 상태였다. 다른 프로젝트의 프로세스를 종료하지 않았다.
- 기존 G6 설정 파일: 12Hz, sample_rate=18, range_min=0.28m, range_max=15m, baudrate=512000, frame_id=laser_frame, inverted=true, reversion=false.
- sensor 모드에서는 Cartographer의 tracking_frame 및 published_frame이 laser_frame이고, map_frame=map, odom_frame=odom이다. IMU와 odometry 사용은 false이다.
- g6_slam.launch.py는 현재 구조에서 저장 PBStream을 불러오는 인자를 전달하지 않는다. 새 프로세스 실행 시 기존 메모리 상태를 이어받지 않는 구성이며, 실제 새 프로세스 실행은 고정 확인 후에 수행한다.
- g6.rviz는 Fixed Frame=map / TopDownOrtho Angle=0이고, g6_front_up.rviz는 Fixed Frame=map / Target Frame=map / Angle=−π/2이다. Launch 기본값은 g6.rviz이며 직전 실제 시험은 g6_front_up.rviz를 명시했다. 새 시험에서도 실제 인자와 런타임 값을 별도로 기록해야 한다.
- 이전 진단 시작 때 기록한 기존 파일 264개의 SHA256를 현재와 비교한 결과 변경 없음. 이번 재진단에서도 원본 코드·Lua·YAML·RViz·TF·URDF 및 별도 시뮬레이터는 수정하지 않는다.
- 기존 성공 지도, PBStream, 로그, 보고서는 삭제하거나 덮어쓰지 않았다. commit/push 또는 차량 구동·조향 실행은 하지 않았다.

자료 보존 및 새 시험 계획: `verification/g6_rviz_orientation_recheck/`.

## 2. 이전 센서 이동 영향 — 참고자료이며 새 정지 기준이 아님

이전 자료 `diagnostics/g6_rviz_20261018_210809/`에는 사용자 고정 확인이 없다. 사용자는 당시 센서가 실제로 움직였을 가능성을 추가로 알렸다.

이전 첫 관측의 map→laser_frame yaw 중앙값은 약 +0.0153°, 뒤 관측은 약 −6.6416°로 약 −6.6569° 차이가 있다. 이는 지도와 센서 사이의 상대 자세가 달라졌다는 기록이다. 실제 센서 회전, SLAM 추정 변화 또는 둘의 혼합을 이 데이터만으로 분리할 수 없다. 따라서 이 수치를 고정 장착 오프셋 또는 정지 yaw 드리프트로 단정하지 않는다.

map을 Fixed Frame으로 사용하는 경우 센서가 이동/회전하면 센서 축과 실시간 스캔의 화면 방향이 바뀌는 것이 정상이다. Cartographer가 추정한 센서 자세와 pose graph 보정도 map→laser_frame 관계에 반영된다. IMU/odometry가 없어도 LiDAR만으로 SLAM은 가능하지만, 빠른 움직임·스캔 중 움직임·불충분한 주변 형상은 추정 불확실성과 지도 변형을 늘릴 수 있다. 과거 그림의 형태가 다르다는 이유만으로 TF 오류나 정확도 저하를 확정하지 않는다.

기존 지도와 새 지도는 환경, 센서 위치, 관측 시간과 범위가 달라질 수 있으므로 형상의 시각적 차이를 직접적인 정확도 비교로 해석하지 않는다.

## 3. 물리 방향·ROS 프레임·카메라 구분

제조사 G6 Data sheet V1.6의 2.6절은 회전 중심을 원점으로, PH2.0-5P 케이블 출구 방향을 0°, 위에서 본 시계 방향을 양수로 정의하며 개체별 ±3° 차이를 명시한다. [제조사 데이터시트](https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20G6%20Data%20sheet%20V1.6%28211230%29.pdf)

현재 로컬 SDK의 reversion=false는 추가 180° 회전을 하지 않고, inverted=true는 각도 부호를 바꾸어 ROS의 반시계 방향 양수 관례에 대응한다. 이는 정적 코드에서 예상되는 대응 관계이며 현재 실물 케이블 방향과 ROS 0°를 실측한 결과가 아니다.

sensor 모드의 예정 TF 경로는 map→odom→laser_frame이다. 이 모드에서 base_link→laser_frame 정적 TF를 새로 만들지 않는다. 실제 새 세션의 부모/자식 및 RPY는 계측 전이므로 아직 보고할 수 없다.

지도 기준 센서 yaw를 ψ, 카메라 Angle을 φ라고 하면 센서 +X의 화면 위쪽 대비 방위 차이는 wrap(ψ−φ−90°)이다. φ=−90°이면 화면상 센서 축 편차는 ψ와 같다. 여기에 물리 케이블과 ROS 축 사이에 실측된 차이가 있다면 별도로 더해진다. 따라서 화면상 약 30°만으로 센서 TF 보정값을 결정할 수 없다.

## 4. 빨간 점·지도 출처 — 이전 런타임에서 확인된 사실

아래는 **이전 진단 결과**이며 새 고정 시험 결과로 재사용하지 않는다.

| 항목 | 이전 실제 적용값/관찰 |
|---|---|
| 빨간 거리점 | rviz_default_plugins/LaserScan, /scan, sensor_msgs/msg/LaserScan |
| 검정·회색 지도 | rviz_default_plugins/Map, /map, nav_msgs/msg/OccupancyGrid |
| LaserScan 표시 | Flat Squares, 0.03m, Alpha=1, Decay Time=0 |
| LaserScan 색상 | Color Transformer=Intensity, Use rainbow=true; 수집 intensity는 전부 0 |
| Scan QoS | Best Effort / Volatile / Depth 10 / Filter size 10 |
| Map 표시 | Color Scheme=map, Alpha≈0.7, Draw Behind=false |
| Map QoS | Reliable / Transient Local / Depth 1 |
| Submap/Marker/PointCloud Display | 이전 런타임 구성에는 없음. Submap 데이터 발행과 Submap 표시 활성화는 별개 |
| 기타 빨간 표시 | TF 및 Axes의 +X 축도 빨강이므로 측정점과 구분해야 함 |

이전 화면 비교에서 Scan만 켜면 빨간 점이 있었고 Map만 켜면 빨간 점이 없었다. Intensity=0은 intensity=false 설정에서 관찰한 값이므로 이를 표적의 반사 강도가 약하다는 근거로 사용하지 않는다. Map 색상은 점유 확률에 따른 회색조이며 free/unknown/occupied를 실측 원시 값으로 구분해야 한다.

이전 A/B 화면은 서로 다른 시각의 실시간 데이터다. 빨간 픽셀 수 차이만으로 레이어 가림을 확정하지 않는다. 특히 그때 센서가 움직였을 가능성이 있으므로 표적이 있는 위치와 측정점의 대응을 다시 검사해야 한다.

## 5. 이전 표시 On/Off 시험에서 재현한 TF 시간 문제

이전 시험에서는 LaserScan Display를 껐다 켠 직후 스캔 끝 시각 TF의 future extrapolation 오류가 반복됐다. 같은 설정의 새 RViz에서는 약 43초 동안 시작 직후 오류 1회만 있었으나, 표시를 재활성화한 뒤 12초 동안 오류 13회가 추가됐다.

별도 이전 관측에서 스캔 시작 시각 TF는 268개 모두 수신 즉시 조회 가능했지만, 끝 시각 TF는 241개에서 즉시 조회할 수 없었다. 같은 끝 시각 TF는 30ms 뒤 268개 모두 조회됐다. 이 비율은 RViz의 실제 유실 프레임 비율과 동일하지 않다. RViz에는 자체 버퍼와 필터가 있기 때문이다.

이 사실은 현재 설치된 RViz 11.2.25에서 표시 재활성화 이후 시간 필터 동작을 따로 검토할 근거이다. 사용자가 원래 관찰한 장애물 현상의 원인이라고 확정하는 증거는 아니다. 새 시험에서는 초기 정상 동작, 런타임 표시 조작, 오류 시작 시각을 명확히 분리한다. 이전에 조작으로 유발한 오류를 센서 이동이나 장애물의 직접 영향으로 해석하지 않는다.

## 6. 새 정지 시험과 표적 시험 상태

**미시작. 사용자 고정 확인을 기다리는 중이다.**

계획: 새 Cartographer 프로세스/trajectory에서 70초를 관찰하고 초기 5초를 별도 처리하여 최소 60초 이상의 스캔 간격을 확보한다. 실제 수신 빈도, 처리량, 지도 발행, XY/yaw 추정 변동, TF, RViz 상태, 오류/경고를 기록한다. 고정 확인 없이는 정지 성공을 선언하지 않는다.

다음 표적 단계는 판 없음 → 케이블 전방 약 1m에 판 설치 → 판만 제거 순서이다. 매 상태마다 사용자 완료 확인을 받고 기록한다. 판은 실제 스캔 높이를 가로질러야 하고 케이블/노트북 조작이 센서를 움직이지 않아야 한다. 필요하면 후방·좌측·우측 표적을 추가한다.

표적이 설치된 방향의 새 근거리 반사점, 제거 후 근거리 반사점 소멸, 뒤쪽 배경 반사점 출현을 각도와 거리 수치로 비교한다. 실시간 Scan과 누적 Map의 잔존 점유를 구분한다. 아직 이 실물 실험은 수행하지 않았고, 목표물의 실제 각도와 보정 yaw는 미확정이다.

## 7. 요청한 9개 질문에 대한 현재 답변

1. **이전 이동 영향 가능성:** 있다. 상대 yaw 변화 기록과 사용자 진술이 있으나 실제 이동량은 계측되지 않았다.
2. **새 정지 SLAM 정상 여부:** 아직 계측하지 않아 판정 불가.
3. **현재 빨간 점 토픽:** 새 실행은 아직 없다. 이전 실제 런타임에서는 /scan LaserScan으로 입증했다.
4. **지도 Display:** 이전 실행은 /map OccupancyGrid를 표시하는 Map Display였다. 새 런타임에서도 대조 예정.
5. **실시간 표시 정상 여부:** 이전에는 실제 빨간 스캔을 확인했으나 재활성화 후 TF 시간 오류도 재현했다. 새 정지 및 표적 시험 판정은 보류.
6. **케이블과 ROS +X 일치:** 문서와 설정상 예상되지만 실제 표적 실측이 없어 미확정.
7. **약 30° 원인:** TF/카메라/지도 프레임/센서 이동을 독립적으로 측정하기 전에는 확정할 수 없다. 기존 수치에서 고정 30°를 입증한 자료는 없다.
8. **정확한 수정 파일·방법:** 현재 변경할 원본 파일은 선정하지 않았다. 후보는 사용 RViz 파일의 Color Transformer를 FlatColor, Color를 255;0;0, Decay Time을 0으로 명시하는 표시 설정이다. Map Draw Behind는 가림 재현 여부를 보고 판단한다. 이것만으로 TF 시간 문제를 해결한다고 보장하지 않는다. TF yaw, inverted/reversion, 드라이버·Cartographer 변경은 실측 근거가 없어 제안하지 않는다.
9. **추가 검증:** 사용자 고정 확인 후 60초 이상 새 정지 계측, 판 설치/제거 확인, 전후좌우 방향 시험이 필요하다.

## 8. 변경 및 생성 파일

원본 변경 없음. 새 진단 폴더에 원본 설정 사본, SHA256 목록, 과거 분석, 상태 기록, 시험 계획 및 확인 전 실행을 막는 계측 스크립트만 준비했다. 최종 수정안 선정(95%)과 완료 보고(100%)는 새 계측 및 미확정 항목 정리 뒤에 수행한다.
