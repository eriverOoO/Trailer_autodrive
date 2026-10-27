# 벽 없는 실차 LiDAR 검증

이 브랜치는 차량을 구동하지 않는 관측 도구다. 기존 AutoParking의 ㄷ자 탐지기·Hybrid A*·제어기는 변경하지 않는다. `/wallless/status`와 RViz `/wallless/markers`로 오른쪽 두 물체 사이 공간을 별도로 검증한다. 초록 박스는 관측 조건을 만족한 기하학적 후보이며 주차 실행 승인이나 충돌 없는 주차 경로를 뜻하지 않는다.

## 준비와 실행

1. 실제 두 차량을 고정하고 우리 차의 LiDAR 스캔 높이에서 윤곽이 잡히는지 확인한다. 차축·힌지·센서 실측 및 장착 TF는 기존 절차를 따른다. ROS Humble, numpy, scipy, Cartographer가 필요하다.
2. 워크스페이스에서 `source /opt/ros/humble/setup.bash`, `colcon build --symlink-install --packages-select real_parking mvsim_slam_real_bringup`, `source install/setup.bash`를 실행한다. 이 두 패키지 이외의 센서/SLAM 의존성은 먼저 빌드되어 있어야 한다.
3. 기존 G6 SLAM을 실행한다. 예: `ros2 launch mvsim_slam_real_bringup g6_slam.launch.py lidar_port:=/dev/lidar frame_mode:=vehicle enable_rviz:=true`. `/dev/lidar`는 예시이므로 실제 포트로 바꾼다. vehicle 모드는 실측 완료된 `g6_mount.yaml`이 필요하다. sensor 모드를 사용하면 아래 진단에도 `ego_frame:=laser_frame`을 지정해야 하며, 결과는 차축/차체 기준 위치가 아니다.
4. 별도 터미널에서 `ros2 launch mvsim_slam_real_bringup wallless_validation.launch.py output_file:=/tmp/wallless-run.jsonl`을 실행한다. RViz에 MarkerArray `/wallless/markers`를 추가하고 Fixed Frame을 map으로 둔다. `ros2 topic echo /wallless/status`로 오류와 후보를 확인한다. 이 launch는 SLAM이나 모터를 시작하지 않는다. 수동 운전은 기존에 확인된 수단으로 별도 수행한다.
5. `ros2 bag record -o /tmp/wallless-bag /scan /tf /tf_static /map /tractor/imu/data /trailer/imu/data /scan_matched_points2 /wallless/status /wallless/reference_label`로 원본 데이터를 남긴다. 서로 다른 실험에는 다른 출력 경로를 사용한다.

## 현장 절차

벽을 추가하지 않고 출발점·중간점·주차 접근점에 바닥 표시를 한다. 같은 센서 위치와 같은 차체 방향으로 정차할 수 있도록 표시한다. 저속 왕복 및 주차 진입/후진 경로를 반복한다. 각 표시점에서 완전히 정차한 뒤 예를 들어 `ros2 topic pub --once /wallless/reference_label std_msgs/msg/String '{data: start}'`를 보낸다. 같은 실제 점에는 같은 이름을 쓴다. 초기 자세가 동일한지 사람이 확인해야 한다.

`python3 tools/summarize_wallless.py /tmp/wallless-run.jsonl`은 같은 기준점 방문 사이 최대 위치·방향 차이, 신선도 실패 횟수, 공간 후보 관측 횟수를 출력한다. 재방문이 없는 기준점은 반복성을 측정했다고 표시하지 않는다. 반복성은 절대 정확도가 아니다. 줄자/바닥 기준으로 실측한 차체 위치와도 비교한다. 허용 위치·방향 오차는 주차 여유와 트랙터·트레일러 끝단 이동량에서 정한다. 맵 중복 윤곽, TF 점프, 가려짐, 전방/후진 각각의 실패 구간을 rosbag/RViz와 대조한다. 신선한 TF만으로 scan matching의 정확도가 입증되지는 않는다.

오프라인 분석은 기록 당시 Cartographer가 만든 /map과 /tf를 그대로 재생한다: `ros2 bag play /tmp/wallless-bag --clock`과 진단 launch의 `use_sim_time:=true`. 이때 실시간 SLAM/센서 TF 발행기는 종료해 중복 발행을 피한다. raw /scan만으로 SLAM을 다시 실행하려면 별도 sim-time Cartographer 설정이 필요하며 이 launch는 그 작업을 수행하지 않는다.

## 인식 방법과 한계

- 원시 스캔은 인접 빔 거리와 거리별 간격으로 분할한다. invalid 빔을 가로질러 연결하지 않는다. scan frame의 클러스터 개수·중심·크기를 상태에 기록한다.
- 지도에서는 점유 셀의 연결 성분을 추출하고 크기로 긴 벽/작은 잡음을 거른다. 차량 전방 오른쪽 후보만 대상으로 한다. 길이 방향은 현재 차량 방향을 prior로 사용하므로 주차열과 평행한 접근 구간에서 검증한다. 큰 회전 중 차량 판별을 보장하지 않는다.
- map 좌표의 중심을 서로 다른 map timestamp에서 연관시켜 반복 관측한다. 새 지도 3회 및 2초 이상을 기본 조건으로 한다. **누적 지도에서의 지속성은 실제 물체의 정지 증명이 아니다.** 이동체의 잔상·지도 보정도 영향을 주므로 현장에서는 두 차량을 실제로 고정하고 실시간 scan 클러스터/원본 bag과 비교한다.
- 두 후보의 정렬·간격을 검사하고 사이 영역을 실제로 관측된 free 셀로 확인한다. unknown을 free로 채우거나 뒤쪽 가상 벽을 만들지 않는다. 내부 90% free 및 점유 샘플 0을 요구한다. 신선도가 끊기면 유효 후보 출력을 중단한다.
- 벽이 없으면 주차 깊이를 뒤쪽 경계로 측정할 수 없다. `bay_width`는 차량 치수에 기반한 설정값이며 출력에도 configured_prior로 명시한다. 설정한 깊이의 관측 영역에 대한 가설이다.
- 박스·의자·다른 물체도 비슷한 단면을 만들 수 있다. vehicle_sized_static_candidate는 의미적 차량 분류가 아니다. 보이지 않는 면의 전체 차체 크기나 트레일러 회전 여유도 입증하지 않는다.
- 점유도가 낮아 차량이 여러 조각으로 나뉘거나 서로 붙으면 후보를 놓칠 수 있다. 붙은 셀을 공격적으로 합쳐 free 근거를 만들지 않는다. 기록된 스캔으로 임계값과 물체 크기를 조정해야 한다.

설정: `src/mvsim_slam_real_bringup/config/wallless.yaml`. 기본 후보 치수는 약 0.99 × 0.55m 전동차를 대상으로 넓게 둔 시작값이다. `min_gap=2.6m`는 길이 2.19m인 일자 차량의 기본 후보 필터일 뿐 주차 가능 최소 길이를 보장하지 않는다.

현장에서는 정상 두 차량, 한 대만 있음, 빈 공간에 작은 장애물 있음, 사람 통과, 차량 위치 변경, 가려짐, unknown 영역, 센서 입력 중단을 각각 기록한다. 단순한 박스 두 개도 후보가 될 수 있음을 확인하고 차량 분류 정확도로 보고하지 않는다. 실제 시험 결과는 아직 없다.
