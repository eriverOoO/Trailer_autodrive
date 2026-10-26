# Software verification — 2026-10-16

하드웨어 연결·보정·펌웨어 업로드·모터/조향 구동·실차 주차 테스트는 수행하지 않았다. 아래 PASS는 소프트웨어 검사에 한정된다.

| 기능 | 구현 / 보존 | 빌드 | 소프트웨어 검증 |
|---|---|---|---|
| LiDAR 인터페이스 | 새 gateway / 선택 RPLidar source, 원본 보존 | PASS | stamp/angle/unknown beams, 합성 scan→Cartographer PASS |
| Tractor IMU | EBIMU 전용 UART/ASCII | PASS | 프레임/단위/회전/covariance/독립 topic PASS |
| Trailer IMU | 같은 드라이버 독립 port | PASS | 두 센서 분리/gyro 수신/누락 처리 PASS |
| Arduino Adapter | 별도 신규 노드 | PASS | 단위/부호/범위/rate/timeout/E-stop/출력 OFF PASS |
| Original Serial Sender | 패키지 전체 원형 보존 | PASS | 메시지/문자열/소스 hash PASS; 포트 열기·송신 미실시 |
| Cartographer | B LiDAR-only Lua 보존 | 외부 Humble 바이너리 로딩 PASS | 합성 scan map/pose PASS; 선택 IMU Lua 로딩 PASS, 실제 fusion 미검증 |
| Tractor-Trailer Estimator | 실제 rear-axle 변환 + 원형 GyroYaw | PASS | gyro epoch/wrap/gap/pairing/TF 및 센서 loss PASS |
| Gap Detector | 최종 B 원형 | PASS | U 경계·공간 크기·내부 장애물 거부 PASS |
| Hybrid A* | 최종 B 원형 + native build | PASS | 전진/후진/invalid start/clearance/native 일치/steering-priority/CDE 실제 jobs PASS |
| LQR | 원형 Controller | PASS | 전진/후진/종점 완료/단위 PASS; 독립 시작·종료 PASS |
| 통합 Launch | 새 Bringup | PASS | 기본 OFF / 9개 프로세스 / 합성 입력 / 정상 종료 / 개발모드 actuation 거부 PASS |
| 비전 코드 | 전체 코드·리소스 원형 보존 | PASS | hash PASS; torch 없음/모델 미준비로 YOLO 런타임 미검증 |

- `verification/build_final.log`: `colcon build --symlink-install`, 11 packages finished [3.20s], 실패 없음.
- `verification/tests.log`: 20 tests passed [0.49s].
- `verification/default_no_hardware.json`: 93 invalid state messages, WAIT_FOR_LOCALIZATION, zero preview, hardware commands 0. 시작/종료 모두 오류 없음.
- `verification/synthetic_integration.json`: 명시적 합성 입력으로 실제 Cartographer 실행. 278 valid states, 18 occupied maps, 278 tracking path messages. 원형 AutoParking SEARCH_FORWARD 진입 및 Adapter preview 확인. 입력 중단 후 EMERGENCY_STOP / preview [0,0,0]. hardware commands 0.
- `verification/optional_imu_config.json`: optional Lua·tracking frame 구성 로딩, 입력 없는 대기 및 정상 종료 PASS. 가속도 fusion 실제 측정 검증은 하지 않았다.
- `verification/individual_nodes.json`: gap_detector/hybrid_astar/lqr 설치된 정식 엔트리 시작·정상 종료 PASS.
- `verification/actuation_rejected.log`: development_mode에서 enable_actuation=true 실행 거부 확인.
- `verification/preservation.json`: 원본 baseline 174개 파일, 보호 복사본 103개 파일 SHA-256 일치 PASS. Serial sender 패키지/메시지/Arduino 스케치/비전/LiDAR 원형 보존. 원본 임시 colcon-discovery 로그는 세션 자체 생성물임을 확인해 제거하고 증거를 신규 verification/initial_colcon_discovery.log에 보관했다. 원본 소스는 수정하지 않았다.
- `verification/algorithm_preservation.json`: vendor/optimizer 20개 소스 byte-identical PASS. GyroYaw도 원본 그대로 복사했다. 공유 라이브러리는 복사된 C++ 소스를 -O3로 재빌드했다.
- RViz YAML 파싱과 토픽/프레임 구성 확인. GUI 렌더링은 별도 검증하지 않았다.

해결한 문제: symlink-install 공유 라이브러리 경로, reliable LaserScan 구독 호환성, CDE supervisor 예산 파일, 실제 TF 발행 주기의 gyro pairing 버퍼, shutdown context publish 오류, static-only 노드 signal wakeup. 모두 신규 래퍼에서 수정했고 보호된 Arduino/serial/알고리즘 소스는 바꾸지 않았다.

확인하지 못한 내용: 실제 UART 수신/전송, LiDAR 모델·범위·회전 방향·beam timing, EBIMU 실제 축/자세 정확도/시간 jitter/드리프트, 모터·조향 부호/응답/변환계수, 실측 기구학, 실제 SLAM·fusion·주차 성능, RViz GUI 화면, 비전 모델 추론. 합성 정적 맵 검사는 실차 전체 주차 완료 검사가 아니다.

기존 한계: 펌웨어 receive watchdog/encoder/독립 cutoff 없음; optional YOLO torch/모델 미준비; 기존 setup의 lidar_obstacle_detector_node 및 traffic_light_detector_node 모듈 없음. 기본 핵심 실행과 빌드는 이들로부터 분리했다. 원본 World/ROS·Python 설치 환경 변경 및 git push 없음.
