# 자동 안정화 및 초기 대비 IMU 회전량

요청: 시작 시 두 차량은 같은 방향. 라이다·두 IMU가 안정된 후 출발하고, IMU는
SLAM 융합 없이 각 센서의 초기값 대비 좌우 변화량만 계산한다.

구현:
- 기존 2D 시각화의 YawTrack/Alignment를 확장하고 별도 relative_yaw 프로파일을 추가.
- 5초간 신선한 scan/map/scan-time TF 및 시간 정합된 두 IMU, 낮은 gyro,
  1도/2cm 이내 안정 조건을 확인. 독립 yaw 평균을 기준으로 기록.
- ±180도 경계를 연속적으로 처리. 같은 초기 지도 방향에 각 센서의 변화량을 더해 표시.
- 트레일러 180도 장착은 sensor_to_body_rpy에서 반영. yaw 변화량의 부호를 추가 반전하지 않음.
- 물리 차축·힌지 원점으로 외곽선을 배치. 기존 원본 렌더러/스타일은 유지.
- /vehicle_viz_2d/ready Bool 및 status의 ready/초기대비 각도 필드.
- 수신 누락·각도 점프 시 준비 해제. 기준이 소실되면 명시적 reset 전 자동 재영점 없음.
- manual_slam_check.py --require-ready는 준비 전/신호 stale 시 속도 0 출력.
- 새 통합 launch는 use_imu=false, frame_mode=vehicle, 공통 TF 발행기 1개 사용.
- IncludeLaunchDescription 사이 params_file 이름 충돌은 scoped GroupAction으로 격리.

검증:
- 루트 tests 및 2D 시각화 tests 합계 86개 통과 (합성 단위 테스트).
- 실제 G6/트랙터 IMU/트레일러 IMU로 관찰용 launch 실행, 모터 노드 없음.
- 실제 첫 성공 실행: 프로세스 시작 약 9.28초 후 5초 안정화가 끝나 READY.
- 238 status/marker 배치, READY 61개 수신. 첫 표시 변화량은 tractor +0.0047°,
  trailer -0.0345°, phi -0.0393°. 이는 정지 잡음 관찰이며 물리 각도 정확도 검증이 아님.
- 실제 ROS graph에서 Cartographer가 두 IMU 토픽을 구독하지 않음을 확인.
- 최초 실행은 launch 인자 충돌로 IMU 프로파일 대신 G6 YAML을 읽어 NOT_READY가 됨.
  원인을 수정하고 재실행하여 PASS. 최초 결과는 first_run/에 보존.
- 실기 시험 후 이번에 시작한 프로세스 종료, 포트 해제 확인.

남은 검증:
- 기존 좌우 부호 sign=-1은 과거 진단 설정을 유지한 것. 과거 CW/CCW 최종 기록은
  FAIL_OR_INCONCLUSIVE로, 이번 정지 시험으로 물리 방향/각도 정확도를 확정하지 않음.
- 센서 fused yaw 자체의 자기장 영향/드리프트, 실제 좌·우 회전량은 수동 회전 시험 필요.
- IMU 높이·roll·pitch는 차체의 화면 회전으로 사용하지 않지만, 센서 자체 기울기가
  제공 yaw의 해석에 미치는 영향까지 제거했다고 주장하지 않음.
- 기존 자율주차 상태추정기·제어·구동 확인 플래그에는 적용하지 않은 관찰용 경로.
