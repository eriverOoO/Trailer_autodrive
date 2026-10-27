# 검증 기록 (2026-10-27)

- 벽 없는 합성 점유지도 및 원시 스캔 단위 테스트 9개 통과.
- ROS Humble에서 합성 TF/scan/map을 진단 콜백에 입력: 시간에 따른 후보 확인, JSONL 기록, 물리 기준점 라벨 기록 경로, stale scan 시 후보 억제 통과. `tools/wallless_ros_smoke.py`로 재현 가능. 실제 DDS 구독을 통한 end-to-end 센서 실험은 아니다.
- 요약 도구 실행: 1회 방문은 반복성 미측정으로 표시됨을 확인.
- real_parking C++/Python 패키지 빌드 통과. bringup 설치 시 glob이 __pycache__ 디렉터리를 포함하던 문제를 파일 필터로 수정.
- 기존 전체 tests는 생성된 interfaces_pkg, 소스에서 요구하는 libnative.so, 일부 Python GUI 의존성이 없어 수집 단계에서 중단. 전체 회귀 통과로 보고하지 않는다.
- 실제 G6/IMU 연결, 로비 매핑, 차량 이동, 모터 명령 전송, 실차 주차는 수행하지 않음.

테스트 환경의 시스템 Python에 scipy가 없어 기존 `/home/didgang1203/Trailer_autodrive/.venv-parking-linux/bin/python`을 사용했다. ROS 관련 검증에는 `/opt/ros/humble/setup.bash`를 source했다.
