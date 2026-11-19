# 손으로 밀며 카메라 물체 후보와 LiDAR 비교

외장 C920와 G6의 실제 ROS 토픽을 구독하는 도구입니다. 차량 제어 명령을 발행하지 않습니다.

```bash
cd ~/Trailer_autodrive/mvsim_slam_real
# 최초 1회: LFS 전동차 가중치에서 ONNX를 내보내고 별도 추론 환경 준비
bash tools/prepare_ride_on_runtime.sh
bash tools/run_sensor_compare.sh
```

센서 드라이버가 `/parking_vision/image_raw`와 `/scan`을 먼저 발행해야 합니다.
카메라는 reliable QoS, 스캔은 sensor-data QoS로 수신합니다.
인식은 작업 스레드에서 최신 이미지 한 장씩만 처리합니다. 창은 `LIVE`/`STALE`,
측정 FPS와 ROS 이미지 메시지의 나이를 표시합니다. 나이는 카메라 노출부터의 지연을
측정한 값은 아닙니다. 입력이 0.5초 이상 오래되면 `STALE` 표시로 바뀝니다.

왼쪽 초록 박스는 전동차 전용 `ride_on_yolo11n` 모델의 **ride_on_car 후보**입니다.
실제 주차 ROS 노드와 같은 ONNX Runtime 검출기를 사용하며 기본 신뢰도는 0.65입니다.
사용한 LFS 가중치는 `artifacts/ride_on_detector/ride_on_yolo11n.pt`이고, 내보낸 ONNX와
원본/내보낸 파일의 해시는 같은 디렉터리의 `.export.json`에 기록됩니다.
렌즈 가림·처음 보는 배경에서는 오검출할 수 있습니다.
역사적인 MobileNet-SSD 비교가 필요하면 `--detector-backend mobilenet_ssd`를 명시합니다.
그 가중치 준비는 기존 `tools/setup_sensor_compare_model.sh`를 사용합니다.

오른쪽은 센서 중심 반경 5m의 실제 2D 스캔입니다. 전방(+X)은 위, 왼쪽(+Y)은 왼쪽입니다.
인접 반사점 사이가 0.18m 이내인 점무리를 묶고, 세 점 이상인 무리에 L 번호와
가장 가까운 반사점의 거리(m)를 표시합니다. 화면이 겹치지 않도록 거리 표시는 일부만 보입니다.
L 번호는 추적 ID가 아니므로 매 스캔 바뀔 수 있습니다. 5m 밖의 점은 화면 밖에 있지만
원본 스캔을 저장할 때는 유지됩니다. G6는 한 높이의 단면을 관측합니다.

**카메라–LiDAR 공간 보정은 아직 없습니다.** C 번호와 L 번호는 자동 연결되지 않으며,
영상 위 점 투영이나 물체별 거리 확정값을 제공하지 않습니다.
영상 프레임과 헤더 시각이 100ms 이내인 스캔을 골라 비교하고, 없으면 경고하며
가장 최근 스캔을 별도로 표시합니다. 헤더 시각 근접은 하드웨어 동기화나 공간 대응의 증거가 아닙니다.
스캔 한 회전 중 차량이 움직이는 효과도 보정하지 않습니다.

`현재 영상·스캔 저장`을 누르면 `artifacts/sensor_compare/날짜_시각/`에
원본 `camera.png`, 후보 표시 `detections.png`, `observation.json`을 저장합니다.
JSON에는 후보 종류·신뢰도·픽셀 영역, 원본 이미지와 스캔 시각·프레임, 거리 배열과
각도·beam 시간 정보가 들어갑니다. 무한대/NaN 거리는 null로 기록합니다.
최신 영상과 시각이 가까운 스캔이 없으면 저장을 거부합니다. `calibrated`와
`automatic_object_association`은 false로 기록됩니다.

정지한 상태에서 전동차 차체가 실제 스캔 높이에 걸리도록 배치한 뒤 천천히 밀면서 영상 후보와 LiDAR 점무리가
같은 물체인지 육안으로 비교하고 장면을 저장합니다. 받침대의 점을 차체로 오인하지 않도록 확인합니다.

## 장비 없이 저장 데이터 재생

ROS domain 230을 이 재생 전용으로 사용하며, 라이브 장비/제어를 같은 도메인에서 실행하지 않습니다.
두 터미널에서 `source tools/g6_env.bash` 후 실행합니다.

```bash
# 터미널 1: 카메라와 시리얼 포트를 열지 않는 학습 모델 노드
ROS_DOMAIN_ID=230 ros2 launch mvsim_slam_real_bringup parking_vision_diagnostic.launch.py \
  detector_backend:=ride_on_onnx image_topic:=/sensor_replay/image_raw scan_topic:=/sensor_replay/scan
# 터미널 2: 원본 주기 약 20초, 이미지 43장과 스캔 232개
/usr/bin/python3 tools/replay_sensor_capture.py --domain-id 230
```

기록의 상대 시각 간격을 유지해 현재 ROS 시각으로 이동시킵니다. `/sensor_replay/*` 입력 토픽에만
발행하며 모터/주차 명령을 만들지 않습니다. 기록에서 inf와 NaN이 모두 null로 저장된 부분은
원래 종류를 복구할 수 없으므로 재생 시 NaN(유효하지 않은 거리)으로 취급합니다.
실측 `calibration_file`이 없으면 영상 후보만 나오고 `associations.valid=false`, `objects=[]`입니다.
가상의 보정값으로 이 실제 촬영을 보정했다고 표시하지 않습니다.
