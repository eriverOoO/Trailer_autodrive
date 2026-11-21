# 전동차 근접 인식 시험

좌측 C920 영상에서 `ride_on_car` 한 클래스를 찾는 YOLO11n 객체 검출 실험입니다.
직사각형은 영상에서 실제로 보이는 차체를 표시합니다. 받침대는 배경이며, 화면 밖으로
잘린 차체는 보이는 부분까지만 라벨링했습니다.

## 데이터

- `sheets/`에는 내장 이미지 생성 도구로 만든 원본 시트가 있습니다.
- `annotations.json`에는 시트별로 직접 확인한 바운딩 박스와 분할이 있습니다.
- `prepare.py`가 시트를 개별 프레임으로 자르고 `dataset/`에 이미지와 YOLO 라벨을 씁니다.
- 푸시된 C920 사진은 학습 입력과 모델 선택용 검증에 포함하지 않았습니다. 다만 같은 촬영의
  한 장을 참조한 앞선 합성 예시가 생성 구도의 참고 자료였으므로, 결과는 이 한 번의 촬영에 대한
  오프라인 점검입니다. 별도 주행에서 일반화 성능을 확인해야 합니다.

생성 프롬프트 요약. 첨부 사진에서 흰색 AMG 전동차 차체를 참고하고, 합성 예시에서 C920의
낮은 시점과 근접 구도를 참고합니다. 3×3 배열의 각각 독립된 4:3 이미지에 접근하는 앞범퍼,
잘린 펜더와 문짝, 이탈 중 뒤쪽을 넣습니다. 조명과 벽, 바닥, 받침대 재질을 바꿉니다.
음성 예시는 전동차 없이 받침대, 흰 벽, 상자, 의자만 같은 카메라 높이에서 보이게 합니다.

## 재현

배포한 오프라인 시험 가중치는 `ride_on_yolo11n.pt`입니다. Git LFS를 사용하는
저장소에서 포인터만 받은 경우 `git lfs pull --include="artifacts/ride_on_detector/ride_on_yolo11n.pt"`로
가중치를 받습니다.

저장소 루트에서 Python 환경을 활성화합니다. 시험에는 Python 3.10.12,
PyTorch 2.6.0+cu124, Ultralytics 8.4.172를 사용했습니다.

```bash
curl -fL https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt \
  -o artifacts/ride_on_detector/yolo11n.pt
python artifacts/ride_on_detector/prepare.py
python artifacts/ride_on_detector/train.py --name replication
python artifacts/ride_on_detector/select_checkpoint.py \
  artifacts/ride_on_detector/runs/replication/weights
python artifacts/ride_on_detector/evaluate_real.py \
  artifacts/ride_on_detector/ride_on_yolo11n.pt
```

`yolo11n.pt`는 공식 COCO 사전학습 체크포인트입니다. SHA-256은
`0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`입니다.
합성 검증에서 신뢰도 0.25와
IoU 0.50을 적용해 F1이 가장 높은 체크포인트를 선택합니다. 동률이면 양성 검증 프레임에서
가장 잘 맞는 박스의 평균 IoU가 높은 모델을 사용합니다.
`evaluate_real.py`는 C920 대표 프레임 12장의 수동 박스를 기준으로 COCO의 `car` 출력과
전이학습한 `ride_on_car` 출력을 비교합니다. 기본 신뢰도는 0.25, 박스 판정은 IoU ≥ 0.50입니다.
결과는 `real_evaluation.json`과 비교 이미지에 기록됩니다.

## 결과

합성 프레임 72장 중 54장(차체 45장, 배경 9장)으로 학습하고,
나머지 18장(차체 9장, 배경 9장)으로 가중치를 골랐습니다.
RTX 4060에서 65에폭을 마쳤고, 최종 `ride_on_yolo11n.pt`는 합성 검증에서
차체 9장을 모두 찾았고 배경 9장에는 검출을 내지 않았습니다.

| C920 녹화 표본 12장 | 차체 박스 일치 (양성 11장) | 정답과 일치하지 않은 출력 박스 |
|---|---:|---:|
| YOLO11n COCO `car` | 2/11 | 0 |
| 전동차 전이학습 `ride_on_car` | 10/11 | 2 |

오검출 두 건은 렌즈가 크게 가려진 프레임입니다. 일부 차체가 남은 0018에는
가림을 포함한 큰 박스를 출력했고, 차체가 보이지 않는 0019에도 박스를 출력했습니다.
현장 적용 전에 전동차 없이 받침대와 배경만 보이는 별도 회차와 렌즈가 가려지는 장면이 필요합니다.

## 바퀴 장착 이미지 보강 모델

`ride_on_yolo11n_wheels.pt`는 기존 `ride_on_yolo11n.pt`에서 추가 학습한 모델입니다. 기존 합성 학습
이미지 54장에 `wheel_augmentation/`의 장착 이미지 16장을 더했습니다. ImageGen에서 학습용 시트 A/F의
구도와 차체를 유지하고 보이는 빈 휠하우스에 검은 타이어와 휠을 추가했습니다. 추가 이미지의 라벨은
장착한 바퀴를 포함한 보이는 차체 영역이며, 검증 시트 E 및 실제 C920 영상은 학습 입력에서 제외했습니다.

```bash
python artifacts/ride_on_detector/prepare.py --wheel-augmentation
python artifacts/ride_on_detector/train.py --wheel-augmentation --epochs 16 --device cpu
```

재학습 결과는 기본적으로 `runs/yolo11n_ride_on_ft_wheels/weights/best.pt`에 저장됩니다.
실차의 두 구조물 테스트 창에서는 [전동차 ONNX 모델 선택](../../mvsim_slam_real/tools/TWO_STRUCTURE_TEST.md#카메라-설치)을 사용합니다.

보강 모델은 Ultralytics 8.3.167과 PyTorch 2.7.1 CPU에서 16에폭을 학습한 `best.pt`입니다.
두 모델을 416px ONNX로 변환하고 같은 추론 코드에서 신뢰도 0.65로 비교했습니다.
두 모델 모두 저장 C920 양성 42장 중 41장(IoU ≥ 0.50)을 검출했고, 정답과 일치하지 않는 박스는
각각 1개였습니다. 학습에 사용하지 않은 바퀴 합성 비교본 두 장도 두 모델이 모두 검출했습니다.
바퀴를 장착한 실차에서의 성능은 별도 촬영으로 검증해야 합니다.

## 주차 파이프라인에서의 역할

LiDAR의 두 군집으로 구조물의 위치와 사이 간격을 계산합니다. 카메라 모델은 영상에서 전동차 후보
영역을 냅니다. 센서 보정 이후 LiDAR 점을 영상 위에 투영해, 겹치는 군집에 전동차 관측을 대응시킬 수
있습니다. 받침대가 스캔 평면 아래에 있으면 LiDAR는 그 평면에 걸친 차체를 관측합니다.
정지 상태에서 차체에 해당하는 점이 영상에서도 차체에 놓이는지 확인합니다.
이번 결과물은 영상 모델의 오프라인 시험입니다.

좌우·앞뒤·높이 오프셋과 카메라 방향은 외부 보정의 초기값으로 사용합니다.
실제 대응에는 카메라 내부 파라미터·왜곡과 LiDAR→카메라 회전·이동을 같은 해상도에서 적용합니다.
차량을 세우고 서로 다른 두세 위치에서 점이 같은 물체 표면에 놓이는지 확인합니다.
