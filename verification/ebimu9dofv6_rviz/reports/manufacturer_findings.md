# 공식 자료 조사 (2026-10-19)

출처:

- 제품: https://www.e2box.co.kr/entry/AHRS-EBIMU-9DOFV6
- 매뉴얼: https://www.e2box.co.kr/entry/EBIMU-9DOFV6-%EB%A7%A4%EB%89%B4%EC%96%BC
  공식 PDF를 evidence/manufacturer_rev10.pdf에 저장했다. 아래 상세 사항은 이 PDF에 근거한다.
- ROS 2: https://www.e2box.co.kr/entry/EBIMU-9DOF-ROS2-Package-1
- Python: https://www.e2box.co.kr/entry/EBIMU-9DOF-파이썬python-예제-1
- ROS 메시지 규약은 설치된 /opt/ros/humble/share/sensor_msgs/msg/Imu.msg를 직접 확인했다.
  docs.ros.org 페이지는 접근 차단되어 근거로 사용하지 않았다.

| 항목 | 제조사 명시 사항 / 근거 |
|---|---|
| 전원 | VCC 3.3~6V, GND 0V (4~5쪽) |
| 핀 | 1 VCC, 2 GND, 3 TX 출력, 4 RX 입력, 5 nPD/nRST; 물리 핀 방향은 4쪽 그림 참조 |
| UART 레벨 | TX/RX는 VCC와 같은 레벨. CP2102 보드의 허용 입력·실제 VIO는 별도 확인 필요 (5~6,35쪽) |
| 연결 의미 | 센서 TX는 호스트 RX, 센서 RX는 호스트 TX, 공통 GND. 이번 작업에서 배선 변경 안 함 |
| 전기 정격 | RX high ≥2.5V, low ≤0.5V; TX high ≥VCC-0.5V, low ≤0.4V (35쪽) |
| UART 기본 | 115200, 8N1, 하드웨어 flow control 없음 (5쪽) |
| 출력 기본 | ASCII(soc1), Euler(sof1), 주기 sor10 = 10ms; gyro/accel/mag/distance/temp/timestamp 기본 off (14~15쪽) |
| 자동 작동 | pons1 기본: 전원 인가 시 작동; 실제 현재 설정이 기본과 같다고 가정할 수 없음 (19,34쪽) |
| ASCII | `*` 시작, 쉼표 구분, CR LF 종료. 이 형식에 CRC/checksum 항목은 정의되지 않음 (12쪽) |
| Binary | SOP 0x5555, 각 항목 16bit 2의 보수, 마지막 2byte CHK. CHK는 SOP 포함 모든 byte 합, overflow 무시 (13쪽) |
| Binary 척도 | Euler /100, Quaternion /10000, gyro /10; 채널에 따라 패킷 길이가 달라짐 (21쪽) |
| Polling | sor0일 때 호스트 `*`마다 현재 자세 한 번 출력 (8,20쪽) |
| Euler | Roll,Pitch,Yaw / degree; 범위 ±180, ±90, ±180 (21쪽) |
| Quaternion | z,y,x,w 순서, ASCII 소수 4자리 (12,21쪽) |
| 각속도 | gx,gy,gz, DPS. ROS 변환시 π/180 배 (21쪽) |
| 가속도 | soa1: 중력 포함 body g; soa2: local 중력 제거; soa3: global 중력 제거; soa4/5는 속도 (22쪽) |
| Timestamp | sots1일 때 ms 추가 (24쪽) |
| 출력속도 | sor1..1000 = 1..1000ms, 1~1000Hz. baud/패킷 길이 제한시 설정값보다 느려질 수 있음 (7,20쪽) |
| 명령 | `<` + command + data + `>`; `<ver>`, `<cfg>` 등. cfg 종료에는 추가 `>` 필요 (8,14,34쪽) |
| 설정 저장 | sb/sor/soc/sof/sog/soa 등 설정은 내부 비휘발성 메모리에 저장된다고 명시. 이번 도구는 모두 송신하지 않음 (20~24쪽) |
| 좌표 | 9쪽 Euler/heading 그림과 10쪽 gyro/accel/mag 그림 직접 확인. 그림의 축 방향이 다르며 이를 동일한 ROS ENU라고 단정할 수 없음 |

전기 측정 및 배선 실물 확인은 수행하지 않았다. 전원·GND·센서 TX 신호·nPD/nRST 상태·호스트 RX 허용전압은
RX=0이 재현될 때 사용자와 제조사 도면에 근거해 확인할 대상이며, 현재 배선 변경을 지시한 것이 아니다.

공식 Python 예제는 readline 후 `*` 확인·쉼표 분리이며 ROS 메시지/좌표계 검증은 하지 않는다.
기존에 내려받은 공식 ROS 2 패키지 ebimu_pkg는 std_msgs/String `/ebimu_data`를 발행한다.
해당 Publisher는 포트를 두 번 open하는 코드가 있고, Imu 변환·공분산·시각화 기능은 없다.
이번 작업에서 공식 예제나 기존 드라이버를 수정하지 않았다.

기존 프로젝트 src/ebimu_driver/ebimu_driver/protocol.py는 이미 Quaternion z,y,x,w 순서와
accel/timestamp 옵션을 지원한다. 진단 파서는 기존 399개 실제 패킷의 Quaternion과 일치했다.
별도 도구를 만든 이유는 기존 제어 토픽과 분리하고 원시 수신량·없는 필드·통신 중단을 기록하기 위해서다.
센서 Quaternion의 active/passive 회전 의미와 물리 축 일치는 실제 회전으로 검증해야 하며,
문서의 순서와 정규화만 확인한 것을 회전 방향 합격으로 확대하지 않았다.
