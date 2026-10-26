# mvsim_slam_real — Progress

## 최신 IMU 소프트웨어/UART 진단 — 2026-10-16 23:29–23:31 KST

**진단 진행률 100% / 결과 UNRESOLVED / 실제 수신 총 0 bytes.** 허용된 읽기 전용 진단을 완료했다는 뜻이며 통신 성공이 아니다. 아래 과거 벤치 시험의 진행률보다 이 절을 우선한다. 기존 전체 프로젝트의 소프트웨어 구현 완료 기록은 그대로 보존한다.

| 단계 | 결과 |
| --- | --- |
| 0% 기존 로그/드라이버 | 기존 initial_query.json 및 수동 baud scan 재분석, 드라이버/조회 도구 확인 완료 |
| 20% 제조사 프로토콜 | Downloads의 PDF 3개 직접 확인, 추출 텍스트와 references의 바이트 일치 확인, p34 원본 그림 확인 완료 |
| 40% Linux 설정 | 실제 포트 fd의 termios/모뎀 상태, pyserial 구현, 서비스/커널 로그 확인 완료 |
| 60% 독립 스크립트 | tools/diagnose_ebimu.py 구현, 무송신 기본값·raw/HEX/ASCII/시각·자동 종료·예외/close 검증 완료 |
| 80% 실제 UART | 기존과 다른 초기 제어선 조건, cfg 조회 종료, 8개 baud의 조회/polling 시험 완료; 전부 0 bytes |
| 100% 결론/기록 | UNRESOLVED. 원인 확정 불가. 실제 ROS 센서 데이터 검증으로 진행하지 않음 |

### 제조사 문서로 확인한 프로토콜

공식 자료 목록: [E2BOX EBIMU-9DOFV6 매뉴얼](https://www.e2box.co.kr/entry/EBIMU-9DOFV6-%EB%A7%A4%EB%89%B4%EC%96%BC).
직접 읽은 로컬 원본은 `/home/venezuelachocolatemilk/Downloads/EBIMU-9DOFV6_rev10.pdf`, `EBIMU-9DOF_시작하기_rev12.pdf`, `EBIMU-9DOFV6_상황별_설정.pdf`이다. `pdftotext -layout` 결과가 각각 기존 `references/EBIMU-9DOFV6_rev10.txt`, `getting_started.txt`, `settings.txt`와 정확히 일치한다. 아래 쪽수는 PDF의 인쇄 쪽수이다.

| 항목 | 확인 내용 / 근거 |
| --- | --- |
| UART 기본값 | 115200bps, 8 data bits, no parity, 1 stop bit, 하드웨어 흐름 제어 없음. rev10 p5/20 |
| 지원 baud | 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600. p15/20 |
| 기본 출력 | pons=1, sor=10(10ms/100Hz), soc=1(ASCII), sof=1(Euler); 일반 출력은 호스트 요청 없이 자동 전송. p7/15/18, 시작하기 p10–11/14 |
| 기본 채널 | Roll/Pitch/Yaw만 출력, sog=0(gyro 출력 OFF); soa/som/sod/sot/sots도 기본 OFF. p15 |
| ASCII | 시작 `*`=2a, 쉼표 `,`=2c 구분, 끝 CR LF=0d 0a. Euler R,P,Y 다음 활성화된 gyro x,y,z 등 순서. p12 |
| Binary | SOP=55 55, 16bit 필드(상위 바이트 먼저, signed 2의 보수 데이터), 끝은 16bit CHK; SOP와 데이터의 바이트 합, overflow 무시. ASCII식 CR/LF나 쉼표 없음. p13의 예제/정의 |
| 자세/gyro 단위 | Euler degree; gyro DPS(degree/s), rad/s=degree/s×π/180. binary Euler /100, gyro /10. p21 |
| 명령 프레이밍 | 소문자 ASCII `<CMD[DATA]>`, 3c/3e로 경계. CR/LF가 명령 종료 조건이 아님. p14, 시작하기 p12–13 |
| 버전 조회 | `<ver>`=3c 76 65 72 3e 정식 지원. 응답 예 `<imu9dofv600>`, 이 명령은 `<ok>`를 보내지 않음. p34 |
| 설정 조회 | `<cfg>`=3c 63 66 67 3e 정식 지원. 출력 후 **추가 `>`=3e를 받을 때까지 정지**. 조회를 끝낼 때 반드시 별도 `>` 필요. p34 |
| 명령 응답 | `<` 수신 시 일반 출력 중단/명령 수신, `>`로 명령 완료, 실행/응답 뒤 일반 출력 재개 도식에 about 500ms 표시. cfg는 별도 종료가 필요한 예외. p8/34. 일반 정상 `<ok>`, 오류 `<er>`. p19 |
| 출력 모드 차이 | sor=0이면 자동 전송 대신 단일 `*`(2a)에 현재 자세 데이터를 즉시 응답. ASCII/Binary는 데이터 프레임 차이이며 명령은 ASCII `<...>`로 정의됨. p8/12–14 |
| 자동 시작 비활성/정지 | pons=0은 전원 인가 시 미작동, `<stop>`은 작동 중지. `<start>`는 상태 변경이므로 이번 시험에서 전송하지 않음. p18/34 |
| 센서 부팅 시간 | 이 세 문서에서 수치로 명시한 전원 인가→준비 완료 시간은 확인하지 못함. p8의 about 500ms를 부팅 시간으로 해석하지 않음 |
| DTR/RTS | 센서 UART 연결에 필수라는 기술 없음. UART는 TX/RX이며 no H/W flow controls 명시. 변환기 제어선이 실제 센서 nPD/nRST에 연결되어 있는지는 소프트웨어로 확인 불가. p4–6 |

센서의 **현재** baud/출력 코드/채널/pons/sor/실행 상태/펌웨어는 무응답이어서 미확인이다. 공장 기본값을 현재 설정으로 간주하지 않는다. 출력 모드별 명령 문법은 공통으로 정의되어 있으나, 정지/비정상 초기화 상태에서 항상 응답한다는 보장까지 문서에서 확인한 것은 아니다.

### 기존 Python/ROS와 새 진단의 초기화 비교

| 설정 | 기존 ROS node.py | 기존 configure.py | 새 독립 진단의 실제 설정 |
| --- | --- | --- | --- |
| baud / UART | 기본 115200 / 8N1 | 고정 115200 / 8N1 | 지원 8종 / 8N1; 마지막 115200 |
| timeout | Serial timeout=0; 노드 timeout=0.2는 수신 감시/파서 gap 용도 | Serial timeout=0.2, 조회 deadline=3초 | Serial timeout 최대0.05초, monotonic으로 phase 시간 제한 |
| inter_byte_timeout | pyserial 기본 None | pyserial 기본 None | 명시적 None |
| xonxoff / rtscts / dsrdtr | 모두 False | xonxoff 기본 False, 나머지 False | 모두 명시적 False |
| DTR / RTS | 명시 없음; pyserial 기본 True/True | 명시 없음; pyserial 기본 True/True | port=None에서 LOW/LOW 또는 HIGH/HIGH 설정 후 open; ioctl로 실제 상태 확인 |
| 초기 대기 | 별도 없음, timer가 바로 수신 | 별도 없음, 바로 명령 전송 | 첫 무송신 시험 3초 + 2초; 다른 시험 1초 또는0.6초 초기 캡처 |
| 입력 버퍼 | pyserial.open 내부에서 한 번 flush | 동일 | 동일, open 뒤 추가 flush 없음; 초기 대기 중에도 수신 기록 |
| open/close | 명령 없이 한 번 open, 종료 시 close | 별도 CLI마다 open/close | 각 진단 내 같은 fd 유지, cfg 종료 뒤 수신 확인, finally close |

기존 수동 시험은 로그에서 8N1/flow OFF/exclusive 및 결과를 확인했지만, 시험 스크립트 원본과 모든 timeout/DTR/RTS/flush 시점은 남아 있지 않아 정확한 값을 소급 확정하지 않는다. 해당 시험을 같은 조건으로 반복하지 않았다.

실제 termios는 CS8, CREAD/CLOCAL=True, PARENB/CSTOPB/CRTSCTS=False, IXON/IXOFF/IXANY=False, ICANON/ECHO/ISIG/IEXTEN=False, INLCR/IGNCR/ICRNL/ISTRIP=False, OPOST=False, VMIN/VTIME=0이었다. 따라서 줄바꿈 대기나 canonical/echo/CR 변환, DCD 대기, RTS/CTS·XON/XOFF 차단 없이 raw bytes를 읽는 조건이다. HUPCL=True는 기존 TTY 상태 그대로이며 close 시 제어선 하강이 가능하다. 제어선의 open 순간 glitch나 실제 전압/센서 리셋 여부는 ioctl로 증명할 수 없다. LOW와 HIGH 두 상태 모두 결과는 0이다.

ModemManager는 active이고 udev ID_MM_CANDIDATE=1이지만 이번 boot 로그에는 해당 USB 장치를 지원하는 plugin이 없다고 기록된다. serial-getty@ttyUSB0/brltty/gpsd 서비스는 inactive, 종료 후 fuser는 점유 없음이다. 시험 중 fuser에 나타난 python3 PID는 이 진단 프로세스다. 커널 로그에서 CP210x 연결 뒤 오류/재연결 기록은 확인되지 않았다. 간헐적 외부 프로세스 간섭을 완전히 증명/배제한 것은 아니며, 원인이라는 증거도 없다. 서비스 중단/udev·영구 OS 설정 변경은 하지 않았다.

### 실제 읽기 전용 UART 시험 결과

포트: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` → `/dev/ttyUSB0`.
모든 시험은 pyserial 3.5, exclusive=True, 8N1, flow OFF, inter_byte_timeout=None이다. 조회/poll phase는 p8의 about 500ms보다 긴 1.1초 또는1.5초로 제한했다.

| 시험/증거 prefix | 조건과 송신 순서 | 수신 |
| --- | --- | --- |
| 01_passive_low | 115200, DTR/RTS LOW, 3초 초기 캡처+2초 passive, **송신 없음** | 0 bytes |
| 02_queries_low_115200 | LOW, 초기1초+passive1.5초, 이전 cfg 종료 `>` → `<ver>` → `<cfg>` → 조회 종료 `>` → `*`; 각 후1.5초 캡처 | 각 phase 0 bytes, TX 13 bytes |
| 03_query_baud_9600/19200/38400/57600/230400/460800/921600 | HIGH, 각 초기0.6초+passive1.1초, `>` → `<ver>` → `*`; 각 후1.1초 캡처 | 7회 모두 각 phase 0 bytes, 각 TX 7 bytes |
| 04_queries_high_115200 | HIGH, 초기0.6초+passive1.1초, `>` → `<ver>` → `*`; 각 후1.1초 캡처 | 각 phase 0 bytes, TX 7 bytes |

기존의 8개 baud passive 스캔과 달리 추가 7개 baud 시험은 **저장 baud 변경 + polling/조회 대기 상태**라는 가설에 대해 실제 읽기 요청을 수행한 것이다. 긴 무송신 반복이나 미확인 명령 추측은 하지 않았다.

신규 10회 합계 **RX 0 bytes / host write가 받아들인 TX 69 bytes**. 각 명령의 HEX/ASCII/시각, 실제 포트 설정, 단계별 수신량, close 및 summary는 `verification/imu_uart_diagnosis_20261016/*.jsonl`에 있다. 대응 `*.rx.bin` 10개도 모두 길이0이며, RX ASCII/HEX는 빈 값이다. write 성공은 호스트/변환기에 전달됐다는 범위의 증거이지 센서가 명령을 인식했다는 ACK가 아니다. 총괄 `summary.json`, OS 증거 `linux_evidence.json` 참조. 모든 포트 정상 해제, 마지막 호스트 baud=115200.

전송한 바이트 종류는 `>`(3e), `<ver>`(3c 76 65 72 3e), `<cfg>`(3c 63 66 67 3e), `*`(2a)뿐이다. CR/LF를 추가하지 않았다. `<start>/<stop>/<reset>/<lf>`, 출력/지자기 설정/캘리브레이션/비휘발성 저장 명령은 전송하지 않았다.

### 확인된 소프트웨어 문제와 최소 수정

1. **확정된 조회 도구 문제:** 기존 `src/ebimu_driver/ebimu_driver/configure.py`는 `<cfg>` 후 추가 `>` 없이 close했다. 매뉴얼 p34와 불일치하므로 configuration action에만 try/finally로 `>` 전송을 추가했다. 조회가 실패/중단되어도 열린 포트에서 조회를 종료하려 시도한다. 이전 버전은 증거 디렉터리에 보존했다. 이번 독립 스크립트도 같은 종료 처리를 한다. 이번 실제 시험에서 종료 문자를 보낸 뒤에도 RX=0이므로 최초 무수신 원인으로 확정하거나 복구됐다고 주장하지 않는다.
2. **확인된 호환 조건:** 기존 `AsciiParser` 기본은 Euler+gyro 6필드(옵션9필드)만 허용하지만 제조사 기본은 Euler 3필드이다. binary/quaternion/기타 추가 채널도 이 파서와 맞지 않는다. 이는 bytes가 들어온 뒤 파싱 실패를 설명하며 raw RX=0은 설명하지 않는다. 실제 프레임/설정을 확보하지 못했으므로 gyro를 가짜0으로 채우거나 파서를 추측 수정하지 않았다. `node.py`와 `protocol.py`는 변경 없음.
3. **추가 코드 확인:** 설정 변경 경로의 오류 검사는 `<err`를 찾지만 제조사 오류 코드는 `<er>`(p19)이다. 즉각적인 오류 분류가 늦어질 수 있으며 최종 ACK 누락 예외로 종료된다. 해당 설정 변경 경로는 이번에 실행/수정하지 않았고 RX=0의 근거로 사용하지 않는다.
4. **신규 진단:** `tools/diagnose_ebimu.py`는 기본 무송신, 허용 TX는 명시적 조회/조회 종료/polling뿐이며 임의 명령 인자를 제공하지 않는다. ASCII/HEX와 시간, 실제 수신량, termios 및 제어선, 예외·종료를 JSONL/원시 파일로 기록한다. 종료코드2는 정상 진단 종료이나 RX=0(UNRESOLVED_ZERO_RX), 1은 오류, 0도 RX 관측일 뿐 유효 IMU 패킷 성공 보장은 아니다.

검증: 스크립트 PTY 무송신/조회 프레이밍·종료 2건 PASS(`script_checks.json`, **합성 전송 검증**); 기존 configure 도구의 정상 cfg/읽기 예외 cfg/version 회귀 3건 PASS(`configure_checks.json`, mock serial); 설치된 `ros2 run ebimu_driver ebimu_configure configuration`의 무송신 OFFLINE PREVIEW 정상. 스크립트 문법 검증 PASS. PTY 시험의 15 bytes는 실제 센서 RX 총계에 포함하지 않는다. 설정 변경 코드를 실제 센서로 시험하지 않았으며 전체 재빌드/워크스페이스 재구성은 하지 않았다.

### 원인 판단과 실제 ROS 검증 한계

- 이번 조건에서 제조사 UART와 Linux 설정 불일치, CR/LF/줄 단위 디코딩, ROS 파서, 버퍼 대기/대기 후 flush, 기본 DTR/RTS 활성 상태만으로 설명되는 무수신 가능성을 검토했다. 독립 raw 캡처도0이므로 현재 장애는 ROS 파싱/토픽 단계 이전이다. ASCII/Binary 디코딩 차이 자체는 raw bytes가0인 이유가 아니다.
- 조회 종료 누락과 polling 가능성에 대해 문서상의 읽기 요청을 시험했지만 응답0이다. 명령이 센서에 도달/인식되었다는 증거가 없어 해당 센서 상태 자체를 확정적으로 배제하지 않는다. 지원 baud8종 모두 host측 조회 실패이며 실제 센서 baud를 식별한 것은 아니다.
- 센서가 정상이라는 전제를 유지한다. 현재 저장 설정, stop/pons 상태, 초기화/리셋 상태, UART 양방향 전달 여부는 미확인이다. 호스트에서 USB 인식/쓰기 성공만으로 센서 동작 상태나 UART 전기 신호를 관측할 수 없다. 이전 불확실한 루프백은 고장 판단에 사용하지 않았다. 하드웨어 고장으로 결론 내리지 않는다.
- `/tractor/imu/data`, `/trailer/imu/data`: **실센서 발행 미실행/미검증**. raw RX=0이므로 수신 성공 조건 이후의 ROS 절차로 진행하지 않았다. frame/stamp/rad/s 변환/실제 Hz/정상·비정상 실프레임 처리도 미검증이며 기존 합성 시험 PASS를 실센서 성공으로 바꾸지 않는다. 연결 한 개 기준까지만 진단했다. 전체 자율주차 Launch/모터 동작 없음.
- 다음 판단에 필요한 정보는 센서의 이전 baud·저장 출력/자동 시작 설정과 현재 작동 상태다. 추가 `<start>` 또는 재시작/전원 재인가로 상태를 바꾸는 시험은 이번 읽기 전용 결과를 검토한 뒤 별도 승인/협의할 단계다. 현 증거로 특정 물리 조작이 반드시 필요하다고 단정할 수 없어 배선/납땜/부품 변경을 요구하지 않는다. 동의 없이 상태 변경 명령이나 물리 조작을 진행하지 않았다.

기존 원본 `~/mvsim_slam`, Arduino, Serial Sender, MotionCommand, Hybrid A*, LQR, Cartographer 및 전체 프로젝트 완료 기록을 수정하지 않았다. 이번 변경 범위는 IMU 진단/증거, 조회 도구의 cfg 종료 보완, 이 기록뿐이다. Git remote push 없음.

## 현재 작업: 실제 EBIMU 벤치 시험 (2026-10-16, KST)

이 절은 아래의 과거 소프트웨어 개발 100% 기록과 구분한다. 이번 사용자 요청으로 **IMU 단독 연결/통신/ROS 시험은 허용**되었다. 기존 원본 워크스페이스, Arduino/Serial Sender/MotionCommand/Hybrid A*/LQR 수정 및 모터/전체 자율주차 Launch 실행은 금지된다.

- 실제 IMU 시험 진행률: **0% 단계 완료, 20% 장치 식별 대기**. USB-UART는 발견했으나 IMU 응답은 아직 없다.
- 0% 드라이버/문서 분석: 완료. README.md, HARDWARE_INTERFACES.md, node.py/protocol.py/configure.py, imu.yaml 및 로컬 제조사 `references/EBIMU-9DOFV6_rev10.txt` 확인.
- 실제 엔트리: `ros2 run ebimu_driver ebimu_node`; `--ros-args -r __node:=tractor_imu -p enable_device:=true -p port:=<PORT> -p baud:=115200 -p output_topic:=/tractor/imu/data -p frame_id:=tractor_imu_link`. trailer는 별도 포트와 `/trailer/imu/data`, `trailer_imu_link`, `trailer_imu` 사용. 전체 Launch 불필요.
- 코드 확인: 115200 8N1/흐름 제어 없음, 시작 시 명령 전송 없음. 기본 ASCII `*R,P,Y,gx,gy,gz\r\n` 정확히 6필드, Euler degree 및 gyro DPS→rad/s, quaternion 생성, host 수신 시각, sensor-data QoS. 가속도 미수신 covariance[0]=-1. timeout 기본 0.2초. 선택적 9필드는 include_acceleration=true가 필요하다. 위 내용은 **코드 확인이며 실센서 검증이 아님**.
- 발견한 USB-UART: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` → `/dev/ttyUSB0`; VID:PID `10c4:ea60`, serial `0001`, CP2102. by-path `/dev/serial/by-path/pci-0000:80:14.0-usb-0:10:1.0-port0`. **USB 어댑터 정보만으로 EBIMU 또는 Arduino라고 판정하지 않음**. ttyUSB/ttyACM 후보는 이것 1개뿐이었다. dialout 권한 있음, fuser 점유 없음.
- 최초 실제 포트 조회: pyserial exclusive=True, 115200 8N1, 흐름 제어 OFF. passive 2초→0 bytes, `<ver>` 3초→0 bytes, `<cfg>` 3초→0 bytes. 이외 명령 송신 없음. 종료 시 포트 닫힘. 펌웨어/현재 설정/데이터 형식은 **미확인**.
- 원본 증거: `verification/imu_bench_20261016/initial_query.json`, `passive.bin/.txt`, `version.bin/.txt`, `configuration.bin/.txt` (0바이트 응답도 보존).
- 제조사 문서 printed p34: `<cfg>` 출력 후 추가 `>` 전까지 센서가 정지한다. 정상 응답 확보 후 `>`로 조회 모드를 종료해야 한다. 기존 configure CLI는 추가 종료 문자를 전송하지 않으므로 실제 동작을 확인할 때 유의한다. 아직 드라이버 수정 없음.
- 사용자에게 CP2102에 센서까지 연결/전원 인가되었는지, 기존 baudrate가 무엇인지 질의함. 설정 변경 없이 응답을 기다리는 중.
- 추가 무송신 점검: 제조사 지원 baud 9600/19200/38400/57600/115200/230400/460800/921600에서 각각 0.75초 passive read → 모두 0 bytes. PC 측 baud만 변경했고 센서 명령은 보내지 않음. 끝에 115200으로 돌린 후 포트 닫음. `verification/imu_bench_20261016/passive_baud_scan.json` 참조. 출력이 중지되었거나 polling 모드일 가능성까지 배제한 것은 아님.
- 설치된 ROS 엔트리 실제 발견 확인: `ros2 pkg executables ebimu_driver` → `ebimu_configure`, `ebimu_node`; package prefix는 이 워크스페이스의 `install/ebimu_driver`. `ebimu_configure --help` 실행 성공. 수신 노드는 아직 미실행, 빌드/소스 수정 없음.
- 40% 제조사 통신/정지·회전 수신: 미완료.
- 60% 현재 설정 보고/필요한 변경/자이로 보정: 미완료. `<sem0><soc1><sof1><sog1><soa0><sor10>` 등 아직 미전송. `<cg>`는 실제 정지 확인 후에만 실행. `<lf>`, `<cmf>` 금지.
- 80% ROS 실제 토픽/단위/주기/중단 감지: 미완료, ROS 드라이버 아직 실행하지 않음.
- 100% 두 실센서 독립 동시 실행: 미완료. 확인된 IMU 0개, USB-UART 1개뿐임. 가짜 데이터/가상 포트로 실센서 성공을 대체하지 않음.
- 다음 재개: 장치/점유 상태 재확인 → 센서 연결/전원/baudrate 답변 확인 → 현재 설정을 바꾸지 않고 `<ver>`, `<cfg>` 응답 저장 → `>` 조회 종료 → 현재 설정 사용자 보고 → 정지/회전 측정 → 필요한 최소 설정 및 정지 확인 후 보정 → 진단 포트 닫고 IMU 노드만 실행해 실제 ROS 검증. 하나만 연결되면 두 개 동시 검증은 미완료로 유지.

---

## Current Status
Overall: 100%
Current Stage: 6/6
Stage Progress: 100%

## Completed Stages
- Stage 1: 100%
- Stage 2: 100%
- Stage 3: 100%
- Stage 4: 100%
- Stage 5: 100%
- Stage 6: 100%

## Completed Work / checkpoints
- 2026-10-16T22:24:06.594948 Stage 1 100%: Stage 1 complete: verified package names, protocol, active B path and porting map. Starting independent workspace.
- 2026-10-16T22:24:36.412585 Stage 2 20%: Independent workspace created outside original; copying complete, source manifests recorded.
- 2026-10-16T22:24:47.949036 Stage 2 40%: All 7 original ROS packages, vision resources and four firmware sketches copied without caches.
- 2026-10-16T22:24:48.021630 Stage 2 60%: Four new ROS packages scaffolded; real_parking compiles copied native clearance source; protected hashes PASS.
- 2026-10-16T22:25:21.296763 Stage 2 80%: YAML geometry, sensor, parking, controller, hardware and SLAM configs written. Architecture documented; all dimensions except supplied outer size PROVISIONAL.
- 2026-10-16T22:25:27.546092 Stage 2 100%: colcon discovers 11 distinct ROS packages; preserved 7 and new 4. Stage 2 structure/dependencies documented.
- 2026-10-16T22:25:27.559220 Stage 3 0%: Implementing LiDAR standard gateway, EBIMU protocol and external Arduino adapter.
- 2026-10-16T22:26:00.264154 Stage 3 20%: Standard LaserScan gateway and optional RPLidar source implemented outside original package; no port opened; timestamps/QoS/errors handled.
- 2026-10-16T22:26:42.797190 Stage 3 40%: Manufacturer six-field ASCII parser implemented; rejects corrupt/partial/nonfinite frames; Euler and DPS converted. Settings/calibration separate explicit CLI, preview default.
- 2026-10-16T22:27:27.854007 Stage 3 60%: Two parameterized EBIMU nodes implemented with independent ports/topics, axis rotation, covariance and no startup writes; no hardware opened.
- 2026-10-16T22:27:27.870128 Stage 3 80%: External Arduino adapter implemented: original Twist yaw-rate -> steering steps ±7, signed PWM ±255, rate/range/finiteness/freshness/latched E-stop gates, preview topic and actuation disabled by default.
- 2026-10-16T22:27:55.639664 Stage 3 100%: IMU parser tests 3 PASS; manufacturer protocol and hardware limits documented in HARDWARE_INTERFACES.md. Stage 3 implemented; ROS/custom-message adapter tests deferred until build.
- 2026-10-16T22:27:55.653653 Stage 4 0%: Porting Cartographer, static/dynamic TF, real rear-axle state and aligned trailer gyro integration.
- 2026-10-16T22:29:01.225381 Stage 4 20%: Final B LiDAR-only Lua ported intact; optional IMU Lua separate. No default fusion change.
- 2026-10-16T22:29:16.355026 Stage 4 40%: Implemented SLAM scan/map/matching freshness bridge; separate detection/planning map and TF-at-scan free-ray augmentation.
- 2026-10-16T22:29:16.368434 Stage 4 60%: Static sensor TF and map/local_map ownership implemented; body-center SLAM pose converted to rear axle; articulated trailer body TF dynamic.
- 2026-10-16T22:29:26.781409 Stage 4 80%: Preserved GyroYaw with explicit supplied bias and articulated anchor; coherent timestamp pairing and freshness gates implemented; 6 parser/geometry/gyro tests PASS.
- 2026-10-16T22:29:26.794776 Stage 4 100%: Stage 4 ROS state/TF/map interfaces complete. Tractor IMU freshness required, trailer uses gyro-only integration; initial calibration acknowledgments default false.
- 2026-10-16T22:29:26.808895 Stage 5 0%: Connecting unchanged Final B AutoParking, native optimizer, steering-priority/CDE, independent algorithm entries and external adapter.
- 2026-10-16T22:30:34.667324 Stage 5 20%: Unchanged active OccupancyGrid U-gap detector connected in AutoParking plus independent gap_detector diagnostic node. Original observation stability and target checks retained.
- 2026-10-16T22:30:50.183261 Stage 5 40%: Preserved off-axle articulated model and Hybrid A* installed; YAML-to-model/footprints input factory added outside vendor; independent planner request/result implemented.
- 2026-10-16T22:30:50.199200 Stage 5 60%: B bootstrap applies exact active native clearance+shot+resume20, steering_priority and CDE candidate_scheduler_online. No algorithm source modified.
- 2026-10-16T22:31:00.656213 Stage 5 80%: Original Controller/LQR connected through unchanged AutoParking.send yaw-rate convention to new adapter; independent lqr outputs isolated topic; terminal/sensor/failure guards retained.
- 2026-10-16T22:31:00.709148 Stage 5 100%: Full Final B state-machine algorithm graph implemented with separate hardware inputs/output; 43 Python modules syntax checked, algorithm sources 20/20 identical. Runtime validation scheduled in Stage 6.
- 2026-10-16T22:31:00.722784 Stage 6 0%: Creating individual/integrated/development launch, RViz; next full colcon build and ROS hardware-free tests.
- 2026-10-16T22:32:02.449942 Stage 6 20%: Individual ros2 entries (LiDAR/IMUs/SLAM/state/detector/planner/LQR/adapter) provided; original sender and vision entries preserved.
- 2026-10-16T22:32:27.666505 Stage 6 40%: Integrated development-safe launch, optional sensor/SLAM/vision/RViz flags and RViz config implemented. Reliable scan output resolves preserved AutoParking QoS; IMU fusion is explicitly optional with documented soa1 mode.
- 2026-10-16T22:32:46.967018 Stage 6 60%: Full colcon build --symlink-install PASS: all 11 packages. Launch arguments discovered; checking installed B runtime/native library and custom message.
- 2026-10-16T22:32:53.625671 Stage 6 40%: Build succeeds for 11 packages, but installed runtime probe FAILED: symlink-install resolves native.py into source while libnative.so is installed elsewhere. Stage 6 completion corrected; fixing external bootstrap/CMake without editing preserved source. Existing system SciPy/NumPy warning also recorded.
- 2026-10-16T22:33:13.296507 Stage 6 60%: Native symlink runtime path resolved in new bootstrap; preserved optimizer unchanged. Installed B profile and MotionCommand introspection PASS. All 11 packages build. Tracking Path RViz added.
- 2026-10-16T22:37:53.853229 Stage 6 60%: 17 tests PASS including native model/clearance, B forward/reverse planning, CDE scheduler, LQR, detector, two IMUs, adapter and real state. Default ROS no-hardware smoke PASS. Synthetic Cartographer integration FAIL (map exists but state invalid), investigating with trace capture; do not mark Stage 6 complete.
- 2026-10-16T22:38:27.037212 Stage 6 60%: Investigation: synthetic gyro/SLAM pairing missing; adding diagnostic lags. Original source unchanged. Initial colcon list had created a new log directory in original cwd; verified exact tool-owned contents, copied evidence to verification/initial_colcon_discovery.log and removed only session-created artifacts to restore initial original directory.
- 2026-10-16T22:39:46.778349 Stage 6 80%: 18 tests PASS. Default ROS graph smoke PASS; synthetic LaserScan -> real Cartographer map/TF -> gyro state -> original SEARCH_FORWARD -> adapter preview PASS. 278 valid states, 18 maps, 278 tracking paths; stopping synthetic input produces EMERGENCY_STOP and zero preview; hardware commands 0. TF buffering bug fixed externally without relaxing original 30ms pairing.
- 2026-10-16T22:42:47.606621 Stage 6 80%: 20 unit tests PASS. Integration functionality PASS, but expanded shutdown checks found static-only sensor_tf waits indefinitely without ROS signal wakeup. New common runner now uses bounded spin_once(.1s); rerunning affected launch/shutdown tests. No preservation changes.
- 2026-10-16T22:45:20.092445 Stage 6 100%: Software deliverables complete: all 11 packages build; 20 tests PASS; default/synthetic/optional-IMU config launches and shutdown PASS; independent algorithm entries PASS; forbidden actuation rejected. Source baseline174/protected103 and algorithm20 hashes identical. README/VERIFICATION/porting/architecture/hardware docs complete. Hardware, real fusion, RViz GUI and optional YOLO runtime explicitly unverified; torch missing. No pending software code edits.

## Created Files
- ARCHITECTURE.md
- HARDWARE_INTERFACES.md
- PORTING_MAP.md
- PROGRESS.md
- README.md
- VERIFICATION.md

## Architecture Decisions
See PORTING_MAP.md, ARCHITECTURE.md, HARDWARE_INTERFACES.md. Preserve Final B vendor/optimizer algorithms; new ROS hardware adapters. Rear axle planning point. No simulated truth. Separate IMU ports. Hardware disabled in development mode.

## Protected Files
Entire original ~/mvsim_slam is read-only. Copied serial_communication_pkg, interfaces_pkg definitions, control/**/*.ino and original vision/LiDAR packages remain byte-identical, excluding caches/build outputs. tools/verify_preservation.py uses source_baseline.json and protected_manifest.json.

## Verification
See verification/ for actual command logs/results. No hardware verification performed. Source/copy hashes must pass at completion.

## Unresolved Problems
Unmeasured geometry/motor scale/polarities/IMU axes and yaw alignment require actual measurement. Original firmware has no receive watchdog, encoder feedback or independent cutoff. Current ROS Python has no torch, so optional YOLO runtime cannot start; model weights also require provisioning. Original setup entries lidar_obstacle_detector_node/traffic_light_detector_node have missing modules. Core LiDAR software is independent. Hardware and actual IMU fusion remain unverified. See README.

## Current Work
Software deliverables complete: all 11 packages build; 20 tests PASS; default/synthetic/optional-IMU config launches and shutdown PASS; independent algorithm entries PASS; forbidden actuation rejected. Source baseline174/protected103 and algorithm20 hashes identical. README/VERIFICATION/porting/architecture/hardware docs complete. Hardware, real fusion, RViz GUI and optional YOLO runtime explicitly unverified; torch missing. No pending software code edits.

## Next Actions
1. Read this file, progress.json, documents and inspect actual src/ and verification/ files before resuming.
2. Continue incomplete stages; do not repeat completed source analysis.
3. Build: cd ~/mvsim_slam_real && source /opt/ros/humble/setup.bash && colcon build --symlink-install
4. Tests: source install/setup.bash && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest tests -q
5. Preservation: /usr/bin/python3 tools/verify_preservation.py

## Restrictions
No original edits, protected copy edits, deletions/moves of old data, git push, global ROS/Python changes, hardware connection/test/calibration or firmware upload. Default enable_actuation=false. Never claim hardware PASS or measured geometry.


## Created implementation files
- src/ebimu_driver/ebimu_driver/__init__.py: new implementation/entry
- src/ebimu_driver/ebimu_driver/configure.py: new implementation/entry
- src/ebimu_driver/ebimu_driver/node.py: new implementation/entry
- src/ebimu_driver/ebimu_driver/protocol.py: new implementation/entry
- src/ebimu_driver/package.xml: new implementation/entry
- src/ebimu_driver/resource/ebimu_driver: new implementation/entry
- src/ebimu_driver/setup.cfg: new implementation/entry
- src/ebimu_driver/setup.py: new implementation/entry
- src/real_vehicle_bridge/package.xml: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/__init__.py: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/adapter.py: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/common.py: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/estimator.py: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/geometry.py: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/lidar.py: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/rplidar_source.py: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/sensor_tf.py: new implementation/entry
- src/real_vehicle_bridge/real_vehicle_bridge/yaw_estimator.py: new implementation/entry
- src/real_vehicle_bridge/resource/real_vehicle_bridge: new implementation/entry
- src/real_vehicle_bridge/setup.cfg: new implementation/entry
- src/real_vehicle_bridge/setup.py: new implementation/entry
- src/real_parking/CMakeLists.txt: new B integration/entry
- src/real_parking/executables/auto_parking: new B integration/entry
- src/real_parking/executables/gap_detector: new B integration/entry
- src/real_parking/executables/hybrid_astar: new B integration/entry
- src/real_parking/executables/lqr: new B integration/entry
- src/real_parking/package.xml: new B integration/entry
- src/real_parking/real_parking/__init__.py: new B integration/entry
- src/real_parking/real_parking/bootstrap.py: new B integration/entry
- src/real_parking/real_parking/nodes.py: new B integration/entry
- src/real_parking/real_parking/optimizer/native.cpp: preserved B algorithm
- src/real_parking/real_parking/optimizer/native.py: preserved B algorithm
- src/real_parking/real_parking/optimizer/priority_schedule.py: preserved B algorithm
- src/real_parking/real_parking/optimizer/resumable.py: preserved B algorithm
- src/real_parking/real_parking/optimizer/steering_priority.py: preserved B algorithm
- src/real_parking/real_parking/optimizer/variants.py: preserved B algorithm
- src/real_parking/real_parking/support/candidate_scheduler_online.py: preserved B algorithm
- src/real_parking/real_parking/support/path_geometry.py: preserved B algorithm
- src/real_parking/real_parking/support/speed_clearance.py: preserved B algorithm
- src/real_parking/real_parking/vendor/observed_stop.py: preserved B algorithm
- src/real_parking/real_parking/vendor/raw_reject_event_capture/raw_reject_capture.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_local_parking_gap_detector.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_parallel_parking_auto.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_parking_articulation_safety.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_parking_blocked_debug.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_parking_clearance.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_parking_controller.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_parking_map_safety.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_parking_perception.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_slam_parking_adapter.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_trailer_hybrid_astar.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_trailer_model.py: preserved B algorithm
- src/real_parking/real_parking/vendor/tractor_trailer_state_estimator.py: preserved B algorithm
- src/mvsim_slam_real_bringup/config/controller.yaml: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/hardware.yaml: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/imu.yaml: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/lidar.yaml: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/map_builder.lua: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/parking.yaml: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/pose_graph.lua: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/reference_imu_optional.lua: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/reference_lidar_only.lua: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/slam.yaml: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/trajectory_builder.lua: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/trajectory_builder_2d.lua: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/trajectory_builder_3d.lua: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml: sensor/vehicle/SLAM parameters (unmeasured values provisional)
- src/mvsim_slam_real_bringup/launch/real_parking.launch.py: hardware-safe launch definition
- src/mvsim_slam_real_bringup/launch/slam.launch.py: hardware-safe launch definition
- src/mvsim_slam_real_bringup/launch/vision.launch.py: hardware-safe launch definition
- src/mvsim_slam_real_bringup/mvsim_slam_real_bringup/__init__.py: new implementation/entry
- src/mvsim_slam_real_bringup/package.xml: new implementation/entry
- src/mvsim_slam_real_bringup/resource/mvsim_slam_real_bringup: new implementation/entry
- src/mvsim_slam_real_bringup/rviz/real_parking.rviz: RViz display configuration
- src/mvsim_slam_real_bringup/setup.cfg: new implementation/entry
- src/mvsim_slam_real_bringup/setup.py: new implementation/entry

## Final Verification
- Build: verification/build_final.log, 11/11 packages PASS.
- Tests: verification/tests.log, 20/20 PASS.
- Integration: verification/default_no_hardware.json, synthetic_integration.json, optional_imu_config.json, individual_nodes.json all PASS including normal shutdown.
- Detailed matrix and limits: VERIFICATION.md; machine-readable summary: verification/summary.json.
- SHA-256: original baseline174/protected-copy103 PASS; Final B algorithms20 byte-identical.
- Original SerialSender and all .ino files remain unchanged, including copied metadata/resources.

## Next session — software scope completed
Read this file and actual verification artifacts first. No incomplete software stage remains. Do not rerun initial analysis or scaffolding. Follow-up work requires measured geometry/motor factors, selected LiDAR, actual sensor/vehicle access and separate authorization; current session forbids physical connections/tests. Optional vision requires a workspace-isolated torch/runtime/model installation, without global environment changes.
