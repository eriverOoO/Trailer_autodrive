"""Report generation only; does not import ROS, open UART, or change settings."""
from pathlib import Path
import datetime,json,hashlib,subprocess,yaml
ROOT=Path('/home/venezuelachocolatemilk/mvsim_slam_real')
OUT=Path(__file__).resolve().parent
BASE=ROOT/'visualization/tractor_trailer_rviz_2d'
TARGET=ROOT/'verification/REAL_IMU_YAW_AND_G6_RANGE_FIX_20261020.txt'

def data(name,default=None):
 p=OUT/name
 return json.loads(p.read_text()) if p.exists() else default

def show(value):return json.dumps(value,ensure_ascii=False,indent=2)

before=data('baseline_observation/observation_summary.json',{})
after=data('range_after_observation/observation_summary.json',{})
physical=data('physical_yaw_tests.json',{})
range_counts=data('actual_range_counts.json',{})
matched=data('actual_cartographer_matched_ranges.json',{})
protection=data('protection_check.json',{})
params=yaml.safe_load((ROOT/'src/mvsim_slam_real_bringup/config/g6.yaml').read_text())['ydlidar_ros2_driver_node']['ros__parameters']
visual=yaml.safe_load((BASE/'config/tractor_only_diagnostic.yaml').read_text())
commands=(OUT/'commands.txt').read_text()
sections=[]
def add(title,body):sections.append(title+'\n'+body)
add('REAL_IMU_YAW_AND_G6_RANGE_FIX_20261020',
 '작성: '+datetime.datetime.now().astimezone().isoformat()+
 '\n현재 상태: 실제 G6 거리 검증 완료. 사용자 CCW 시험으로 시각화 역방향을 확인하여 진단 sign=-1 적용. 수정 후 실제 CW/CCW 검증 진행중. 전체 성공/100% 완료 아님.')
add('1. 중단 상태와 NameError 조사',
 '작업 시작 시 이전 종료 요청대로 G6/Cartographer/IMU/RViz/tmux 모두 중단 상태였음. USB 두 포트 점유 없음.\n'
 '기존 timestamp_guard.py/트랙터 단독 모드/백업/27개 검사 및 캡처는 보존됨. 초기화/재구현하지 않음.\n'
 "이전 NameError: name 'base' is not defined는 마지막 보고서 f-string의 임시 inline Python 명령에서 base 경로 변수를 정의하지 않은 오류다.\n"
 '파일 쓰기 전에 발생하여 최종 보고서만 생성되지 않았다. 실행 중 SLAM/IMU 모듈 오류가 아니므로 해당 코드를 고쳐 해결하지 않음.\n'
 '이번 생성기는 독립 write_report.py로 저장하고 ROOT/OUT/BASE/TARGET을 먼저 명시한다. ROS/UART를 사용하지 않음.')
add('2. 기존 Yaw 반전 원인 조사',
 '현재 경로: serial_protocol.Parser (제조사 z,y,x,w -> ROS x,y,z,w) -> serial_bridge 실제 Imu Quaternion -> visualizer.YawTrack -> render_adapter.\n'
 '제조사 EBIMU 매뉴얼 Quaternion 순서와 파서 매핑 일치. 공용 IMU 메시지는 변경하지 않음.\n'
 '수정 전 트랙터 설정 sign=1이 시각화 내부 Yaw에 한 번 적용됐음. 초기 기준은 상수 offset이며 이후 delta의 부호를 바꾸지 않음.\n'
 'row-vector 변환 p @ [[cos,sin],[-sin,cos]]와 Marker Quaternion sin(yaw/2)/cos(yaw/2)는 ROS +Yaw(CCW)와 일치.\n'
 '카메라 TopDownOrtho/Angle0이며 지도·Scan·카메라 반전은 하지 않음.\n'
 '사용자가 실제 장착 상태의 CCW 수동 회전을 완료했다고 확인함. 기록상 raw Yaw와 화면 Yaw가 함께 감소하여 역방향 재현. sign=+1이 네이티브 회전 방향을 그대로 통과시킨 것이 시각화 원인.\n'
 '첫 CCW 기록을 근거로 진단 sign=-1을 적용했으나, 이후 CW 복귀 기록과 일치하지 않아 이 부호의 물리적 최종 판정은 보류한다.\n'
 'CW 복귀 구간 raw=-9.350도, arrow=+9.354도, hull=+9.377도로 기록되었고 Pitch가 약-5도에서-28도로 변했다. 수평 회전과 기울기 변화가 섞여 재시험 요청.\n'
 '센서 축/장착 기울기 또는 제조사 자세 기준의 물리적 세부 원인은 미교정이며, 각도 크기 정확도를 확정하지 않음.')
add('3. 수정 전후 Yaw 계산 및 실제 좌우 시험',
 '기존 계산: visual_yaw = initial_SLAM_heading + (YawTrack.value - initial_value).\n'
 'YawTrack.value = unwrap(sign * yaw(q * inverse(sensor_to_body_mount))).\n'
 '현재 실제 시각화 sign='+str(visual['tractor']['sign'])+'. 실제 CCW 반전 재현 후 이 진단 설정 한 항목만 변경함.\n'
 '진단 config의 tractor.sign만 +1에서-1로 변경. 공용 raw/Quaternion/Cartographer TF/알고리즘을 바꾸지 않음. sign은 시각화 YawTrack 내부에 한 번만 적용. 초기 reference도 새 sign으로 다시 기록하므로 상수 heading 자체는 반전하지 않음.\n'
 '±180도 경계 및 양·음 변화량/진단 설정만 반전 확인 포함 총34개 검사 PASS. 실제 수정후 방향 PASS와 구분. 실제 CCW 전후 값: /home/venezuelachocolatemilk/mvsim_slam_real/verification/imu_yaw_g6_range_20261020_164031/before_fix_actual_ccw.json\n'+show(physical))
add('4. 정상 IMU timestamp 수정 유지',
 '기존 SensorTimestampGuard와 serial_bridge.py/serial_protocol.py를 변경하지 않음.\n'
 '기존 정상 60000ms 경계 허용, 중복/비정상 역행/긴 수신 간극/호스트 시간 불일치 차단 유지.\n'
 '이번 실제 수신 중 약100Hz/LIVE/파싱오류0/여러 정상순환 확인. 상세 after observation 및 imu/tractor 로그.\n'+show(data('imu_current_validation.json',{}))+'\n'
 '트레일러 IMU를 시작하지 않으며 phi/trailer_yaw는 null, 화면 N/A 유지.')
add('5. 기존 거리 1m 제한 원인',
 '기존 src/mvsim_slam_real_bringup/config/g6.yaml range_max=1.0이 SDK LidarPropMaxRange -> m_MaxRange로 전달됨.\n'
 'SDK isRangeValid가 범위 밖을 0으로 처리하고 드라이버 LaserScan은 SDK min/max를 그대로 사용함.\n'
 'active g6_lidar_only.lua는 reference_lidar_only.lua를 include한 뒤 max_range=1.0으로 최종 override.\n'
 'RViz 시점 문제가 아니라 실제 드라이버/Cartographer 양쪽 제한이었음. baseline runtime dump/scan 기록으로 확인.')
add('6. 거리 설정 전후 및 경계',
 '변경 전: G6 0.15~1.0m, Cartographer 0.15~1.0m. 실제 baseline Scan '+str(before.get('scan',{}).get('receipt_hz'))+'Hz.\n'
 '변경 후: G6 range_min='+str(params['range_min'])+', range_max='+str(params['range_max'])+'m; Cartographer min_range=.15/max_range=12.0.\n'
 'SDK 경계는 reading>=m_MinRange && reading<=m_MaxRange, Cartographer도 range>=min && range<=max로 양 끝 포함.\n'
 'float32 LaserScan range_min 표시는 약0.15000000596m. 최소 field와 실제 장치 정확도는 별개.\n'
 'missing_data_ray_length=1.0m는 그대로 유지: 유효 hit의 최대 거리와 별개인 초과 hit의 free-space miss 길이. 무조건12m로 늘려 빈 공간을 길게 지우지 않음.\n'
 '현재 드라이버가 12m 초과를 SDK에서0으로 버리므로 가짜12m 장애물을 생성하지 않음. Cartographer min_range가0도 배제.\n'
 '보존: 512000baud/18K/12Hz/laser_frame/inverted=true/reversion=false/장치by-path/기타센서파라미터.\n'
 '거리 외 Cartographer 알고리즘/LiDAR-only 모드 변경 없음.')
add('7. 실제 거리 및 SLAM 재실행 결과',
 show(range_counts)+'\nCartographer가 실제 받은 Scan Matching 점군의 센서 기준 거리:\n'+show(matched)+
 '\n12m 관측 대상 및 실측 기준물이 없어12m 정확도 NOT TESTED. 0.15m 최소 정확도도 NOT TESTED.\n'
 '제조사 일반 G6 제품 표기는0.12~16m지만 공식 legacy SDK에는18/16/8K별0.28/0.26/0.1m 최소 표기도 있어, 선택한18K에서0.15m 신뢰성은 보장하지 않음.\n'
 '설정 허용과 하드웨어 근거리 정확도를 구분하며 요청한 소프트웨어0.15m 하한은 그대로 적용.\n'
 '제조사 제품: https://www.ydlidar.com/products/view/4.html (검색 색인 본문 확인; 원본 경로 현재404)\n'
 '제조사 SDK: https://github.com/YDLIDAR/sdk (모드별 표기). 근거리 현장 실측 필요.')
add('8. Scan/Map/TF 전후 비교',
 'before Scan:\n'+show(before.get('scan',{}))+'\nafter Scan:\n'+show(after.get('scan',{}))+
 '\nafter Map:\n'+show(after.get('map',{}))+'\nafter TF lookup:\n'+show(after.get('lookup',{}))+
 '\nScan timestamp TF 변환: '+str(after.get('scan_timestamp_tf'))+
 '\n정지/센서 수동 조작 조건이다. 주행/동적 SLAM 정확도는 NOT TESTED.\n'
 '초기 SDK checksum 자동탐지 오류와 Cartographer 시작 queue 짧음/earlier points 경고는 로그에 보존. 이후 지속 여부를 별도로 점검해야 함.')
add('9. 재시작과 지도 보존',
 '처음 모든 노드가 중단되어 직전 메모리 지도는 없었다. 이전 파일/로그는 보존.\n'
 '변경 전 기존1m 설정을 실제 실행하여 baseline 관측 및 map_before.yaml/driver_before.yaml을 저장.\n'
 '설정 적용을 위해 이 작업의 slam_before만 Ctrl+C 후 포트가 비는 것을 확인하고 slam_after 실행.\n'
 '이는 새 trajectory/새 지도이며 이전 trajectory와 연속된 지도라고 하지 않음. IMU 수신은 이 재시작 동안 유지.\n'
 '활성 installed config는 source symlink이며 원본과내용일치 확인. build/driver/SDK 소스 수정 없음.')
add('10. RViz 및 위치/회전 제한',
 '실제12m 지도/Scan/기존 원본 민트 hull/빨간 tractor arrow를 기존 진단 모드로 표시.\n'
 '별도 RViz 설정 rviz_live_12m.rviz는 Angle0/TopDownOrtho/원본Grid·Map·Scan 유지, 시야만 넓힘.\n'
 '실차 후륜 오프셋/장착 heading은 미측정: SENSOR ANCHOR/UNALIGNED 표기, full vehicle valid=false.\n'
 '실제 RViz 캡처에서 민트색 트랙터 외곽선/빨간 화살표/빨간 Scan/지도 및 TRAILER NO DATA 문구를 확인함. Global Status OK. '+str(OUT/'RVIZ_SIGN_MINUS1_RETEST_PENDING.png')+'\n'
 '지도 형상 정확도 및 실제 벽 위치 대응은 별도 미검증. 최종 좌우 방향 판정 전 화면을 최종 수정 성공으로 보고하지 않음.')
add('11. 회귀 검사',
 '기존27개와 시각화 전용 부호/±180도/양음delta/설정범위 7개, 총34개 PASS (tests.log).\n'
 'unmodified SDK isRangeValid 소스 자체를 별도 C++ harness에 복사하여 최소/최대/범위초과/0/NaN/Inf 등9개 경계 검사 PASS.\n'
 '검사 입력은 메모리/실행파일 내부에서만 사용하며 ROS 가상 Scan/IMU/SLAM 데이터로 발행하지 않음.\n'
 '실제 Map 갱신과 Scan/IMU 신선도는 하드웨어 관측으로 별도 확인.')
add('12. 보호 및 변경·백업 파일',
 '허용 대상: 실제 G6 yaml/LiDAR-only Lua, 물리 측정 후 트랙터 진단 sign 설정, 검증자료.\n'
 '원본backup은 '+str(OUT/'backups')+'\n보호 결과:\n'+show(protection)+
 '\n금지 경로/차량 제어/udev/SDK/기존 rollover와 트랙터 단독 구현 변경하지 않음. commit/push 없음.')
add('13. 실행 명령 및 세션',
 '환경: source '+str(OUT/'environment.bash')+'\nROS_DOMAIN_ID0/ROS_LOCALHOST_ONLY0은 실제 기존 환경값. use_sim_time=false.\n'+commands+
 '\n세션 real_yaw_range_20261020, 세부 run_*.bash/프로세스 기록 참조.\n'
 'output_directory는 기존 폴더를 덮어쓰지 않으므로 재실행은 새 경로 사용.')
add('14. 종료 방법',
 '현재 RViz만: tmux send-keys -t real_yaw_range_20261020:rviz C-c\n'
 'SLAM만: tmux send-keys -t real_yaw_range_20261020:slam_after C-c\n'
 'IMU만: tmux send-keys -t real_yaw_range_20261020:imu C-c\n'
 '현재 진단 visualizer: tmux send-keys -t real_yaw_range_20261020:viz_after C-c\n'
 '수정후시각화창이추가되면그이름의창에도Ctrl+C. 다른프로젝트프로세스를무차별종료하지않음.')
add('15. 원복 방법',
 '이 작업의G6/Cartographer와진단시각화만정상종료하고, backups 아래3개파일을동일상대경로에복원.\n'
 '원래0.15~1m와원래시각화sign로돌아간다. 다른파일/기존timestamp수정/트랙터단독구현을초기화하지않음.\n'
 '활성설정은source symlink이므로기존Launch를다시시작하면복원내용을읽는다. udev/시뮬레이터원복작업불필요.')
add('16. 실패·미검증 및 다음 단계',
 '수정 전 실제 CCW 회전 확인으로 sign=-1을 적용함. 첫 수정후 CW 복귀 시험은 FAIL_OR_INCONCLUSIVE이며 성공으로 보고하지 않는다.\n'
 '기울기 변화가 섞인 기록이므로 Pitch/Roll을 유지한 CW 수동 회전 재시험을 요청한 상태. G6/Cartographer/IMU/RViz는 유지.\n'
 'raw/Marker arrow/Marker hull의 실제 변화량을 동시 구독 기록하며, 수정후 CW와 CCW 방향을 각각 검증해야 한다. 물리 각도 정확도는 별도 미검증.\n'
 '실제물리장착축/초기mapheading/후륜위치/각도정확도/12m정확도/0.15m근거리신뢰성/주행정확도는미검증.\n'
 '진행중결과이며두기능전체PASS/100%완료라고보고하지않음.')
add('17. 현재 검증 판정',
 'G6 USB/실제Scan: PASS\n거리설정0.15~12.0m: PASS\n실제1m초과/2m초과측정: PASS\nCartographer실제원거리점사용/지도/TF: PASS\n'
 'IMU실제UART/100Hz/rollover유지: PASS\n수정전 실제CCW 역방향 재현: CONFIRMED_REVERSED\nYaw방향수정: APPLIED (diagnostic tractor.sign=-1)\n'
 '수정후실제우회전/RViz일치: FAIL_OR_INCONCLUSIVE (기울기 변화 동반, 재시험 대기)\n수정후실제좌회전: NOT TESTED\n트레일러가상데이터미사용: PASS\n차량제어미실행: PASS\n'
 '보호대상예상밖변경: 없음\n현재진행은Yaw원인25%/최소수정40% 및거리75%검증완료. 수정후 실제좌우/통합85%항목대기. 전체100%아님.')
TARGET.write_text('\n\n'.join(sections)+'\n')
print(TARGET)
