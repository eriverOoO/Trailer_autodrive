"""Pure logic for the two-structure test window (no tkinter, no rclpy).

Launch-argument building, Korean explanations of status/failure strings and a numpy
rendering of the detection map. Kept separate so it is testable without a display.
"""
import math
from pathlib import Path
import re
import shlex

import numpy as np

MODES = {
    # key: (label, launch arguments)
    'lidar_manual': ('LiDAR만 · 손으로 밀어 탐색 → [주차 시작]',
                     {'require_camera': 'false', 'allow_lidar_only_motion': 'true', 'manual_survey': 'true'}),
    'lidar_auto': ('LiDAR만 · 자동 탐색 → 주차',
                   {'require_camera': 'false', 'allow_lidar_only_motion': 'true'}),
    'camera_manual': ('카메라+LiDAR · 손으로 밀어 탐색 → [주차 시작]',
                      {'require_camera': 'true', 'manual_survey': 'true'}),
    'camera_auto': ('카메라+LiDAR · 자동 탐색 → 주차',
                    {'require_camera': 'true'}),
}

# Rig and limits behind the placement guide (vehicle_geometry.yaml, two_structure_test.yaml,
# vendor search constants). Distances from the tractor rear axle: nose +0.80 m, trailer end -1.39 m.
RIG_LENGTH_M, RIG_WIDTH_M = 2.19, .55
CLEARANCE_M = .18


def lidar_height(geometry_file):
    """Measured LiDAR height (lidar_mounting_offset z) or None."""
    try:
        import yaml
        values = yaml.safe_load(Path(geometry_file).read_text())
        return float(values['lidar_mounting_offset'][2])
    except Exception:  # noqa: BLE001 - guide still shows without it
        return None


def placement_guide(mode, height=None):
    """Short placement rules shown in the window (Korean)."""
    manual = MODES.get(mode, ('', {}))[1].get('manual_survey') == 'true'
    if height is None:
        tall = '· 높이: LiDAR 스캔 높이보다 높아야 함 (vehicle_geometry.yaml의 lidar_mounting_offset z)'
    else:
        tall = (f'· 높이: LiDAR 스캔 높이 {height:.2f} m보다 높아야 함 → 권장 {height+.08:.2f} m 이상\n'
                f'   (더 낮은 구조물을 쓰려면 LiDAR를 그보다 낮게 달고 높이를 다시 측정)')
    lines = [
        f'차 크기: 길이 {RIG_LENGTH_M} m (견인차+트레일러), 폭 {RIG_WIDTH_M} m',
        '[구조물] 차 진행방향 왼쪽에 두 개, 가로·세로 30 cm 이상, 벽에 붙어도 됨(벽보다 15 cm 이상 튀어나오면 됨)',
        tall,
        '· 두 구조물 사이 간격(x): 3.3~6.0 m  (3.6 m 이상이면 한 번 후진으로 진입)',
        '[차 위치] 출발할 때',
        '· 옆(y): 차 왼쪽 옆면 → 구조물 앞면 0.5~1.2 m',
        '· 앞(x): 차 앞코 → 첫 구조물 1~10 m',
        '· 두 번째 구조물 뒤로 도로 약 6 m 비우기 (공간을 지나 멈춘 뒤 후진)',
        '· 오른쪽: 차 오른쪽 옆면에서 0.3 m 이상 비우기 (후진할 때 앞부분이 오른쪽으로 돎)',
        '· 직진으로 출발 (옆으로 1 m 넘게 벗어나면 비상정지)',
    ]
    if manual:
        lines += [
            '[손으로 밀 때]',
            f'· 미는 동안은 모터 0, 충돌 검사 안 함 → 미는 사람이 {CLEARANCE_M} m 안에 잡혀도 멈추지 않음',
            '· 트레일러 뒤 가운데에서 밀기. 공간 옆 도로에 서 있거나 오래 머물지 않기 (사람이 지도에 장애물로 남을 수 있음)',
            f'· [주차 시작] 전에 차에서 1 m 이상 떨어지기 → 이후엔 어느 방향이든 차체 {CLEARANCE_M} m 안에 아무것도 없어야 함',
        ]
    else:
        lines.append(f'[자동 주행] 어느 방향이든 차체 {CLEARANCE_M} m 안에 아무것도(사람 포함) 없어야 함 → 있으면 비상정지')
    return '\n'.join(lines)

DEFAULT_SETTINGS = {
    'mode': 'lidar_manual',
    'workspace': '',
    'lidar_port': '/dev/lidar_g6',
    'tractor_imu_port': '/dev/imu_tractor',
    'trailer_imu_port': '/dev/imu_trailer',
    'arduino_port': '',
    'camera_path': '',
    'calibration_file': '',
    'slot_width': '',
    'enable_actuation': False,
    'enable_rviz': False,
}

SENSOR_CHOICES = {'camera': '카메라 + LiDAR', 'lidar': 'LiDAR만 · 진단'}
SURVEY_CHOICES = {'manual': '손으로 밀기', 'auto': '자동 전진 탐색'}


def mode_choices(mode):
    if mode not in MODES:
        raise ValueError(f'unknown mode {mode!r}')
    sensor, survey = mode.split('_')
    return SENSOR_CHOICES[sensor], SURVEY_CHOICES[survey]


def mode_from_choices(sensor, survey):
    sensors = {value: key for key, value in SENSOR_CHOICES.items()}
    surveys = {value: key for key, value in SURVEY_CHOICES.items()}
    if sensor not in sensors or survey not in surveys:
        raise ValueError('Unknown sensor or survey selection')
    return f'{sensors[sensor]}_{surveys[survey]}'


def primary_action(mode):
    """Only offer the start service appropriate for the selected survey mode."""
    mode_choices(mode)  # validate before selecting a service
    return ('start_parking', '주차 시작') if mode.endswith('_manual') else ('start_search', '자동 탐색 시작')


def launch_arguments(settings):
    """ros2 launch arguments ('name:=value') for two_structure_parking_test.launch.py."""
    mode = settings.get('mode', 'lidar_manual')
    if mode not in MODES:
        raise ValueError(f'unknown mode {mode!r}')
    camera = MODES[mode][1].get('require_camera') == 'true'
    args = {'development_mode': 'false', 'start_g6_driver': 'true', **MODES[mode][1]}
    for name in ('lidar_port', 'tractor_imu_port', 'trailer_imu_port'):
        value = str(settings.get(name, '')).strip()
        if not value:
            raise ValueError(f'{name} is required')
        args[name] = value
    if camera:
        for name in ('camera_path', 'calibration_file'):
            value = str(settings.get(name, '')).strip()
            if not value:
                raise ValueError(f'{name} is required in camera modes')
            args[name] = value
    elif str(settings.get('camera_path', '')).strip():
        # Camera still runs for reporting in LiDAR-only modes when connected.
        args['camera_path'] = str(settings['camera_path']).strip()
        if str(settings.get('calibration_file', '')).strip():
            args['calibration_file'] = str(settings['calibration_file']).strip()
    slot = str(settings.get('slot_width', '')).strip()
    if slot:
        value = float(slot)
        if not math.isfinite(value) or not .5 <= value <= 3.:
            raise ValueError('slot_width must be 0.5..3.0 m')
        args['slot_width'] = f'{value:.3f}'
    # In hand-push modes the test node holds zero commands until [주차 시작].
    actuation = bool(settings.get('enable_actuation'))
    args['enable_actuation'] = 'true' if actuation else 'false'
    if actuation:
        port = str(settings.get('arduino_port', '')).strip()
        if not port:
            raise ValueError('arduino_port is required when motors are enabled')
        args['arduino_port'] = port
    args['enable_rviz'] = 'true' if settings.get('enable_rviz') else 'false'
    return [f'{k}:={v}' for k, v in args.items()]


def launch_shell(workspace, arguments):
    """bash -lc command line that sources ROS/workspace and execs the launch."""
    ws = shlex.quote(str(workspace))
    return ('source /opt/ros/humble/setup.bash && '
            f'cd {ws} && source install/setup.bash && '
            '{ [ -f tools/g6_env.bash ] && source tools/g6_env.bash || true; } && '
            'export PYTHONNOUSERSITE=1 && '
            'exec ros2 launch mvsim_slam_real_bringup two_structure_parking_test.launch.py '
            + ' '.join(shlex.quote(a) for a in arguments))


# (regex, 설명, 확인할 것) — first match wins. Patterns follow the exact strings in the code.
EXPLANATIONS = [
    (r'two structures and observed free interior confirmed', '두 구조물과 공간 내부 확인 완료 (카메라 확인 포함)', ''),
    (r'LiDAR-only test: camera confirmation bypassed', 'LiDAR만으로 공간 확인 완료 (카메라 확인 생략)', ''),
    (r'waiting for two observed left-side structures \(found (\d+)\)',
     '왼쪽 구조물 후보가 2개 미만입니다 (찾은 개수: {0})',
     '구조물이 시작 위치 기준 왼쪽 0.55~6 m 안에 있는지, LiDAR 스캔 높이(현재 0.687 m)보다 높은지, '
     '벽에 붙어 있다면 벽보다 15 cm 이상 튀어나왔는지 확인하세요. 지도에서 파란 점이 구조물 후보입니다.'),
    (r'waiting for two observed left-side structures', '왼쪽 구조물 후보가 2개 미만입니다',
     '구조물 위치와 LiDAR 높이를 확인하세요.'),
    (r'structures found but no usable pair: (.*)', '구조물은 찾았지만 쌍을 만들 수 없습니다: {0}',
     '간격이 허용 범위(기본 3.30~6.0 m)인지, 두 구조물이 진행방향과 나란한지(10° 이내), '
     '두 구조물 사이 앞쪽에 다른 물체가 없는지 확인하세요.'),
    (r'gap found; interior blocked or unknown \(free=([\d.]+), unknown=([\d.]+)/([\d.]+), '
     r'occupied_group_cells=(\d+)/(\d+)\)',
     '공간 내부 확인 실패 (미관측 비율 {1}/{2}, 내부 점유 덩어리 {3}/{4}셀)',
     '미관측이 허용치를 넘으면: 열린 방향의 /scan이 inf인지, 차가 공간 정면을 지나며 관측했는지 확인하세요. '
     '점유 덩어리가 크면: 공간 안의 물체·바닥 반사·벽 일부가 들어왔는지 확인하세요.'),
    (r'gap found; interior blocked or unknown', '공간 내부가 비어 있지 않거나 관측되지 않았습니다', ''),
    (r'waiting for calibrated camera confirmation', 'LiDAR 공간은 확인됨, 카메라 확인 대기 중',
     '카메라 화면(/parking_vision/objects_overlay)에 두 구조물이 각각 따로 잡히는지 확인하세요. '
     'LiDAR만으로 시험하려면 LiDAR 모드를 고르세요.'),
    (r'camera unavailable: (.*)', '카메라 사용 불가: {0}', '카메라 연결, 보정 파일, 영상/스캔 시각 차이(0.1 s 이하)를 확인하세요.'),
    (r'waiting for measured local_map detection grid', '검출용 지도가 아직 없습니다', 'SLAM(Cartographer)과 state_estimator가 실행 중인지 확인하세요.'),
    (r'detection map timestamp stale', '검출용 지도가 오래됐습니다', 'Cartographer 지도 발행이 멈췄는지 확인하세요.'),
    (r'forward search left road corridor', '전진 탐색 중 차가 도로 경로를 벗어났습니다 (옆 1 m 또는 방향 한계)',
     '조향 부호(search_steer_sign), 바퀴 정렬, 바닥 경사를 확인하세요.'),
    (r'search footprint collides with observed map', '전진 탐색 중 차체가 지도상의 장애물과 겹쳤습니다',
     '진행 경로 옆 벽·물체가 너무 가깝거나, 자기 차체가 점으로 찍히는지(self filter) 확인하세요.'),
    (r'automatic gap search limit reached', '22 m / 220 s 안에 공간을 확정하지 못했습니다', '위의 공간 검출 사유를 확인하세요.'),
    (r'localization/map stream stale', '위치추정·지도·스캔 중 하나가 끊겼습니다', '/scan, /map, /parking/state 발행 주기를 확인하세요.'),
    (r'gap revalidation timed out', '정지 후 30초 안에 공간을 다시 확인하지 못했습니다',
     '정지 위치에서 공간이 가려지거나 카메라 시야를 벗어났는지 확인하세요.'),
    (r'no bay-contained goal has a safe reverse entry', '공간 안에 차를 세울 자세를 찾지 못했습니다',
     '주차 자세(차체 + 0.18 m) 안에 점유 셀이 있거나, 공간 앞쪽 약 7 m 도로가 막혔는지 확인하세요.'),
    (r'candidate search exhausted', '경로를 찾지 못했습니다',
     '간격이 좁거나(3.3 m 근처는 여러 번 전후진), 도로 폭·앞쪽 공간이 부족하거나, 미관측 영역이 넓은지 확인하세요.'),
    (r'latest RAW candidate precheck rejected', '경로가 최신 지도에서 0.18 m 여유를 지키지 못했습니다', ''),
    (r'final planned footprints outside measured target', '경로 끝 자세가 공간 밖입니다', ''),
    (r'streams stale before execution', '실행 직전에 센서가 끊겼습니다', ''),
    (r'current actual footprint overlaps latest RAW', '현재 차체가 지도상의 장애물/미관측과 겹칩니다',
     '자기 차체가 점으로 찍히는지, 가상 벽(공간 안쪽 끝) 쪽으로 너무 들어갔는지 확인하세요.'),
    (r'pose too old for closed-loop parking', '실행 중 위치 정보가 늦습니다', 'Cartographer TF 지연을 확인하세요.'),
    (r'replan budget exhausted', '재계획 한도(3회)를 넘었습니다', '경로 추종 오차, 지도 변화를 확인하세요.'),
    (r'articulation', '꺾임각 안전 한계를 넘었습니다', '트레일러 IMU 방향·오프셋을 확인하세요.'),
    (r'trailer IMU missing/stale|tractor IMU missing/stale', 'IMU 신호가 없습니다', 'IMU 포트와 연결을 확인하세요.'),
    (r'Cartographer actual scan matching missing', 'SLAM 스캔 매칭이 동작하지 않습니다', 'Cartographer 실행 상태를 확인하세요.'),
    (r'SLAM map unavailable', 'SLAM 지도가 없거나 점유 셀이 하나도 없습니다', ''),
    (r'scan missing/stale', 'LiDAR 스캔이 끊겼습니다', 'G6 드라이버와 포트를 확인하세요.'),
    (r'initial articulated alignment and trailer bias not confirmed', '차량 초기 정렬/바이어스 확인값이 false입니다',
     'vehicle_geometry.yaml의 initial_alignment_confirmed / trailer_bias_confirmed를 확인하세요.'),
    (r'explicit test stop command', '정지 버튼으로 멈췄습니다. 다시 하려면 종료 후 다시 실행하세요.', ''),
    (r'LiDAR-only diagnostics cannot start automatic motion', 'LiDAR만 진단 모드에서는 자동 주행을 시작할 수 없습니다',
     'LiDAR만 · 자동 탐색/주차 모드로 다시 실행하세요.'),
    (r'Need current SLAM/IMU/scan/camera and a stationary rig', '센서가 준비되지 않았거나 차가 1초간 정지해 있지 않습니다', ''),
]


def explain(text):
    """(설명, 확인할 것) for a status/reason string; unknown strings are returned as is."""
    text = str(text or '')
    for pattern, summary, hint in EXPLANATIONS:
        match = re.search(pattern, text)
        if match:
            return summary.format(*match.groups()), hint
    return text, ''


COLORS = np.array([[150, 150, 150], [245, 245, 245], [30, 30, 30], [110, 110, 110]], dtype=np.uint8)
UNKNOWN, FREE, OCCUPIED, OUTSIDE = range(4)


def render_view(data, resolution, origin, center, span, size):
    """RGB image (size x size) of the map around center (map frame), span metres wide.

    +x right, +y up (as RViz). When a pixel covers several cells they are reduced
    first: occupied if any cell is occupied, else free if any is free, else unknown,
    so thin walls survive zooming out. Returns (rgb, MapView).
    """
    data = np.asarray(data)
    view = MapView(center, span, size)
    step = max(1, int(view.mpp/resolution))
    if step > 1:
        h, w = data.shape
        H, W = -(-h//step), -(-w//step)
        padded = np.full((H*step, W*step), -1, dtype=np.int16)
        padded[:h, :w] = data
        blocks = padded.reshape(H, step, W, step)
        codes = np.where((blocks >= 50).any(axis=(1, 3)), OCCUPIED,
                         np.where(((blocks >= 0) & (blocks < 50)).any(axis=(1, 3)), FREE, UNKNOWN))
    else:
        codes = np.where(data >= 50, OCCUPIED, np.where(data >= 0, FREE, UNKNOWN))
    cell = resolution*step
    rows, cols = np.indices((size, size))
    x = view.center[0]+(cols+.5-size/2)*view.mpp
    y = view.center[1]-(rows+.5-size/2)*view.mpp
    c, s = math.cos(origin[2]), math.sin(origin[2])
    dx, dy = x-origin[0], y-origin[1]
    i = np.floor((c*dx+s*dy)/cell).astype(int)
    j = np.floor((-s*dx+c*dy)/cell).astype(int)
    inside = (i >= 0) & (j >= 0) & (i < codes.shape[1]) & (j < codes.shape[0])
    image = np.full((size, size), OUTSIDE, dtype=np.int8)
    image[inside] = codes[j[inside], i[inside]]
    return COLORS[image], view


def ppm(rgb):
    """Binary PPM bytes for tkinter.PhotoImage(data=..., format='PPM')."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    return f'P6 {rgb.shape[1]} {rgb.shape[0]} 255 '.encode('ascii')+rgb.tobytes()


class MapView:
    """Map-frame point -> pixel for a square view of span metres centred on center."""

    def __init__(self, center, span, size):
        self.center, self.size = np.asarray(center, dtype=float), int(size)
        self.mpp = float(span)/self.size

    def pixel(self, xy):
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        return np.c_[(xy[:, 0]-self.center[0])/self.mpp+self.size/2,
                     self.size/2-(xy[:, 1]-self.center[1])/self.mpp]
