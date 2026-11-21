"""Compact tkinter layout; no ROS, device access or launch side effects."""
from tkinter import ttk
import two_structure_gui_core as core


def scroll_frame(parent, tk):
    outer = ttk.Frame(parent)
    outer.pack(fill='both', expand=True)
    canvas = tk.Canvas(outer, highlightthickness=0, height=180)
    scroll = ttk.Scrollbar(outer, orient='vertical', command=canvas.yview)
    canvas.configure(yscrollcommand=scroll.set)
    scroll.pack(side='right', fill='y')
    canvas.pack(side='left', fill='both', expand=True)
    inner = ttk.Frame(canvas, padding=8)
    window = canvas.create_window(0, 0, anchor='nw', window=inner)
    inner.bind('<Configure>', lambda _: canvas.configure(scrollregion=canvas.bbox('all')))
    canvas.bind('<Configure>', lambda event: canvas.itemconfigure(window, width=event.width))
    return inner


def build(ui, root, map_size):
    tk = ui.tk
    style = ttk.Style(root)
    style.configure('Heading.TLabel', font=('', 15, 'bold'))
    style.configure('Section.TLabel', font=('', 10, 'bold'))
    style.configure('Muted.TLabel', foreground='#607080')
    header = ttk.Frame(root, padding=(14, 10, 14, 6))
    header.grid(row=0, column=0, columnspan=2, sticky='ew')
    ttk.Label(header, text='주차 테스트', style='Heading.TLabel').pack(side='left')
    ui.process_label = ttk.Label(header, text='대기 · 테스트 실행 전', style='Muted.TLabel')
    ui.process_label.pack(side='right')

    map_panel = ttk.Frame(root, padding=(14, 6, 8, 12))
    map_panel.grid(row=1, column=0, sticky='nsew')
    ui.canvas = tk.Canvas(map_panel, width=map_size, height=map_size,
                          bg='#969696', highlightthickness=0)
    ui.canvas.pack(anchor='n', pady=(0, 8))
    ui.canvas.create_text(map_size/2, map_size/2,
                          text='테스트 실행 후 관측 지도가 표시됩니다', fill='white')
    view = ttk.Frame(map_panel)
    view.pack(fill='x')
    ttk.Label(view, text='범위(m)').pack(side='left')
    ui.vars = {}
    ui.vars['span'] = tk.DoubleVar(value=float(ui.settings.get('span', 12.)))
    ttk.Scale(view, from_=4., to=40., variable=ui.vars['span'], length=170).pack(side='left', fill='x', expand=True)
    ui.vars['follow'] = tk.StringVar(value='bay')
    for value, label in [('bay', '공간 중심'), ('vehicle', '차량 중심')]:
        ttk.Radiobutton(view, text=label, value=value, variable=ui.vars['follow']).pack(side='left')
    ttk.Label(map_panel, text='초록 공간 · 노랑 후보 · 파랑 구조물 · 진회색 벽\n'
              '빨강 차량 · 주황 경로 / 흰색 빈 공간 · 검정 점유 · 회색 미관측',
              style='Muted.TLabel').pack(anchor='w', pady=(8, 0))

    side = ttk.Frame(root, padding=(8, 6, 14, 12))
    side.grid(row=1, column=1, sticky='nsew')
    ui.setting_widgets = []
    ui.vars['mode'] = tk.StringVar(value=ui.settings['mode'])
    ui.mode_choice = tk.StringVar(value=core.MODES[ui.settings['mode']][0])
    modes = ttk.Frame(side)
    modes.pack(fill='x')
    ttk.Label(modes, text='테스트 모드', style='Section.TLabel').pack(anchor='w')
    combo = ttk.Combobox(modes, textvariable=ui.mode_choice, values=list(core.MODE_BY_LABEL), state='readonly')
    combo.pack(fill='x', pady=(3, 6))
    combo.bind('<<ComboboxSelected>>', ui.selection_changed)
    ui.setting_widgets.append((combo, 'readonly'))
    outputs = ttk.Frame(side)
    outputs.pack(fill='x')
    ui.vars['enable_actuation'] = tk.BooleanVar(value=False)
    ui.actuation = ttk.Checkbutton(outputs, text='실차 출력 허용', variable=ui.vars['enable_actuation'],
                                   command=ui.actuation_changed)
    ui.actuation.pack(side='left')
    ui.vars['enable_rviz'] = tk.BooleanVar(value=bool(ui.settings.get('enable_rviz')))
    rviz = ttk.Checkbutton(outputs, text='RViz 창 추가', variable=ui.vars['enable_rviz'])
    rviz.pack(side='left', padx=(12, 0))
    ui.setting_widgets.extend([(ui.actuation, 'normal'), (rviz, 'normal')])
    ui.mode_hint = ttk.Label(side, style='Muted.TLabel')
    ui.mode_hint.pack(anchor='w', pady=(2, 7))

    buttons = ttk.Frame(side)
    buttons.pack(fill='x')
    ui.run_button = ttk.Button(buttons, text='테스트 실행', command=ui.launch)
    ui.run_button.pack(side='left', fill='x', expand=True)
    ttk.Button(buttons, text='테스트 종료', command=ui.terminate).pack(side='left', fill='x', expand=True, padx=(6, 0))
    actions = ttk.Frame(side)
    actions.pack(fill='x', pady=(6, 8))
    ui.primary_button = ttk.Button(actions, command=ui.start_action)
    ui.primary_button.pack(side='left', fill='x', expand=True)
    ui.stop_button = tk.Button(actions, text='정지', command=ui.stop,
                                bg='#b32d2d', fg='white', activebackground='#912323',
                                activeforeground='white', font=('', 11, 'bold'), width=14)
    ui.stop_button.pack(side='left', fill='x', expand=True, padx=(6, 0))
    ui.sequence_button = ttk.Button(side, text='주차 동작만 테스트…', command=ui.open_sequence)
    ui.sequence_button.pack(fill='x', pady=(0, 8))
    if ui.preview:
        ui.run_button.configure(state='disabled')
        ui.primary_button.configure(state='disabled')
        # This button can also stop the no-device timed sequence preview.

    ui.fields = {}
    summary = ttk.LabelFrame(side, text='진행 상황', padding=8)
    summary.pack(fill='x', pady=(0, 8))
    for row, (key, label) in enumerate([('stage', '단계'), ('recognition', '공간 인식'), ('failure', '주차 / 정지 사유')]):
        ttk.Label(summary, text=label, style='Section.TLabel').grid(row=row, column=0, sticky='nw', padx=(0, 10), pady=3)
        ui.fields[key] = ttk.Label(summary, text='-', wraplength=370, justify='left')
        ui.fields[key].grid(row=row, column=1, sticky='ew', pady=3)
    summary.columnconfigure(1, weight=1)

    ui.notebook = ttk.Notebook(side)
    ui.notebook.pack(fill='both', expand=True)
    tabs = {}
    for title in ('상태 상세', '연결 설정', '배치 안내', '로그'):
        tab = ttk.Frame(ui.notebook)
        ui.notebook.add(tab, text=title)
        tabs[title] = tab
    details = scroll_frame(tabs['상태 상세'], tk)
    for key, label in [('hint', '확인할 것'), ('bay', '공간 정보'), ('camera', '카메라'),
                       ('estimator', '위치 추정'), ('response', '마지막 명령 응답')]:
        ttk.Label(details, text=label, style='Section.TLabel').pack(anchor='w', pady=(6, 2))
        ui.fields[key] = ttk.Label(details, text='-', wraplength=440, justify='left')
        ui.fields[key].pack(anchor='w', fill='x')
    connections = scroll_frame(tabs['연결 설정'], tk)
    for row, (key, label) in enumerate([
            ('workspace', '워크스페이스'), ('lidar_port', 'LiDAR'),
            ('tractor_imu_port', '견인차 IMU'), ('trailer_imu_port', '트레일러 IMU'),
            ('camera_path', '카메라'), ('calibration_file', '카메라 보정'),
            ('detector_model', '전동차 ONNX 모델'), ('detector_confidence', '차체 검출 신뢰도'),
            ('arduino_port', 'Arduino'), ('slot_width', '주차 깊이(m)'),
            ('ROS_DOMAIN_ID', 'ROS_DOMAIN_ID'), ('ROS_LOCALHOST_ONLY', 'ROS_LOCALHOST_ONLY')]):
        ttk.Label(connections, text=label).grid(row=row, column=0, sticky='w', padx=(0, 8), pady=3)
        value = str(ui.settings.get(key, ''))
        if key == 'detector_model':
            ui.vars[key] = tk.StringVar(value=value or core.ORIGINAL_DETECTOR)
            entry = ttk.Combobox(connections, textvariable=ui.vars[key], width=30,
                                 values=(core.ORIGINAL_DETECTOR, core.WHEELS_DETECTOR))
        else:
            ui.vars[key] = tk.StringVar(value=value)
            entry = ttk.Entry(connections, textvariable=ui.vars[key], width=30)
        entry.grid(row=row, column=1, sticky='ew', pady=3)
        ui.setting_widgets.append((entry, 'normal'))
    connections.columnconfigure(1, weight=1)
    ttk.Label(connections, text='전동차 모델을 선택하거나 ONNX 경로를 입력하세요.\n'
              '주차 깊이 빈칸 = YAML 사용\nROS 도메인 변경 후에는 창을 다시 여세요.',
              style='Muted.TLabel').grid(row=12, column=0, columnspan=2, sticky='w', pady=6)
    ui.guide = tk.Text(tabs['배치 안내'], wrap='word', height=10, padx=8, pady=8,
                       relief='flat', state='disabled')
    guide_scroll = ttk.Scrollbar(tabs['배치 안내'], command=ui.guide.yview)
    ui.guide.configure(yscrollcommand=guide_scroll.set)
    guide_scroll.pack(side='right', fill='y')
    ui.guide.pack(fill='both', expand=True)
    ui.vars['full_log'] = tk.BooleanVar(value=False)
    ttk.Checkbutton(tabs['로그'], text='전체 로그', variable=ui.vars['full_log']).pack(anchor='w')
    ui.log = tk.Text(tabs['로그'], height=8, wrap='word', relief='flat')
    log_scroll = ttk.Scrollbar(tabs['로그'], command=ui.log.yview)
    ui.log.configure(yscrollcommand=log_scroll.set)
    log_scroll.pack(side='right', fill='y')
    ui.log.pack(fill='both', expand=True)
    root.columnconfigure(1, weight=1)
    root.rowconfigure(1, weight=1)
