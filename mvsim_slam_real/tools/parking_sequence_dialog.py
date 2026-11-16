"""Review a nominal fixed-PWM sequence before a standalone actuator test."""
import copy
import tkinter as tk
from tkinter import messagebox, ttk

import parking_sequence_test as sequence


class SequenceDialog:
    def __init__(self, ui):
        self.ui = ui
        self.window = tk.Toplevel(ui.root)
        self.window.title('주차 동작만 테스트 · PWM 50')
        self.window.geometry('770x690')
        self.window.minsize(680, 620)
        self.window.transient(ui.root)
        self.window.grab_set()
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.window.columnconfigure(0, weight=1)
        self.window.rowconfigure(3, weight=1)
        body = ttk.Frame(self.window, padding=14)
        body.grid(row=0, column=0, sticky='ew')
        ttk.Label(body, text='주차 동작만 테스트', font=('', 14, 'bold')).pack(anchor='w')
        actual = ui.vars['enable_actuation'].get() and not ui.preview
        ttk.Label(body, text='실차 출력: ON · Arduino 직접 송신' if actual else
                  '실차 출력: OFF · 시간 흐름만 미리보기').pack(anchor='w', pady=(4, 0))
        ttk.Label(body, text='인식·SLAM·경로 생성 없이 저장된 조향 순서를 실행합니다.\n'
                  '합성 예시 기반 · 시간은 미측정 가정 · 장애물 감지/위치 보정 없음',
                  foreground='#8d4b00').pack(anchor='w', pady=(5, 0))
        inputs = ttk.LabelFrame(self.window, text='50 PWM에서의 가정 속도 → 이동시간 계산', padding=8)
        inputs.grid(row=1, column=0, sticky='ew', padx=14)
        initial = sequence.build_sequence()
        self.forward = tk.StringVar(value=str(initial['nominal_forward_mps']))
        self.reverse = tk.StringVar(value=str(initial['nominal_reverse_mps']))
        self.editors = []
        for column, (caption, variable) in enumerate([('전진(m/s)', self.forward), ('후진(m/s)', self.reverse)]):
            ttk.Label(inputs, text=caption).grid(row=0, column=column*2, padx=(0, 5))
            editor = ttk.Entry(inputs, textvariable=variable, width=10)
            editor.grid(row=0, column=column*2+1, padx=(0, 12))
            self.editors.append(editor)
        self.recalculate = ttk.Button(inputs, text='시간 다시 계산', command=self.rebuild)
        self.recalculate.grid(row=0, column=4)
        self.summary = ttk.Label(self.window)
        self.summary.grid(row=2, column=0, sticky='w', padx=14, pady=8)
        table_frame = ttk.Frame(self.window, padding=(14, 0))
        table_frame.grid(row=3, column=0, sticky='nsew')
        self.table = ttk.Treeview(table_frame, columns=('number', 'action', 'steering', 'pwm', 'seconds'),
                                  show='headings', height=14, selectmode='browse')
        for key, title, width in [('number', '#', 35), ('action', '동작', 150), ('steering', '조향 단계', 95),
                                   ('pwm', '좌/우 PWM', 100), ('seconds', '유지시간(초)', 100)]:
            self.table.heading(key, text=title)
            self.table.column(key, width=width, anchor='center')
        scrollbar = ttk.Scrollbar(table_frame, command=self.table.yview)
        self.table.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        self.table.pack(fill='both', expand=True)
        self.table.bind('<<TreeviewSelect>>', self.selected)
        adjust = ttk.Frame(self.window, padding=(14, 8))
        adjust.grid(row=4, column=0, sticky='ew')
        ttk.Label(adjust, text='선택 구간 시간(초)').pack(side='left')
        self.seconds = tk.StringVar()
        entry = ttk.Entry(adjust, textvariable=self.seconds, width=9)
        entry.pack(side='left', padx=6)
        self.editors.append(entry)
        self.apply = ttk.Button(adjust, text='적용', command=self.adjust)
        self.apply.pack(side='left')
        ttk.Label(adjust, text='0.05초 단위 · 조향/전후진 대기 유지').pack(side='left', padx=10)
        self.status = ttk.Label(self.window, text='시작 전 전체 순서와 주변 공간을 확인하세요.', wraplength=700)
        self.status.grid(row=5, column=0, sticky='w', padx=14)
        buttons = ttk.Frame(self.window, padding=14)
        buttons.grid(row=6, column=0, sticky='ew')
        self.start_button = ttk.Button(buttons, text='실차 시퀀스 시작' if actual else '시퀀스 미리보기 시작', command=self.start)
        self.start_button.pack(side='left', expand=True, fill='x')
        tk.Button(buttons, text='정지', bg='#b32d2d', fg='white', command=ui.stop,
                  width=14).pack(side='left', padx=(8, 0), fill='x')
        self.busy = False
        self.program = initial
        self.populate()

    def populate(self):
        self.table.delete(*self.table.get_children())
        for index, frame in enumerate(self.program['frames']):
            step, pwm = frame['steering'], frame['left_pwm']
            steering = '중앙' if not step else f'{"왼쪽" if step < 0 else "오른쪽"} {abs(step)}'
            self.table.insert('', 'end', iid=str(index), values=(index+1, '정지 / 조향 대기' if not pwm else
                              '전진' if pwm > 0 else '후진', steering, f'{pwm:+d} / {pwm:+d}', f'{frame["ticks"]*sequence.DT:.2f}'))
        summary = sequence.validate_sequence(self.program)
        self.summary.configure(text=f"PWM ±50 · {summary['frames']}구간 · 총 {summary['duration_s']:.2f}초 · 실측값 아님")

    def selected(self, _event=None):
        selected = self.table.selection()
        if selected:
            self.seconds.set(f"{self.program['frames'][int(selected[0])]['ticks']*sequence.DT:.2f}")

    def rebuild(self):
        if self.busy:
            return
        try:
            program = sequence.build_sequence(float(self.forward.get()), float(self.reverse.get()))
        except (ValueError, KeyError) as exc:
            messagebox.showerror('시간 계산 오류', str(exc), parent=self.window)
            return
        self.program = program
        self.populate()

    def adjust(self):
        if self.busy or not self.table.selection():
            return
        try:
            seconds = sequence.positive(float(self.seconds.get()), 'seconds')
            program = copy.deepcopy(self.program)
            program['frames'][int(self.table.selection()[0])]['ticks'] = round(seconds/sequence.DT)
            sequence.validate_sequence(program)
        except (ValueError, OverflowError) as exc:
            messagebox.showerror('시간 설정 오류', str(exc), parent=self.window)
            return
        self.program = program
        self.populate()

    def start(self):
        self.ui.launch_sequence(copy.deepcopy(self.program), self)

    def set_busy(self, busy):
        self.busy = busy
        for widget in [*self.editors, self.recalculate, self.apply, self.start_button]:
            widget.configure(state='disabled' if busy else 'normal')

    def progress(self, line):
        self.status.configure(text=line)
        try:
            position = line.split('] ', 1)[1].split('/', 1)[0]
            index = str(int(position)-1)
            if self.table.exists(index):
                self.table.selection_set(index)
                self.table.see(index)
        except (ValueError, IndexError):
            pass

    def close(self):
        if self.busy:
            self.ui.stop()
        self.ui.sequence_dialog = None
        self.window.destroy()
