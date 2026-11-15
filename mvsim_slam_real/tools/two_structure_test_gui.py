#!/usr/bin/env python3
"""Two-structure parking test window.

Choose the test mode (LiDAR only / camera+LiDAR, hand push / automatic), launch and stop
the stack, send start/stop commands, and watch the recognised bay on the detection map
with a plain-language reason when recognition or parking fails.
Run with tools/run_two_structure_gui.sh (sources ROS and the workspace).
"""
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import two_structure_gui_core as core  # noqa: E402

CONFIG = Path.home()/'.config'/'two_structure_test_gui.json'
WORKSPACE = Path(__file__).resolve().parents[1]
LOG_KEYS = ('[Parking]', 'Waiting for map target', 'Target center', 'Plan found', '4D Hybrid A*',
            'ERROR', 'Error', 'error:', 'Traceback', 'FAILED', 'EMERGENCY', 'Exception', 'process has died')
MAP_SIZE = 540
LOG_LINES = 3000
PR_SET_PDEATHSIG = 1


def child_setup():
    """Runs in the launch child: new process group + SIGINT when this window's process dies."""
    os.setsid()
    try:
        import ctypes
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)
    except OSError:
        pass


def quaternion_yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class RosLink:
    """Minimal rclpy client: latest topic values and Trigger service calls."""

    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import OccupancyGrid, Path as RosPath
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.rclpy, self.Trigger = rclpy, Trigger
        rclpy.init()
        self.node = rclpy.create_node('two_structure_test_gui')
        self.latest, self.pending = {}, []
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        for topic, kind, key in [('/parking/status', String, 'status'),
                                 ('/parking_test/observation', String, 'observation'),
                                 ('/parking/state', String, 'state'),
                                 ('/parking/detection_map', OccupancyGrid, 'map'),
                                 ('/parking/planned_tractor_path', RosPath, 'path')]:
            self.node.create_subscription(kind, topic, lambda m, k=key: self.store(k, m), reliable)
        self.clients = {name: self.node.create_client(Trigger, '/parking_test/'+name)
                        for name in ('start_search', 'start_parking', 'stop')}

    def store(self, key, message):
        if hasattr(message, 'data') and isinstance(message.data, str):
            try:
                message = json.loads(message.data)
            except ValueError:
                return
        self.latest[key] = (time.monotonic(), message)

    def stack_running(self):
        """True when a two-structure test node already offers its services on this domain."""
        return any(name == '/parking_test/start_search' for name, _ in self.node.get_service_names_and_types())

    def call(self, name, done):
        client = self.clients[name]
        if not client.service_is_ready():
            done(False, '서비스가 없습니다. 테스트가 실행 중인지 확인하세요.')
            return
        self.pending.append((client.call_async(self.Trigger.Request()), done))

    def spin(self):
        for _ in range(20):
            self.rclpy.spin_once(self.node, timeout_sec=0)
        for item in list(self.pending):
            future, done = item
            if future.done():
                self.pending.remove(item)
                try:
                    result = future.result()
                    done(result.success, result.message)
                except Exception as exc:  # noqa: BLE001 - report any service failure
                    done(False, str(exc))

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class TestWindow:
    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.root = tk, root
        self.process, self.lines = None, queue.Queue()
        self.photo, self.last_render, self.map_key, self.map_data = None, 0., None, None
        self.settings = dict(core.DEFAULT_SETTINGS, workspace=str(WORKSPACE))
        try:
            self.settings.update(json.loads(CONFIG.read_text()))
        except (OSError, ValueError):
            pass
        # A desktop shortcut does not read ~/.bashrc: apply the saved ROS network settings
        # before rclpy starts, so this window and the launched stack use the same domain.
        for name in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY'):
            value = str(self.settings.get(name, '')).strip()
            if value:
                os.environ[name] = value
            self.settings[name] = os.environ.get(name, '')
        try:
            self.ros = RosLink()
            ros_error = ''
        except Exception as exc:  # noqa: BLE001 - window still usable for launching
            self.ros, ros_error = None, f'ROS 연결 실패: {exc} (run_two_structure_gui.sh로 실행하세요)'
        root.title('두 구조물 주차 테스트')
        root.geometry('1500x1000')
        root.protocol('WM_DELETE_WINDOW', self.close)

        left = ttk.Frame(root, padding=8)
        left.grid(row=0, column=0, sticky='ns')
        self.vars = {}
        ttk.Label(left, text='테스트 모드', font=('', 11, 'bold')).pack(anchor='w')
        self.vars['mode'] = tk.StringVar(value=self.settings['mode'])
        for key, (label, _) in core.MODES.items():
            ttk.Radiobutton(left, text=label, value=key, variable=self.vars['mode'],
                            command=self.mode_changed).pack(anchor='w')
        ttk.Separator(left).pack(fill='x', pady=6)
        for key, label in [('workspace', '워크스페이스'), ('lidar_port', 'LiDAR 포트'),
                           ('tractor_imu_port', '견인차 IMU 포트'), ('trailer_imu_port', '트레일러 IMU 포트'),
                           ('camera_path', '카메라 장치'), ('calibration_file', '카메라 보정 파일'),
                           ('arduino_port', 'Arduino 포트'), ('slot_width', '주차 깊이 m (빈칸=YAML)'),
                           ('ROS_DOMAIN_ID', 'ROS_DOMAIN_ID (바꾸면 창 다시 열기)'),
                           ('ROS_LOCALHOST_ONLY', 'ROS_LOCALHOST_ONLY (1=이 PC만)')]:
            ttk.Label(left, text=label).pack(anchor='w')
            self.vars[key] = tk.StringVar(value=str(self.settings.get(key, '')))
            ttk.Entry(left, textvariable=self.vars[key], width=38).pack(anchor='w')
        self.vars['enable_actuation'] = tk.BooleanVar(value=False)  # never remembered as on
        self.actuation = ttk.Checkbutton(left, text='모터 실제 구동 (밀기 모드는 [주차 시작] 이후만)', variable=self.vars['enable_actuation'],
                                         command=self.actuation_changed)
        self.actuation.pack(anchor='w', pady=(6, 0))
        self.vars['enable_rviz'] = tk.BooleanVar(value=bool(self.settings.get('enable_rviz')))
        ttk.Checkbutton(left, text='RViz도 열기', variable=self.vars['enable_rviz']).pack(anchor='w')
        row = ttk.Frame(left)
        row.pack(fill='x', pady=6)
        self.run_button = ttk.Button(row, text='실행', command=self.launch)
        self.run_button.pack(side='left', expand=True, fill='x')
        ttk.Button(row, text='종료', command=self.terminate).pack(side='left', expand=True, fill='x')
        row = ttk.Frame(left)
        row.pack(fill='x')
        ttk.Button(row, text='탐색 시작', command=lambda: self.service('start_search')).pack(side='left', expand=True, fill='x')
        ttk.Button(row, text='주차 시작(밀기 후)', command=lambda: self.service('start_parking')).pack(side='left', expand=True, fill='x')
        tk.Button(left, text='정지', bg='#c0392b', fg='white', font=('', 12, 'bold'),
                  command=lambda: self.service('stop')).pack(fill='x', pady=6)
        self.process_label = ttk.Label(left, text='실행 안 됨', wraplength=300)
        self.process_label.pack(anchor='w')

        middle = ttk.Frame(root, padding=8)
        middle.grid(row=0, column=1, sticky='n')
        self.canvas = tk.Canvas(middle, width=MAP_SIZE, height=MAP_SIZE, bg='#969696')
        self.canvas.pack()
        row = ttk.Frame(middle)
        row.pack(fill='x')
        ttk.Label(row, text='보기 범위(m)').pack(side='left')
        self.vars['span'] = tk.DoubleVar(value=float(self.settings.get('span', 12.)))
        ttk.Scale(row, from_=4., to=40., variable=self.vars['span'], length=220).pack(side='left')
        self.vars['follow'] = tk.StringVar(value='bay')
        for value, label in [('bay', '공간 중심'), ('vehicle', '차량 중심')]:
            ttk.Radiobutton(row, text=label, value=value, variable=self.vars['follow']).pack(side='left')
        ttk.Label(middle, text='초록=확정 공간  노랑=후보  파랑=구조물  진회색=벽  빨강=차량  주황=계획 경로\n'
                               '흰색=빈 공간  검정=점유  회색=미관측').pack(anchor='w')

        right = ttk.Frame(root, padding=8)
        right.grid(row=0, column=2, sticky='nsew')
        box = ttk.LabelFrame(right, text='배치 안내 (실행 전 확인)', padding=6)
        box.pack(fill='x')
        self.guide = ttk.Label(box, text='', wraplength=540, justify='left')
        self.guide.pack(anchor='w')
        self.fields = {}
        for key, label in [('stage', '현재 단계'), ('recognition', '공간 인식'), ('hint', '확인할 것'),
                           ('bay', '공간 정보'), ('camera', '카메라'), ('failure', '실패/정지 사유'),
                           ('estimator', '위치 추정'), ('response', '마지막 명령 응답')]:
            ttk.Label(right, text=label, font=('', 10, 'bold')).pack(anchor='w', pady=(6, 0))
            self.fields[key] = ttk.Label(right, text='-', wraplength=520, justify='left')
            self.fields[key].pack(anchor='w')

        bottom = ttk.Frame(root, padding=8)
        bottom.grid(row=1, column=0, columnspan=3, sticky='nsew')
        self.vars['full_log'] = tk.BooleanVar(value=False)
        ttk.Checkbutton(bottom, text='전체 로그 보기 (끄면 상태 변화·오류만)', variable=self.vars['full_log']).pack(anchor='w')
        self.log = tk.Text(bottom, height=12, wrap='none')
        self.log.pack(fill='both', expand=True)
        root.grid_rowconfigure(1, weight=1)
        root.grid_columnconfigure(2, weight=1)
        self.mode_changed()
        if ros_error:
            self.append(ros_error)
        self.root.after(100, self.tick)

    # --- settings / process -------------------------------------------------
    def current_settings(self):
        values = {k: v.get() for k, v in self.vars.items() if k not in ('full_log', 'follow')}
        return dict(self.settings, **values)

    def mode_changed(self):
        geometry = Path(self.vars['workspace'].get())/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'
        self.guide.config(text=core.placement_guide(self.vars['mode'].get(), core.lidar_height(geometry)))

    def actuation_changed(self):
        from tkinter import messagebox
        if self.vars['enable_actuation'].get() and not messagebox.askyesno(
                '모터 구동 확인', '실제로 모터를 구동합니다. 주변이 안전하고 정지 버튼을 바로 누를 수 있나요?'):
            self.vars['enable_actuation'].set(False)

    def launch(self):
        from tkinter import messagebox
        if self.process is not None and self.process.poll() is None:
            messagebox.showinfo('실행 중', '이미 실행 중입니다. 먼저 종료하세요.')
            return
        settings = self.current_settings()
        try:
            arguments = core.launch_arguments(settings)
        except ValueError as exc:
            messagebox.showerror('설정 오류', str(exc))
            return
        workspace = Path(settings['workspace'])
        if not (workspace/'install/setup.bash').exists():
            messagebox.showerror('빌드 필요', f'{workspace}/install/setup.bash가 없습니다. colcon build를 먼저 하세요.')
            return
        if str(settings.get('ROS_DOMAIN_ID', '')) != os.environ.get('ROS_DOMAIN_ID', ''):
            messagebox.showerror('ROS_DOMAIN_ID 변경됨', '창을 닫고 다시 열어야 새 ROS_DOMAIN_ID가 적용됩니다.')
            self.save(settings)
            return
        if self.ros is not None and self.ros.stack_running() and not messagebox.askyesno(
                '이미 실행 중', '같은 ROS 도메인에 테스트 노드가 이미 있습니다(다른 터미널/창).\n'
                '중복 실행하면 오동작합니다. 그래도 실행할까요?'):
            return
        missing = [settings[k] for k in ('lidar_port', 'tractor_imu_port', 'trailer_imu_port', 'arduino_port', 'camera_path')
                   if settings.get(k) and str(settings[k]).startswith('/dev/') and not os.path.exists(settings[k])
                   and (k != 'arduino_port' or settings.get('enable_actuation'))]
        if missing and not messagebox.askyesno('장치 없음', '없는 장치: '+', '.join(missing)+'\n그래도 실행할까요?'):
            return
        self.save(settings)
        command = core.launch_shell(workspace, arguments)
        self.append('$ ros2 launch ... '+' '.join(arguments))
        # Own process group (Ctrl+C to the whole launch) and SIGINT to the launch if this
        # window dies, so the stack never keeps running without its stop button.
        self.process = subprocess.Popen(['bash', '-lc', command], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, bufsize=1, errors='replace', preexec_fn=child_setup)
        threading.Thread(target=self.reader, args=(self.process,), daemon=True).start()
        self.process_label.config(text=f'실행 중 (모드: {core.MODES[settings["mode"]][0]})')

    def save(self, settings):
        saved = {k: v for k, v in settings.items() if k != 'enable_actuation'}
        try:
            CONFIG.parent.mkdir(parents=True, exist_ok=True)
            CONFIG.write_text(json.dumps(saved, ensure_ascii=False, indent=1))
        except OSError:
            pass

    def reader(self, process):
        for line in process.stdout:
            self.lines.put(line.rstrip())
        self.lines.put(f'[launch 종료, 코드 {process.wait()}]')

    def terminate(self):
        process = self.process
        if process is None or process.poll() is not None:
            return
        self.append('[종료 요청: Ctrl+C 전달]')
        os.killpg(process.pid, signal.SIGINT)

        def escalate(sig, delay):
            if process.poll() is None:
                self.append(f'[아직 종료 안 됨: {sig.name} 전달]')
                os.killpg(process.pid, sig)
                if sig != signal.SIGKILL:
                    self.root.after(delay, lambda: escalate(signal.SIGKILL, 0))
        self.root.after(8000, lambda: escalate(signal.SIGTERM, 4000))

    def service(self, name):
        if self.ros is None:
            self.fields['response'].config(text='ROS 연결이 없어 명령을 보낼 수 없습니다.')
            return

        def done(success, message):
            summary, hint = core.explain(message)
            text = ('성공: ' if success else '거부: ')+summary+(f'\n→ {hint}' if hint else '')
            self.fields['response'].config(text=text)
            self.append(f'[{name}] {"OK" if success else "거부"}: {message}')
        self.ros.call(name, done)

    def append(self, line):
        self.log.insert('end', line+'\n')
        # Bounded log so long runs do not slow the window down.
        if int(self.log.index('end-1c').split('.')[0]) > LOG_LINES:
            self.log.delete('1.0', f'{LOG_LINES//5}.0')
        self.log.see('end')

    # --- periodic update ----------------------------------------------------
    def tick(self):
        try:
            # The reader thread always drains the launch pipe; here at most a batch per tick.
            for _ in range(400):
                if self.lines.empty():
                    break
                line = self.lines.get()
                if self.vars['full_log'].get() or any(key in line for key in LOG_KEYS):
                    self.append(line)
            if self.process is not None and self.process.poll() is not None:
                self.process_label.config(text=f'종료됨 (코드 {self.process.returncode})')
            if self.ros is not None:
                self.ros.spin()
                self.update_status()
                if time.monotonic()-self.last_render > .5:
                    self.last_render = time.monotonic()
                    self.draw()
        except Exception as exc:  # noqa: BLE001 - a display error must never stop updates/buttons
            if str(exc) != getattr(self, 'last_tick_error', ''):
                self.last_tick_error = str(exc)
                self.append(f'[창 표시 오류, 계속 동작: {type(exc).__name__}: {exc}]')
        self.root.after(100, self.tick)

    def fresh(self, key, age=3.):
        item = self.ros.latest.get(key)
        return item[1] if item and time.monotonic()-item[0] <= age else None

    def update_status(self):
        status = self.fresh('status', 30.)
        observation = self.fresh('observation')
        state = self.fresh('state', 1.)
        if status:
            self.fields['stage'].config(text=str(status.get('state', '-')))
            failure = status.get('failure') or ''
            summary, hint = core.explain(failure)
            self.fields['failure'].config(text=(summary+(f'\n→ {hint}' if hint else '')) if failure else '-')
        if observation:
            summary, hint = core.explain(observation.get('reason'))
            self.fields['recognition'].config(text=('✔ ' if observation.get('valid') else '✘ ')+summary,
                                              foreground='#1e8449' if observation.get('valid') else '#b03a2e')
            rejected = observation.get('rejected_pairs') or []
            self.fields['hint'].config(text=(hint or '-')+(('\n버린 쌍: '+'; '.join(rejected)) if rejected else ''))
            if observation.get('length'):
                depth = observation.get('rear_depth_m')
                wall = f'뒤 벽까지 {depth:.2f} m (벽에 맞춤)' if depth is not None else '뒤 벽 없음 (설정 깊이)'
                self.fields['bay'].config(text=f'간격 {observation["length"]:.2f} m × 깊이 {observation["width"]:.2f} m, '
                                               f'{wall}, 구조물 후보 {observation.get("structures_found", 0)}개, '
                                               f'내부 빈칸 비율 {observation.get("free_fraction", 0):.2f}')
            else:
                self.fields['bay'].config(text=f'구조물 후보 {observation.get("structures_found", 0)}개')
            camera = ('확인됨' if observation.get('camera_confirmed') else '미확인')
            camera += (' · 필수' if observation.get('require_camera') else
                       ' · 참고용 (LiDAR만 주행 허용)' if observation.get('lidar_only_motion') else ' · 참고용')
            camera += '' if observation.get('camera_stream_ready') else ' · 카메라 스트림 없음'
            self.fields['camera'].config(text=camera)
        elif not status:
            self.fields['recognition'].config(text='관측 메시지 없음 (테스트가 실행 중인지, 대기/탐색 단계인지 확인)',
                                              foreground='')
        if state is not None:
            self.fields['estimator'].config(text='정상' if state.get('valid') else core.explain(state.get('reason'))[0])
        elif self.ros.latest:
            self.fields['estimator'].config(text='/parking/state 수신 없음')

    def draw(self):
        message = self.fresh('map', 10.)
        canvas = self.canvas
        if message is None:
            return
        import numpy as np
        info = message.info
        key = (message.header.stamp.sec, message.header.stamp.nanosec, info.width, info.height)
        if key != self.map_key:
            self.map_data = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
            self.map_key = key
        o = info.origin
        origin = [o.position.x, o.position.y, quaternion_yaw(o.orientation)]
        observation = self.fresh('observation') or {}
        state = self.fresh('state', 1.)
        # Follow the bay (or the vehicle); fall back to the map centre.
        center = None
        if self.vars['follow'].get() == 'bay' and observation.get('bay_polygon'):
            center = np.mean(observation['bay_polygon'], axis=0)
        elif state and state.get('valid') and state.get('state'):
            center = np.asarray(state['state'][:2])
        if center is None:
            c, s = math.cos(origin[2]), math.sin(origin[2])
            half = np.array([info.width, info.height])*info.resolution/2
            center = np.array(origin[:2])+[c*half[0]-s*half[1], s*half[0]+c*half[1]]
        rgb, self.view = core.render_view(self.map_data, info.resolution, origin, center,
                                          self.vars['span'].get(), MAP_SIZE)
        self.photo = self.tk.PhotoImage(data=core.ppm(rgb), format='PPM')
        canvas.delete('all')
        canvas.create_image(0, 0, image=self.photo, anchor='nw')
        for wall in observation.get('wall_points') or []:
            self.dots(wall, '#404040', 1)
        for structure in observation.get('structure_points') or []:
            self.dots(structure, '#1f4fd1', 2)
        if observation.get('bay_polygon'):
            points = self.view.pixel(observation['bay_polygon'])
            canvas.create_polygon(*points.ravel().tolist(), outline='#1e8449' if observation.get('valid') else '#d4ac0d',
                                  fill='', width=3)
        path = self.fresh('path', 600.)
        if path is not None and len(path.poses) > 1:
            points = self.view.pixel([[p.pose.position.x, p.pose.position.y] for p in path.poses])
            canvas.create_line(*points.ravel().tolist(), fill='#e67e22', width=2)
        if state and state.get('valid') and state.get('state'):
            x, y, yaw = state['state'][:3]
            (px, py), (qx, qy) = self.view.pixel([[x, y], [x+.5*math.cos(yaw), y+.5*math.sin(yaw)]])
            canvas.create_line(px, py, qx, qy, fill='#c0392b', width=3, arrow='last')
            canvas.create_oval(px-4, py-4, px+4, py+4, fill='#c0392b', outline='')

    def dots(self, points, color, radius):
        for px, py in self.view.pixel(points):
            self.canvas.create_rectangle(px-radius, py-radius, px+radius, py+radius, fill=color, outline='')

    def close(self):
        from tkinter import messagebox
        if self.process is not None and self.process.poll() is None:
            if not messagebox.askyesno('종료', '테스트가 실행 중입니다. 종료하고 창을 닫을까요?'):
                return
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
        if self.ros is not None:
            self.ros.close()
        self.root.destroy()


def main():
    import tkinter as tk
    root = tk.Tk()
    TestWindow(root)
    root.mainloop()


if __name__ == '__main__':
    main()
