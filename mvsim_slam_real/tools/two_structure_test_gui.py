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
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import two_structure_gui_core as core  # noqa: E402

CONFIG = Path.home()/'.config'/'two_structure_test_gui.json'
WORKSPACE = Path(__file__).resolve().parents[1]
LOG_KEYS = ('[Parking]', 'Waiting for map target', 'Target center', 'Plan found', '4D Hybrid A*',
            'ERROR', 'Error', 'error:', 'Traceback', 'FAILED', 'EMERGENCY', 'Exception', 'process has died')
MAP_SIZE = 500
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

    def motion_source_running(self):
        return (self.stack_running()
                or bool(self.node.get_publishers_info_by_topic('/topic_control_signal'))
                or any(name in ('control_adapter', 'serial_sender_node', 'auto_parking', 'manual_motor_bridge')
                       for name, _ in self.node.get_node_names_and_namespaces()))

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
    def __init__(self, root, preview=False):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.root = tk, root
        self.preview, self.running_settings = preview, None
        self.ui_running = False
        self.process, self.lines = None, queue.Queue()
        self.process_kind, self.sequence_dialog, self.sequence_file = 'stack', None, None
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
            self.ros = None if preview else RosLink()
            ros_error = ''
        except Exception as exc:  # noqa: BLE001 - window still usable for launching
            self.ros, ros_error = None, f'ROS 연결 실패: {exc} (run_two_structure_gui.sh로 실행하세요)'
        root.title('두 구조물 주차 테스트')
        root.geometry('1180x740')
        root.minsize(1080, 680)
        root.protocol('WM_DELETE_WINDOW', self.close)

        from two_structure_gui_layout import build
        build(self, root, MAP_SIZE)
        self.mode_changed()
        if ros_error:
            self.append(ros_error)
            self.process_label.configure(text='ROS 연결 없음 · 로그 확인')
            self.fields['estimator'].configure(text=ros_error)
        if preview:
            self.process_label.configure(text='미리보기 · ROS/장치 연결 없음')
        self.root.after(100, self.tick)

    # --- settings / process -------------------------------------------------
    def current_settings(self):
        values = {k: v.get() for k, v in self.vars.items() if k not in ('full_log', 'follow')}
        return dict(self.settings, **values)

    def running_mode(self):
        return (self.running_settings or self.current_settings())['mode']

    def selection_changed(self, event=None):
        self.vars['mode'].set(core.mode_from_choices(self.sensor_choice.get(), self.survey_choice.get()))
        self.mode_changed()

    def start_action(self):
        self.service(core.primary_action(self.running_mode())[0])

    def set_running(self, running):
        if self.ui_running == running:
            return
        self.ui_running = running
        for widget, idle_state in self.setting_widgets:
            widget.configure(state='disabled' if running else idle_state)
        self.run_button.configure(state='disabled' if running or self.preview else 'normal')
        self.sequence_button.configure(state='disabled' if running else 'normal')
        self.primary_button.configure(state='disabled' if self.preview or (running and self.process_kind == 'sequence') else 'normal')
        if self.sequence_dialog is not None:
            self.sequence_dialog.set_busy(running and self.process_kind == 'sequence')
        if not running:
            self.running_settings = None
            self.mode_changed()

    def mode_changed(self):
        geometry = Path(self.vars['workspace'].get())/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'
        self.guide.configure(state='normal')
        self.guide.delete('1.0', 'end')
        self.guide.insert('1.0', core.placement_guide(self.vars['mode'].get(), core.lidar_height(geometry)))
        self.guide.configure(state='disabled')
        self.primary_button.configure(text=core.primary_action(self.vars['mode'].get())[1])

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
        if self.preview:
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
        self.process_kind = 'stack'
        self.append('$ ros2 launch ... '+' '.join(arguments))
        # Own process group (Ctrl+C to the whole launch) and SIGINT to the launch if this
        # window dies, so the stack never keeps running without its stop button.
        self.process = subprocess.Popen(['bash', '-lc', command], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, bufsize=1, errors='replace', preexec_fn=child_setup)
        threading.Thread(target=self.reader, args=(self.process,), daemon=True).start()
        self.running_settings = dict(settings)
        self.set_running(True)
        self.process_label.config(text='실행 중 · '+core.MODES[settings['mode']][0])

    def open_sequence(self):
        if self.sequence_dialog is not None:
            self.sequence_dialog.window.lift()
            return
        from parking_sequence_dialog import SequenceDialog
        self.sequence_dialog = SequenceDialog(self)

    def launch_sequence(self, program, dialog):
        from tkinter import messagebox
        import parking_sequence_test as sequence
        if self.process is not None and self.process.poll() is None:
            messagebox.showerror('실행 중', '기존 테스트를 종료한 뒤 실행하세요.', parent=dialog.window)
            return
        try:
            summary = sequence.validate_sequence(program)
            if self.ros is not None and self.ros.motion_source_running():
                raise ValueError('다른 주차/모터 송신 노드를 먼저 종료하세요.')
            settings = self.current_settings()
            actual = bool(settings['enable_actuation']) and not self.preview
            port = settings.get('arduino_port', '').strip()
            if actual and (not port or not os.path.exists(port)):
                raise ValueError('연결 설정에 존재하는 Arduino 포트를 지정하세요.')
            if actual and not messagebox.askyesno(
                    '주차 동작만 실차 시험',
                    f"PWM ±50, {summary['frames']}구간, {summary['duration_s']:.2f}초를 실행합니다.\n"
                    '시간은 미측정 가정입니다. 인식·SLAM·장애물 감지·위치 보정 없이 움직입니다.\n'
                    '주변 공간과 즉시 전원을 끌 수 있는 상태를 확인했나요?', parent=dialog.window):
                return
            with tempfile.NamedTemporaryFile(mode='w', suffix='.json', prefix='parking-sequence-',
                                             encoding='utf-8', delete=False) as stream:
                json.dump(program, stream, ensure_ascii=False, allow_nan=False)
                self.sequence_file = Path(stream.name)
            command = [sys.executable, '-u', str(Path(__file__).with_name('parking_sequence_test.py')),
                       '--program', str(self.sequence_file)]
            command += ['--execute', '--ack-unmeasured-test', '--port', port] if actual else ['--dry-run']
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            text=True, bufsize=1, errors='replace', preexec_fn=child_setup)
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            if self.sequence_file is not None:
                self.sequence_file.unlink(missing_ok=True)
                self.sequence_file = None
            messagebox.showerror('시퀀스 실행 오류', str(exc), parent=dialog.window)
            return
        self.process_kind = 'sequence'
        self.running_settings = dict(settings)
        self.set_running(True)
        threading.Thread(target=self.reader, args=(self.process,), daemon=True).start()
        label = '실차 구동' if actual else '명령 미리보기 · 모터 OFF'
        self.process_label.configure(text='주차 동작만 · '+label)
        self.fields['stage'].configure(text='SEQUENCE_TEST · '+label)
        self.fields['recognition'].configure(text='미사용 · 인식/SLAM 없이 시간표 재생', foreground='')
        self.fields['failure'].configure(text='실차 주차 성공 여부는 자동 판정하지 않음')
        self.append(f'[주차 동작] {label} · {summary["duration_s"]:.2f}초 · {self.sequence_file}')

    def stop(self):
        if self.process_kind == 'sequence':
            self.terminate()
        else:
            self.service('stop')

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
        if self.preview:
            return
        if getattr(self, 'process_kind', 'stack') == 'sequence':
            if name == 'stop':
                self.terminate()
            return
        if name in ('start_search', 'start_parking'):
            expected, _ = core.primary_action(self.running_mode())
            if name != expected:
                self.fields['response'].config(text='현재 탐색 방식과 맞지 않는 시작 명령입니다.')
                return
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
                if line.startswith('[주차 동작]'):
                    self.fields['response'].configure(text=line)
                    if self.sequence_dialog is not None:
                        self.sequence_dialog.progress(line)
                if self.vars['full_log'].get() or line.startswith('[주차 동작]') or any(key in line for key in LOG_KEYS):
                    self.append(line)
            if self.process is not None and self.process.poll() is not None:
                self.process_label.config(text=f'종료됨 (코드 {self.process.returncode})')
                if self.sequence_file is not None:
                    self.sequence_file.unlink(missing_ok=True)
                    self.sequence_file = None
                self.set_running(False)
            if self.ros is not None:
                self.ros.spin()
                if self.process_kind != 'sequence':
                    self.update_status()
                if self.process_kind != 'sequence' and time.monotonic()-self.last_render > .5:
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
                self.fields['bay'].config(text=f'간격 {observation["length"]:.2f} m · 목표 영역 폭 {observation["width"]:.2f} m, '
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
        if self.sequence_file is not None:
            self.sequence_file.unlink(missing_ok=True)
        self.root.destroy()


def main():
    import argparse
    import tkinter as tk
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', action='store_true', help='Open layout only, without ROS or hardware control')
    args = parser.parse_args()
    root = tk.Tk()
    TestWindow(root, preview=args.preview)
    root.mainloop()


if __name__ == '__main__':
    main()
