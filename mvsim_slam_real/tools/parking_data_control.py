#!/usr/bin/env python3
"""ros2_parking data_collection controls, with camera capture removed.

Reference: ros2_parking/src/data_collection/data_collection.py and the adjacent
CPython 3.10 Data_Collect.process bytecode. Key priority and accumulated values
are retained. No ROS gate, velocity conversion, camera import, or image output.
"""
import argparse
import ctypes
import ctypes.util
import signal
import subprocess
import time
import tkinter as tk


class DataControls:
    def __init__(self, now=None):
        self.steering = self.left_speed = self.right_speed = 0
        self.exit = False
        self.max_steering = 7
        self.keyboard_sensing_period = .1
        self.last_input_time = time.time() if now is None else now

    def process(self, keys, now=None):
        now = time.time() if now is None else now
        if now - self.last_input_time < self.keyboard_sensing_period:
            return {'exit': self.exit}
        # The original is an if/elif chain, not simultaneous independent axes.
        if 'w' in keys:
            self.left_speed = min(self.left_speed + 10, 250)
            self.right_speed = min(self.right_speed + 10, 250)
        elif 's' in keys:
            self.left_speed = max(self.left_speed - 10, -250)
            self.right_speed = max(self.right_speed - 10, -250)
        elif 'a' in keys:
            self.steering = max(self.steering - 1, -self.max_steering)
        elif 'd' in keys:
            self.steering = min(self.steering + 1, self.max_steering)
        elif 'r' in keys:
            self.reset()
        elif 'c' in keys:
            pass  # Camera connection and image collection deliberately removed.
        elif 'f' in keys:
            self.reset()
            self.exit = True
        self.last_input_time = now
        return {'exit': self.exit}

    def reset(self):
        self.steering = self.left_speed = self.right_speed = 0

    def packet(self):
        return f's{self.steering}l{self.left_speed}r{self.right_speed}\n'.encode('ascii')


class X11Keyboard:
    """Poll held keys globally without the root access required by keyboard."""
    def __init__(self):
        self.lib = ctypes.CDLL(ctypes.util.find_library('X11') or 'libX11.so.6')
        self.lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.lib.XOpenDisplay.restype = ctypes.c_void_p
        self.lib.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.lib.XStringToKeysym.restype = ctypes.c_ulong
        self.lib.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.lib.XKeysymToKeycode.restype = ctypes.c_ubyte
        self.lib.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte)]
        self.lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.display = self.lib.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('Cannot open X11 display for keyboard input')
        self.keycodes = {key: self.lib.XKeysymToKeycode(self.display, self.lib.XStringToKeysym(key.encode()))
                         for key in 'wasdrcf'}

    def pressed(self):
        state = (ctypes.c_ubyte * 32)()
        self.lib.XQueryKeymap(self.display, state)
        return {key for key, code in self.keycodes.items()
                if code and state[code // 8] & (1 << (code % 8))}

    def close(self):
        if self.display:
            self.lib.XCloseDisplay(self.display)
            self.display = None


class ParkingControlWindow:
    def __init__(self, root, port=None, dry_run=False, keyboard=None, connection=None):
        self.root, self.dry_run = root, dry_run
        self.controls = DataControls()
        self.keyboard = keyboard if keyboard is not None else X11Keyboard()
        self.serial = connection
        self.closed = False
        self.last_packet = None
        root.title('ros2_parking 수동조작 — 카메라 없음')
        root.geometry('780x410')
        tk.Label(root, text='W/S: PWM +10 / -10    A/D: 조향 -1 / +1', font=('', 19)).pack(pady=(24, 12))
        tk.Label(root, text='키를 떼어도 속도·조향이 유지됩니다.\nR: 속도 0 · 조향 중앙    F: 정지 후 전체 종료', font=('', 17), fg='#a12c20').pack(pady=10)
        tk.Label(root, text='원본과 같은 우선순위 W > S > A > D > R > C > F\n0.1초마다 증감 · PWM ±250 · 조향 ±7 · C 촬영 기능 없음', font=('', 13)).pack(pady=8)
        self.status = tk.StringVar(value='Arduino 연결 중')
        tk.Label(root, textvariable=self.status, font=('', 18)).pack(pady=12)
        tk.Button(root, text='R — 속도·조향 초기화', command=self.reset, height=2).pack(pady=5)
        tk.Label(root, text='키 입력은 창 포커스와 무관하게 감지됩니다. 종료는 F 또는 창 닫기.', font=('', 12)).pack(pady=8)
        root.protocol('WM_DELETE_WINDOW', self.close)
        try:
            if not dry_run and self.serial is None:
                import serial
                owner = subprocess.run(['fuser', port], capture_output=True, text=True)
                if owner.returncode != 1:
                    raise RuntimeError('Arduino port busy or ownership check failed')
                self.serial = serial.Serial(port, 115200, timeout=1, write_timeout=.2, exclusive=True)
                time.sleep(1.)  # Same board reset delay as data_collection.py.
            self.send()
        except BaseException:
            self.keyboard.close()
            if self.serial is not None:
                self.serial.close()
            raise
        root.after(10, self.tick)

    def send(self):
        data = self.controls.packet()
        if self.serial is not None:
            if self.serial.write(data) != len(data):
                raise OSError('Incomplete serial write')
        self.status.set(('미리보기: ' if self.dry_run else '송신: ') + data.decode().strip())
        if data != self.last_packet:
            print('Sent:', data.decode().strip(), flush=True)
            self.last_packet = data

    def reset(self):
        self.controls.reset()
        self.send()

    def tick(self):
        if self.closed:
            return
        try:
            if self.controls.process(self.keyboard.pressed())['exit']:
                self.close()
                return
            # Send the retained values directly, including steering at PWM 0.
            self.send()
        except Exception as error:
            print(f'Control error: {error}', flush=True)
            self.close()
            return
        self.root.after(10, self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.controls.reset()
        try:
            for _ in range(3):
                try:
                    self.send()
                except Exception:
                    break
                if self.serial is not None:
                    time.sleep(.05)
        finally:
            if self.serial is not None:
                self.serial.close()
            self.keyboard.close()
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Arduino device; required except in --dry-run')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if not args.dry_run and not args.port:
        parser.error('--port required')
    root = tk.Tk()
    app = None
    try:
        app = ParkingControlWindow(root, args.port, args.dry_run)
        signal.signal(signal.SIGINT, lambda *_: app.close())
        signal.signal(signal.SIGTERM, lambda *_: app.close())
        root.mainloop()
    finally:
        if app is not None:
            app.close()
        else:
            root.destroy()


if __name__ == '__main__':
    main()
