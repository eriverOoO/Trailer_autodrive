#!/usr/bin/env python3
"""Camera-free WASD control, standalone or via the SLAM manual PWM bridge."""
import argparse
import json
import signal
import subprocess
import time
import tkinter as tk


def command(keys, pwm, steering, steering_sign=-1, left_sign=1, right_sign=1):
    """Opposing keys cancel; both motors receive the same signed drive request."""
    speed = (int('w' in keys) - int('s' in keys)) * pwm
    angle = (int('a' in keys) - int('d' in keys)) * steering * steering_sign
    return angle, speed * left_sign, speed * right_sign


def packet(angle, left, right):
    if not -7 <= angle <= 7 or not all(-255 <= v <= 255 for v in (left, right)):
        raise ValueError('Command outside Arduino range')
    return f's{angle}l{left}r{right}\n'.encode('ascii')


class DriveWindow:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.serial = None
        self.ros = None
        self.motor_ready = False
        self.motor_ready_wall = -float('inf')
        self.motor_status = 'SLAM 준비 대기'
        self.motor_status_wall = -float('inf')
        self.keys, self.blocked, self.releases = set(), set(), {}
        self.angle = 0
        self.closed = False
        self.fault = False
        self.active = False
        self.ready_at = time.monotonic() + (0 if args.dry_run or getattr(args, 'ros', False) else 2.)
        root.title('WASD 수동 주행 테스트')
        root.geometry('660x380')
        self.status = tk.StringVar(value='연결 준비 중 — 키를 누르지 마세요')
        tk.Label(root, text='W 전진   S 후진   A 좌조향   D 우조향', font=('', 19)).pack(pady=(25, 12))
        tk.Label(root, text='W+A / W+D 동시 입력 가능 · 키를 떼면 정지\nSpace 정지 · Esc 종료 · 창 밖으로 포커스가 나가면 정지',
                 font=('', 13)).pack(pady=8)
        connection = ('SLAM /parking/manual_pwm' if getattr(args, 'ros', False) else
                      '미리보기 (장치 접속 없음)' if args.dry_run else args.port)
        tk.Label(root, text=f"{connection}  |  PWM {args.pwm}  |  조향 {args.steer}/7",
                 font=('', 12)).pack(pady=8)
        tk.Label(root, textvariable=self.status, font=('', 15), fg='#155b37').pack(pady=10)
        tk.Button(root, text='정지 (Space)', command=self.stop, width=25, height=2).pack(pady=5)
        tk.Label(root, text='USB 단절·프로그램 강제 종료 시 기존 보드에는 자동 정지 기능이 없습니다.',
                 fg='#8a3b20').pack(pady=8)
        root.bind('<KeyPress>', self.press)
        root.bind('<KeyRelease>', self.release)
        root.bind('<FocusOut>', self.focus_out)
        root.bind('<FocusIn>', lambda event: setattr(self, 'active', True))
        root.protocol('WM_DELETE_WINDOW', self.close)
        if getattr(args, 'ros', False):
            import rclpy
            from interfaces_pkg.msg import MotionCommand
            from std_msgs.msg import Bool, String
            rclpy.init()
            self.ros = rclpy.create_node('wasd_manual_control')
            self.motion_type = MotionCommand
            self.publisher = self.ros.create_publisher(MotionCommand, '/parking/manual_pwm', 1)
            self.ros.create_subscription(Bool, '/vehicle_viz_2d/ready', self.on_ready, 1)
            self.ros.create_subscription(String, '/parking/manual_motor_status', self.on_motor_status, 1)
        elif not args.dry_run:
            import serial
            owner = subprocess.run(['fuser', args.port], capture_output=True, text=True)
            if owner.returncode != 1:
                raise RuntimeError('Arduino port busy or ownership check failed: ' + owner.stdout + owner.stderr)
            self.serial = serial.Serial(args.port, 115200, timeout=0, write_timeout=.1, exclusive=True)
        try:
            self.send(0, 0, 0)
        except Exception:
            if self.serial is not None:
                self.serial.close()
            raise
        root.after(50, self.tick)
        root.after(200, root.focus_force)

    def send(self, angle, left, right):
        data = packet(angle, left, right)
        if self.ros is not None:
            msg = self.motion_type()
            msg.steering, msg.left_speed, msg.right_speed = angle, left, right
            self.publisher.publish(msg)
        if self.serial is not None:
            written = self.serial.write(data)
            if written != len(data):
                raise OSError('Incomplete Arduino serial write')
        self.angle = angle
        return data.decode().strip()

    def on_ready(self, msg):
        self.motor_ready, self.motor_ready_wall = msg.data, time.monotonic()

    def on_motor_status(self, msg):
        try:
            value = json.loads(msg.data)
            self.motor_status = f"실제 출력 PWM {value['pwm']} / 조향 {value['steering']} · {value['reason'] or 'READY'}"
            self.motor_status_wall = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.motor_status = '모터 상태 형식 오류'

    def slam_ready(self):
        now = time.monotonic()
        return (self.motor_ready and now - self.motor_ready_wall < .3
                and now - self.motor_status_wall < .5)

    def press(self, event):
        key = event.keysym.lower()
        if key == 'escape':
            self.close();return
        if key == 'space':
            self.stop();return
        if key not in 'wasd' or len(key) != 1:
            return
        if key in self.releases:
            self.root.after_cancel(self.releases.pop(key))
        if time.monotonic() < self.ready_at or self.fault or not self.active:
            self.blocked.add(key)
            return
        if getattr(self, 'ros', None) is not None and not self.slam_ready():
            self.blocked.add(key)
            return
        if key not in self.blocked:
            self.keys.add(key)

    def release(self, event):
        key = event.keysym.lower()
        if key not in 'wasd' or len(key) != 1:
            return
        if key in self.releases:
            self.root.after_cancel(self.releases.pop(key))
        # X11 emits release/press pairs during autorepeat; cancel on repeat press.
        self.releases[key] = self.root.after(20, lambda: self.released(key))

    def released(self, key):
        self.releases.pop(key, None)
        self.keys.discard(key)
        self.blocked.discard(key)

    def focus_out(self, event):
        if self.closed:
            return
        self.active = False
        self.stop()

    def stop(self):
        self.blocked.update(self.keys)
        self.keys.clear()
        try:
            self.send(self.angle, 0, 0)
        except Exception as error:
            self.fault = True
            self.status.set(f'통신 오류: {error}')

    def tick(self):
        if self.closed:
            return
        if self.ros is not None:
            import rclpy
            for _ in range(10):
                rclpy.spin_once(self.ros, timeout_sec=0)
        if not self.fault:
            try:
                if self.ros is not None and not self.slam_ready():
                    self.stop()
                    self.status.set('WAIT / HOLD STILL — 두 차체를 일자로 정렬하고 정지하세요')
                elif time.monotonic() < self.ready_at:
                    self.send(self.angle, 0, 0)
                    self.status.set('보드 시작 대기 중 — 키에서 손을 떼세요')
                else:
                    values = command(self.keys if self.active else set(), self.args.pwm, self.args.steer,
                                     self.args.steering_sign, self.args.left_sign, self.args.right_sign)
                    # Stop holds the last angle; A/D can explicitly steer while stationary.
                    if not self.keys or not self.active:
                        values = (self.angle, 0, 0)
                    text = self.send(*values)
                    self.status.set(self.motor_status if self.ros is not None else
                                    f"{'미리보기' if self.args.dry_run else '명령'}: {text}")
            except Exception as error:
                self.fault = True
                self.stop()
                self.status.set(f'통신 오류 — 전원 차단 확인: {error}')
        self.root.after(50, self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.keys.clear()
        try:
            for _ in range(3):
                try:
                    self.send(self.angle, 0, 0)
                except Exception:
                    break
                if self.serial is not None:
                    time.sleep(.05)
        finally:
            if self.serial is not None:
                self.serial.close()
            if self.ros is not None:
                import rclpy
                self.ros.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            self.root.destroy()


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Explicit Arduino port, e.g. /dev/ttyACM0')
    parser.add_argument('--pwm', type=int, default=60, help='Motor PWM, 1..255 (default 60)')
    parser.add_argument('--steer', type=int, default=3, help='Steering steps, 1..7 (default 3)')
    parser.add_argument('--steering-sign', type=int, choices=(-1, 1), default=-1)
    parser.add_argument('--left-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--right-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--dry-run', action='store_true', help='Show UI and commands without opening any device')
    parser.add_argument('--ros', action='store_true', help='Use SLAM manual PWM bridge; never opens serial here')
    args = parser.parse_args()
    if args.ros and (args.port or args.dry_run):
        parser.error('--ros uses a separate bridge; omit --port and --dry-run')
    if not args.ros and not args.dry_run and not args.port:
        parser.error('--port is required unless --dry-run')
    if not 1 <= args.pwm <= 255 or not 1 <= args.steer <= 7:
        parser.error('--pwm must be 1..255 and --steer must be 1..7')
    return args


def main():
    args = arguments()
    root = tk.Tk()
    app = None
    try:
        app = DriveWindow(root, args)
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
