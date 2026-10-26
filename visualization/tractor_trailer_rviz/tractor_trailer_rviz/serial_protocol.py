"""Independent EBIMU ASCII parser. Rev10 pp12,21-24; no serial writes."""
import argparse
import math
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def euler_quaternion(r, p, y):
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def quaternion_euler(q):
    x, y, z, w = q
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def rotate(q, v):
    x, y, z, w = q
    a, b, c = v
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return (a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx)


class Parser:
    def __init__(self, orientation='quaternion', gyro=True, accel=True, timestamp=True):
        self.orientation, self.gyro, self.accel, self.timestamp = orientation, gyro, accel, timestamp
        self.buffer = bytearray()
        self.accepted = self.rejected = self.discarded = 0
        self.lengths = {}

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while self.buffer:
            start = self.buffer.find(b'*')
            if start < 0:
                self.discarded += len(self.buffer)
                self.buffer.clear()
                break
            self.discarded += start
            del self.buffer[:start]
            end = self.buffer.find(b'\n')
            restart = self.buffer.find(b'*', 1)
            if restart >= 0 and (end < 0 or restart < end):
                self.rejected += 1
                del self.buffer[:restart]
                continue
            if end < 0:
                if len(self.buffer) > 512:
                    self.rejected += 1
                    self.buffer.clear()
                break
            frame = bytes(self.buffer[:end+1])
            del self.buffer[:end+1]
            try:
                if not frame.endswith(b'\r\n') or len(frame) > 512:
                    raise ValueError('framing')
                fields = frame[1:-2].decode('ascii').split(',')
                n = 4 if self.orientation == 'quaternion' else 3
                if len(fields) != n+3*self.gyro+3*self.accel+self.timestamp:
                    raise ValueError('channel count')
                values = [float(v) for v in fields]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('nonfinite')
                if n == 4:
                    z, y, x, w = values[:4]
                    norm = math.sqrt(x*x+y*y+z*z+w*w)
                    if abs(norm-1.) > .05:
                        raise ValueError('quaternion norm')
                    q = tuple(v/norm for v in (x, y, z, w))
                else:
                    r, p, y = values[:3]
                    if abs(r) > 180 or abs(p) > 90 or abs(y) > 180:
                        raise ValueError('Euler range')
                    q = euler_quaternion(*map(math.radians, (r, p, y)))
                    norm = 1.
                g = tuple(math.radians(v) for v in values[n:n+3]) if self.gyro else None
                n += 3*self.gyro
                a = tuple(v*9.80665 for v in values[n:n+3]) if self.accel else None
                t = values[-1] if self.timestamp else None
                if t is not None and (t < 0 or not t.is_integer()):
                    raise ValueError('timestamp')
                out.append(dict(q=q, gyro=g, accel=a, sensor_ms=t, raw_norm=norm,
                                rpy_deg=tuple(map(math.degrees, quaternion_euler(q)))))
                self.accepted += 1
                self.lengths[len(frame)] = self.lengths.get(len(frame), 0)+1
            except (ValueError, UnicodeError):
                self.rejected += 1
        return out


def serial_arguments(p):
    p.add_argument('--port', required=True, help='explicit physically confirmed IMU port')
    p.add_argument('--confirmed-imu-port', action='store_true', required=True)
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--orientation', choices=['quaternion', 'euler'], default='quaternion')
    p.add_argument('--gyro', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--accel', action=argparse.BooleanOptionalAction, default=True,
                   help='only soa1 (body acceleration including gravity) is supported')
    p.add_argument('--timestamp', action=argparse.BooleanOptionalAction, default=True)


def make_parser(args):
    return Parser(args.orientation, args.gyro, args.accel, args.timestamp)


def open_serial(args):
    import serial
    real = os.path.realpath(args.port)
    if not real.startswith('/dev/ttyUSB'):
        raise ValueError('only explicit CP2102 /dev/ttyUSB ports supported')
    r = subprocess.run(['udevadm', 'info', '--query=property', '--name='+real],
                       capture_output=True, text=True, check=True)
    if 'ID_VENDOR_ID=10c4\n' not in r.stdout or 'ID_MODEL_ID=ea60\n' not in r.stdout:
        raise ValueError('port is not expected CP210x VID/PID')
    busy = subprocess.run(['fuser', real], capture_output=True, text=True)
    if busy.returncode != 1:
        raise RuntimeError('port occupied or occupancy check failed: '+busy.stdout+busy.stderr)
    s = serial.Serial(port=None, baudrate=args.baud, timeout=0,
                      xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
    s.dtr = False
    s.rts = False
    s.port = args.port
    s.open()
    # Linux exclusive access also blocks non-cooperating future opens.
    import fcntl
    import termios
    fcntl.ioctl(s.fileno(), termios.TIOCEXCL)
    return s
