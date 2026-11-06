#!/usr/bin/env python3
"""G6 hardware diagnostic only: checks packets; never sends vehicle commands.

Uses YDLIDAR's G6 command and packet format:
https://github.com/YDLIDAR/YDLidar-SDK/blob/master/core/common/ydlidar_protocol.h
https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/master/details.md

115200 baud was verified on this connected unit. The manufacturer's standard
G6 setting is 512000; use --baud for a different unit. This tool does not publish
ROS scans or provide calibrated angle measurements for vehicle control.
"""
import argparse
from contextlib import suppress
import signal
import struct
import time
import serial

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    s = serial.Serial(port=None, baudrate=args.baud, timeout=.2,
                      write_timeout=1, exclusive=True)
    s.port = args.port
    s.dtr = False  # G6: support_motor_dtr=false in the manufacturer's table.
    s.open()
    try:
        def command(value):
            s.write(bytes((0xa5, value)))
            s.flush()
        command(0x65)
        time.sleep(.15)
        s.reset_input_buffer()
        command(0x90)
        reply = s.read(27)
        if len(reply) != 27 or reply[:7] != bytes.fromhex('a55a1400000004') or reply[7] != 13:
            raise RuntimeError('G6 identification failed: ' + reply.hex())
        print(f'G6 CONFIRMED port={args.port} baud={args.baud} firmware={reply[9]}.{reply[8]} hardware={reply[10]}', flush=True)
        command(0x92)
        health = s.read(10)
        if len(health) != 10 or health[:7] != bytes.fromhex('a55a0300000006'):
            raise RuntimeError('Health response invalid: ' + health.hex())
        status, error = struct.unpack('<BH', health[7:])
        print(f'HEALTH status={status} error={error}', flush=True)
        if status not in (0, 1, 2):
            raise RuntimeError(f'Undocumented health status {status}; do not start scanning')
        if status == 2:
            raise RuntimeError('Sensor reports a hardware error')
        command(0x60)
        descriptor = s.read(7)
        if descriptor != bytes.fromhex('a55a0500004081'):
            raise RuntimeError('Scan descriptor invalid: ' + descriptor.hex())
        print('SCANNING (Ctrl+C stops the sensor)', flush=True)
        buf = bytearray()
        total = bad = wraps = valid = samples = nbytes = 0
        minimum = float('inf')
        prev_angle = None
        last_data = report = time.monotonic()
        while running:
            data = s.read(max(1, min(s.in_waiting, 8192)))
            now = time.monotonic()
            if data:
                buf.extend(data)
                nbytes += len(data)
                last_data = now
            elif now - last_data > 5:
                raise RuntimeError('No scan data for five seconds')
            while len(buf) >= 10:
                start = buf.find(b'\xaa\x55')
                if start < 0:
                    del buf[:-1]
                    break
                if start:
                    del buf[:start]
                if len(buf) < 10:
                    break
                ct, n, first, last, expected = struct.unpack_from('<BBHHH', buf, 2)
                if n == 0 or not first & 1 or not last & 1:
                    del buf[:2]
                    continue
                length = 10 + n * 2
                if len(buf) < length:
                    break
                values = struct.unpack_from('<' + 'H' * n, buf, 10)
                checksum = 0x55aa ^ (ct | n << 8) ^ first ^ last
                for value in values:
                    checksum ^= value
                if checksum != expected:
                    bad += 1
                    del buf[:2]
                    continue
                total += 1
                samples += n
                angle = (first >> 1) / 64.
                if prev_angle is not None and prev_angle - angle > 180:
                    wraps += 1
                prev_angle = angle
                for value in values:
                    if value:
                        valid += 1
                        minimum = min(minimum, value / 4000.)
                del buf[:length]
            if now - report >= 1:
                nearest = f'{minimum:.3f}' if valid else 'none'
                print(f'PACKETS={total} CHECKSUM_ERRORS={bad} BYTES={nbytes} SAMPLES={samples} VALID={valid} ANGLE_WRAPS={wraps} NEAREST_M={nearest}', flush=True)
                report = now
    finally:
        if s.is_open:
            try:
                # Unplugging can make both stop and DTR operations fail.
                # Always close the descriptor, without masking the first error.
                with suppress(OSError, serial.SerialException):
                    s.write(bytes((0xa5, 0x65)))
                    s.flush()
            finally:
                with suppress(OSError, serial.SerialException):
                    s.dtr = False
                s.close()
        print('STOPPED', flush=True)

if __name__ == '__main__':
    main()
