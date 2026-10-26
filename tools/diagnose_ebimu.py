#!/usr/bin/env python3
"""Bounded raw EBIMU UART capture; passive by default.

EBIMU-9DOFV6 rev10 pp5,8,12-14,34: 8N1, no flow control;
<ver>, <cfg> then extra >, and polling byte * are the ONLY TX options.
No configuration, reset, start/stop, or calibration commands are exposed.
Opening a pyserial port necessarily changes host TTY/modem state and flushes
the input once inside pyserial.open(); no later input flush is performed.
"""
import argparse
import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import struct
import sys
import termios
import time

import serial

BAUDS = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)


def seconds(value):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= 60:
        raise argparse.ArgumentTypeError('must be finite, >0 and <=60 seconds')
    return value


def tty_settings(fd):
    a = termios.tcgetattr(fd)
    flags = {}
    for index, names in [(0, ('IGNBRK', 'BRKINT', 'PARMRK', 'ISTRIP', 'INLCR',
                              'IGNCR', 'ICRNL', 'IXON', 'IXOFF', 'IXANY')),
                         (1, ('OPOST',)),
                         (2, ('CREAD', 'CLOCAL', 'PARENB', 'CSTOPB', 'CRTSCTS', 'HUPCL')),
                         (3, ('ICANON', 'ECHO', 'ISIG', 'IEXTEN'))]:
        for name in names:
            flags[name] = bool(a[index] & getattr(termios, name, 0))
    result = {'flags': flags, 'CS8': a[2] & termios.CSIZE == termios.CS8,
              'ispeed_constant': a[4], 'ospeed_constant': a[5],
              'VMIN': a[6][termios.VMIN], 'VTIME': a[6][termios.VTIME]}
    try:
        bits = struct.unpack('I', fcntl.ioctl(fd, termios.TIOCMGET, struct.pack('I', 0)))[0]
        result['modem'] = {name: bool(bits & getattr(termios, 'TIOCM_' + name))
                           for name in ('DTR', 'RTS', 'CTS', 'DSR', 'CAR')}
    except OSError as exc:
        result['modem_read_error'] = str(exc)
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--baud', type=int, choices=BAUDS, default=115200)
    p.add_argument('--duration', type=seconds, default=2.0, help='seconds per capture phase')
    p.add_argument('--init-wait', type=seconds, default=3.0,
                   help='capture during this host settling interval; NOT a documented sensor boot time')
    p.add_argument('--dtr', choices=('low', 'high'), default='low')
    p.add_argument('--rts', choices=('low', 'high'), default='low')
    p.add_argument('--query', action='append', choices=('version', 'configuration', 'poll'), default=[],
                   help='explicit read-only requests, in given order; default is no TX')
    p.add_argument('--finish-config', action='store_true',
                   help='send documented extra > to finish a previous interrupted cfg query')
    p.add_argument('--log-prefix', type=Path, help='new .jsonl and .rx.bin files; refuses overwrite')
    args = p.parse_args(argv)
    events = raw = ser = None
    rx_count = tx_count = 0
    started = time.monotonic()
    cfg_pending = False
    status = 'ERROR'
    exit_code = 1

    def emit(event, **fields):
        record = {'at': datetime.datetime.now().astimezone().isoformat(),
                  'elapsed_s': round(time.monotonic() - started, 6), 'event': event, **fields}
        line = json.dumps(record, ensure_ascii=False)
        print(line, flush=True)
        if events:
            events.write(line + '\n')
            events.flush()

    def send(data, phase):
        nonlocal tx_count
        count = ser.write(data)
        tx_count += count
        emit('tx', phase=phase, bytes=count, ascii=repr(data), hex=data[:count].hex(' '))
        if count != len(data):
            raise serial.SerialTimeoutException('incomplete command write')

    def capture(duration, phase):
        nonlocal rx_count
        before = rx_count
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            ser.timeout = min(0.05, max(0.0, deadline - time.monotonic()))
            data = ser.read(min(4096, max(1, ser.in_waiting)))
            if data:
                rx_count += len(data)
                if raw:
                    raw.write(data)
                    raw.flush()
                emit('rx', phase=phase, bytes=len(data), total_rx_bytes=rx_count,
                     ascii=repr(data), hex=data.hex(' '))
        emit('phase_end', phase=phase, rx_bytes=rx_count - before, total_rx_bytes=rx_count)

    try:
        if args.log_prefix:
            events = Path(str(args.log_prefix) + '.jsonl').open('x', encoding='utf-8')
            raw = Path(str(args.log_prefix) + '.rx.bin').open('xb')
        ser = serial.Serial(port=None, baudrate=args.baud, bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                            timeout=0.05, inter_byte_timeout=None, write_timeout=1.0,
                            xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
        ser.dtr = args.dtr == 'high'
        ser.rts = args.rts == 'high'
        ser.port = args.port
        emit('opening', port=args.port, realpath=os.path.realpath(args.port),
             pyserial=serial.__version__, settings=ser.get_settings(), dtr=ser.dtr, rts=ser.rts,
             note='pyserial.open internally flushes input once; no post-open flush; driver may glitch modem lines')
        ser.open()
        emit('opened', tty=tty_settings(ser.fileno()), settings=ser.get_settings(),
             dtr=ser.dtr, rts=ser.rts)
        # Record, rather than discard, bytes received during settling.
        capture(args.init_wait, 'initialization_capture')
        capture(args.duration, 'passive')
        if args.finish_config:
            send(b'>', 'finish_previous_configuration')
            capture(args.duration, 'after_finish_previous_configuration')
        for query in args.query:
            if query == 'configuration':
                cfg_pending = True
                send(b'<cfg>', query)
                try:
                    capture(args.duration, query)
                finally:
                    send(b'>', 'finish_configuration')
                    cfg_pending = False
                capture(args.duration, 'after_configuration')
            else:
                send(b'<ver>' if query == 'version' else b'*', query)
                capture(args.duration, query)
        status = 'RX_OBSERVED' if rx_count else 'UNRESOLVED_ZERO_RX'
        exit_code = 0 if rx_count else 2
    except KeyboardInterrupt:
        status = 'INTERRUPTED'
        exit_code = 130
        emit('interrupted')
    except (OSError, ValueError, serial.SerialException) as exc:
        emit('error', exception=type(exc).__name__, message=str(exc))
    finally:
        if ser is not None and ser.is_open:
            if cfg_pending:
                try:
                    send(b'>', 'finish_configuration_on_error')
                except (OSError, serial.SerialException) as exc:
                    emit('cleanup_error', message=str(exc))
            try:
                emit('closing', tty=tty_settings(ser.fileno()))
            except OSError as exc:
                emit('closing_state_error', message=str(exc))
            finally:
                ser.close()
                emit('closed')
        emit('summary', status=status, rx_bytes=rx_count, tx_bytes=tx_count, exit_code=exit_code)
        if raw:
            raw.close()
        if events:
            events.close()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
