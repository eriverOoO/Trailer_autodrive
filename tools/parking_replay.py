#!/usr/bin/env python3
"""Capture-independent CPU planning and ride-on command preview/replay."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
for package in ('trailer_parking_pkg', 'serial_communication_pkg'):
    sys.path.insert(0, str(ROOT / 'src' / package))

from trailer_parking_pkg.replay import build_program, save_json, transmit, validate_program


def read(path):
    return json.loads(Path(path).expanduser().read_text(encoding='utf-8'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    build = sub.add_parser('plan', help='Compile a recognized-space snapshot; no hardware I/O')
    build.add_argument('--snapshot', required=True)
    build.add_argument('--profile', required=True)
    build.add_argument('--output', required=True)
    build.add_argument('--speed', type=float, default=0.12)
    build.add_argument('--timeout', type=float, default=30)
    for name in ('preview', 'replay'):
        action = sub.add_parser(name)
        action.add_argument('program')
        if name == 'replay':
            action.add_argument('--execute', action='store_true', help='Actually open the Arduino port and transmit')
            action.add_argument('--port', default='/dev/ttyACM0')
    args = parser.parse_args(argv)
    if args.action == 'plan':
        if not 0 < args.timeout <= 300:
            raise ValueError('Planning timeout must be between 0 and 300 seconds')
        program = build_program(read(args.snapshot), read(args.profile), args.speed, args.timeout)
        save_json(args.output, program)
        print('Saved:', args.output)
    else:
        program = read(args.program)
    execute = args.action == 'replay' and args.execute
    summary = validate_program(program, hardware=execute)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    if not execute:
        print('PREVIEW ONLY: no serial port opened; no command sent.')
        for frame in program['frames'][:12]:
            print(f"{frame['ticks'] * program['dt_s']:.2f}s "
                  f"s{frame['steering']}l{frame['left_pwm']}r{frame['right_pwm']}")
        if len(program['frames']) > 12:
            print(f"... {len(program['frames'])} segments total")
        return 0
    # Imports and port opening are deliberately unreachable in preview mode.
    import serial
    from serial_communication_pkg.protocol import encode_command
    with serial.Serial(args.port, 115200, timeout=1, write_timeout=.1, exclusive=True) as port:
        try:
            # Same board boot delay as the existing serial_sender_node.
            time.sleep(4.5)
            transmit(program, port, encode_command, time.monotonic, time.sleep)
        finally:
            port.write(encode_command(0, 0, 0))
            port.flush()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, TypeError, OSError, RuntimeError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)
    except KeyboardInterrupt:
        print('Interrupted; stop requested.', file=sys.stderr)
        raise SystemExit(130)
