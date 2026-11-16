#!/usr/bin/env python3
"""Standalone, fixed-50-PWM actuator test. No recognition, planning or ROS.

Timing is nominal and unmeasured. This is deliberately separate from the
calibrated replay compiler: it makes no collision or parking-success claim.
Without --execute, neither serial nor a device port is accessed.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time

DT = .05
PWM = 50
SOURCE = Path(__file__).with_name('parking_sequence_source.json')


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def build_sequence(forward=None, reverse=None):
    """Preserve nominal travel distances, NOT PWM*time or measured motion."""
    source = json.loads(SOURCE.read_text())
    forward = positive(source['nominal_forward_at_50_mps'] if forward is None else forward, 'forward speed')
    reverse = positive(source['nominal_reverse_at_50_mps'] if reverse is None else reverse, 'reverse speed')
    frames, residual, previous_direction = [], 0., 0
    for frame in source['frames']:
        moving = frame['left_pwm'] != 0
        count = frame['ticks']
        direction = (1 if frame['left_pwm'] > 0 else -1) if moving else 0
        if moving:
            if direction != previous_direction:
                residual = 0.
            old_speed = source['reference_forward_mps'] if direction > 0 else source['reference_reverse_mps']
            new_speed = forward if direction > 0 else reverse
            exact = count * old_speed / new_speed + residual
            count = max(1, round(exact))
            residual = exact - count
            previous_direction = direction
        frames.append(dict(ticks=count, steering=frame['steering'],
                           left_pwm=direction*PWM, right_pwm=direction*PWM))
    program = dict(schema='parking_motor_test_v1', dt_s=DT, timing_status='nominal_unmeasured',
                   nominal_forward_mps=forward, nominal_reverse_mps=reverse,
                   source=source['source'], frames=frames)
    validate_sequence(program)
    return program


def validate_sequence(program):
    if (program.get('schema') != 'parking_motor_test_v1' or program.get('dt_s') != DT
            or program.get('timing_status') != 'nominal_unmeasured'):
        raise ValueError('Unsupported actuator test format')
    positive(program['nominal_forward_mps'], 'forward speed')
    positive(program['nominal_reverse_mps'], 'reverse speed')
    frames = program['frames']
    if not isinstance(frames, list) or not frames or len(frames) > 200:
        raise ValueError('Invalid/empty sequence')
    total, stopped, previous_step, previous_direction = 0, 0, 0, 0
    for frame in frames:
        step, left, right, ticks = (frame[k] for k in ('steering', 'left_pwm', 'right_pwm', 'ticks'))
        if (type(step) is not int or not -7 <= step <= 7 or type(ticks) is not int
                or not 1 <= ticks <= 1200 or type(left) is not int or type(right) is not int
                or left != right or left not in (-PWM, 0, PWM)):
            raise ValueError('Require steering -7..7, PWM 0/+50/-50 and 0.05..60 s per segment')
        total += ticks
        if total > 6000:
            raise ValueError('Sequence exceeds 300 seconds')
        if step != previous_step:
            if left:
                raise ValueError('Steering changes require a stopped segment')
            stopped = 0
        if left == 0:
            stopped += ticks
        else:
            direction = 1 if left > 0 else -1
            if direction != previous_direction and stopped < 12:
                raise ValueError('Direction changes require at least 0.60 s stopped')
            if stopped and stopped < 6:
                raise ValueError('Steering changes require at least 0.30 s stopped')
            stopped, previous_direction = 0, direction
        previous_step = step
    if frames[0]['left_pwm'] or frames[-1]['left_pwm'] or frames[-1]['steering'] != 0 or stopped < 6:
        raise ValueError('Start stopped; finish stopped and centred for at least 0.30 s')
    return dict(duration_s=round(total*DT, 2), frames=len(frames))


def packet(frame):
    return f"s{frame['steering']}l{frame['left_pwm']}r{frame['right_pwm']}\n".encode('ascii')


def run_sequence(program, write=None, *, monotonic=time.monotonic, sleep=time.sleep, progress=print):
    """20 Hz; reject missed deadlines; always stop with the last steering held."""
    validate_sequence(program)
    last_step = 0
    deadline = monotonic()
    try:
        for index, frame in enumerate(program['frames'], 1):
            progress(f"[주차 동작] {index}/{len(program['frames'])} "
                     f"조향 {frame['steering']:+d}, PWM {frame['left_pwm']:+d}, {frame['ticks']*DT:.2f}초")
            command = packet(frame)
            for _ in range(frame['ticks']):
                sleep(max(0., deadline-monotonic()))
                if monotonic()-deadline > .15:
                    raise RuntimeError('Scheduling overrun: stopping instead of catching up')
                # Record before write: a driver may transmit then raise an error.
                last_step = frame['steering']
                if write is not None and write(command) != len(command):
                    raise RuntimeError('Incomplete serial write')
                deadline += DT
        sleep(max(0., deadline-monotonic()))
    finally:
        if write is not None:
            stop = packet(dict(steering=last_step, left_pwm=0, right_pwm=0))
            for _ in range(3):
                try:
                    write(stop)
                except Exception:
                    break
    progress('[주차 동작] 시퀀스 종료 — 실제 주차 성공 판정은 수행하지 않음')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--program', type=Path)
    parser.add_argument('--export', type=Path)
    parser.add_argument('--dry-run', action='store_true', help='Timed preview without serial access')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--ack-unmeasured-test', action='store_true')
    parser.add_argument('--port')
    args = parser.parse_args(argv)
    program = json.loads(args.program.read_text()) if args.program else build_sequence()
    summary = validate_sequence(program)
    if args.execute and (args.dry_run or not args.ack_unmeasured_test or not args.port):
        parser.error('Hardware needs --execute --ack-unmeasured-test --port, without --dry-run')
    if args.export:
        args.export.parent.mkdir(parents=True, exist_ok=True)
        args.export.write_text(json.dumps(program, ensure_ascii=False, indent=2)+'\n')
    print(f"[주차 동작] PWM ±50, {summary['frames']}구간, {summary['duration_s']:.2f}초 · 미측정 가정", flush=True)
    if not args.execute and not args.dry_run:
        for index, frame in enumerate(program['frames'], 1):
            print(f"{index:02d}: {packet(frame).decode().strip()} / {frame['ticks']*DT:.2f}s")
        print('PREVIEW ONLY: no serial imported or port opened.')
        return 0
    def interrupted(signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    if not args.execute:
        run_sequence(program)
        return 0
    # Explicit actuator test: no fake measured calibration and no ROS bypass.
    import serial
    with serial.Serial(args.port, 115200, timeout=.1, write_timeout=.1, exclusive=True) as port:
        started = False
        try:
            print('[주차 동작] Arduino 시작 대기 4.5초', flush=True)
            time.sleep(4.5)
            started = True
            run_sequence(program, port.write)
        finally:
            # A stop during board startup must also request zero drive.
            if not started:
                try:
                    port.write(packet(dict(steering=0, left_pwm=0, right_pwm=0)))
                except Exception:
                    pass
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('[주차 동작] 중단 — 정지 요청', flush=True)
        raise SystemExit(130)
    except (ValueError, KeyError, TypeError, OSError, RuntimeError) as exc:
        print(f'[주차 동작] ERROR: {exc}', flush=True)
        raise SystemExit(1)
