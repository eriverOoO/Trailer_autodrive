#!/usr/bin/env python3
"""Passive bounded capture; cannot send sensor commands. Logs real bytes only."""
import argparse
import datetime
import json
import time
from pathlib import Path
from imu_common import serial_arguments, make_parser, open_serial


def main():
    p = argparse.ArgumentParser(description=__doc__)
    serial_arguments(p)
    p.add_argument('--seconds', type=float, default=10.)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not 0 < args.seconds <= 60:
        p.error('--seconds must be >0 and <=60')
    args.output.mkdir(parents=True, exist_ok=False)
    parser = make_parser(args)
    samples, bins, count, error = [], {}, 0, None
    started = time.monotonic()
    opened = False
    with (args.output/'rx.bin').open('xb') as raw, (args.output/'rx_chunks.jsonl').open('x') as log:
        try:
            s = open_serial(args)
            opened = True
            started = time.monotonic()
            with s:
                while time.monotonic()-started < args.seconds:
                    data = s.read(min(8192, max(1, s.in_waiting)))
                    elapsed = time.monotonic()-started
                    if data:
                        raw.write(data)
                        count += len(data)
                        bins[int(elapsed)] = bins.get(int(elapsed), 0)+len(data)
                        log.write(json.dumps(dict(elapsed=elapsed, hex=data.hex(), bytes=len(data)))+'\n')
                        for sample in parser.feed(data):
                            samples.append(dict(elapsed=elapsed, **sample))
                    else:
                        time.sleep(.001)
        except (OSError, ValueError, RuntimeError) as exc:
            error = repr(exc)
    duration = time.monotonic()-started
    times = [x['elapsed'] for x in samples]
    rate = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None
    summary = dict(at=datetime.datetime.now().astimezone().isoformat(), port=args.port,
                   baud=args.baud, opened=opened, rx_bytes=count, tx_bytes=0,
                   duration_s=duration, bytes_per_second=count/duration,
                   per_second_bytes={i:bins.get(i, 0) for i in range(int(duration)+1)},
                   packets=parser.accepted, rejected=parser.rejected, discarded=parser.discarded,
                   packet_lengths=parser.lengths, packet_rate_hz=rate,
                   checksum='ASCII: not defined by manufacturer; not applicable',
                   layout=vars(args) | {'output':str(args.output)}, error=error)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    with (args.output/'samples.jsonl').open('x') as f:
        for sample in samples:
            f.write(json.dumps(sample)+'\n')
    print(json.dumps(summary, indent=2))
    return 1 if error else (2 if count == 0 else (3 if not samples else 0))


if __name__ == '__main__':
    raise SystemExit(main())
