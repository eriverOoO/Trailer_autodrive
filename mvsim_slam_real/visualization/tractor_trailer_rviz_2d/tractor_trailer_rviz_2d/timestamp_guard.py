"""Validate EBIMU's 60000 ms counter without changing received timestamps."""
import math


class SensorTimestampGuard:
    def __init__(self, period_ms=10.):
        self.period_ms = period_ms
        self.previous = self.wall = self.origin_wall = self.elapsed_ms = None
        self.rollovers = self.missing_packets = 0
        self.event = 'NO DATA'

    def observe(self, sensor_ms, wall):
        if not math.isfinite(sensor_ms) or not 0 <= sensor_ms < 60000 or sensor_ms != int(sensor_ms):
            raise ValueError('sensor timestamp outside documented counter range')
        if self.previous is None:
            self.previous, self.wall, self.origin_wall, self.elapsed_ms = sensor_ms, wall, wall, 0.
            self.event = 'FIRST_SAMPLE'
            return
        host_step = wall - self.wall
        if not 0 <= host_step <= .5:
            raise ValueError('sensor RX gap; restart bridge and realign')
        step = sensor_ms - self.previous
        rollover = step < 0
        if step == 0:
            raise ValueError('sensor timestamp repeated/old packet')
        if rollover:
            step += 60000
            if not (self.previous >= 59000 and sensor_ms < 1000 and 0 < step <= 100):
                raise ValueError('sensor timestamp reset/backward/old packet')
        elapsed = self.elapsed_ms + step
        # Bounded delivery jitter is allowed; backlogs and apparent clock jumps
        # must not establish a new heading epoch silently.
        if abs(elapsed - (wall - self.origin_wall) * 1000) > 250:
            raise ValueError('sensor/host timeline mismatch; possible stale packet or reset')
        missing = max(0, round(step / self.period_ms) - 1)
        self.missing_packets += missing
        self.rollovers += int(rollover)
        self.event = 'ROLLOVER' if rollover else ('PACKET_GAP' if missing else 'LIVE')
        self.previous, self.wall, self.elapsed_ms = sensor_ms, wall, elapsed
