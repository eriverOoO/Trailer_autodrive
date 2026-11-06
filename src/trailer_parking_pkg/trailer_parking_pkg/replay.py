"""CPU-only parking-plan compilation for the modified ride-on car.

No ROS, serial, GPU or simulator imports. All distances use metres and radians.
The planner uses a measured front-steer/dual-PWM profile. Its predicted
states are model results, never measurements or a claim of successful parking.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import tempfile

from .core import Geometry, State, Slot, Scene, advance, goal_reached, plan, wrap

DT = 0.05  # 20 Hz, matching the supplied Arduino's 50 ms control interval.
MAX_TICKS = 12000


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def integer(value, low, high):
    return type(value) is int and low <= value <= high


def save_json(path, data):
    """Replace atomically; a failed export must not leave a partial program."""
    payload = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                     delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(payload + '\n')
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class Platform:
    """Explicit calibration, with no Prius/MVSim or linear-PWM fallback.

    Each drive sample is a measured vehicle speed for one left/right PWM pair.
    As in this platform's controller, steering comes from the steering motor,
    not a differential-drive yaw command. Pairs may include measured motor trim.
    """
    def __init__(self, data):
        self.data = data
        if data.get('schema') != 'ride_on_platform_v1' or data.get('drive_type') != 'front_steer_dual_pwm':
            raise ValueError('Require a front_steer_dual_pwm platform profile')
        if data.get('calibration_status') not in ('example', 'measured'):
            raise ValueError('calibration_status must be example or measured')
        if set(data['geometry']) != set(Geometry.__dataclass_fields__):
            raise ValueError('Supply every geometry field explicitly; no simulator defaults')
        if not all(finite(v) and v > 0 for v in data['geometry'].values()):
            raise ValueError('Unmeasured geometry: replace nulls with measured positive SI values')
        self.g = Geometry(**data['geometry'])
        table = data['steering_radians']
        if set(table) != {str(i) for i in range(-7, 8)}:
            raise ValueError('Measure all steering steps -7..7')
        self.angles = {int(key): value for key, value in table.items()}
        angles = [self.angles[i] for i in range(-7, 8)]
        if (not all(finite(a) and abs(a) <= self.g.max_steer + 1e-9 for a in angles)
                or abs(self.angles[0]) > 1e-9
                or not all(a > b for a, b in zip(angles, angles[1:]))):
            raise ValueError('Steering table: -7=left positive radians, 0=straight, +7=right')
        self.samples = data['drive_samples']
        if not isinstance(self.samples, list) or not self.samples:
            raise ValueError('Supply measured forward and reverse drive samples')
        pairs = set()
        for row in self.samples:
            speed, left, right = row['speed_mps'], row['left_pwm'], row['right_pwm']
            if (not finite(speed) or speed == 0 or
                    not integer(left, -255, 255) or not integer(right, -255, 255) or
                    left * speed <= 0 or right * speed <= 0 or (left, right) in pairs):
                raise ValueError('Invalid/duplicate signed speed and PWM sample')
            pairs.add((left, right))
        if not any(r['speed_mps'] > 0 for r in self.samples) or not any(r['speed_mps'] < 0 for r in self.samples):
            raise ValueError('Separate measured forward and reverse samples are required')
        for name in ('steering_settle_s', 'gear_settle_s'):
            if not finite(data[name]) or not 0.05 <= data[name] <= 10:
                raise ValueError('Invalid steering/gear settling time')

    def steering(self, radians):
        return min(self.angles, key=lambda step: abs(self.angles[step] - radians))

    def drive(self, requested):
        return min((r for r in self.samples if r['speed_mps'] * requested > 0),
                   key=lambda row: abs(row['speed_mps'] - requested))

    def speed(self, left, right):
        if left == right == 0:
            return 0.0
        for row in self.samples:
            if (left, right) == (row['left_pwm'], row['right_pwm']):
                return row['speed_mps']
        raise ValueError('Command uses a PWM pair absent from this calibration')


def snapshot_parts(snapshot, platform):
    if snapshot.get('schema') != 'parking_snapshot_v1':
        raise ValueError('Unsupported parking snapshot')
    if snapshot['geometry'] != platform.data['geometry']:
        raise ValueError('Snapshot geometry differs from the vehicle profile; replan')
    state, slot = State(**snapshot['start']), Slot(**snapshot['slot'])
    if not state.valid() or not all(finite(v) for v in asdict(slot).values()) or min(slot.length, slot.width) <= 0:
        raise ValueError('Invalid snapshot state or slot')
    raw = snapshot['scene']
    bounds, margin = raw['bounds'], raw['margin']
    if (len(bounds) != 4 or not all(finite(v) for v in bounds) or
            bounds[0] >= bounds[1] or bounds[2] >= bounds[3] or not finite(margin) or margin < 0):
        raise ValueError('Invalid scene bounds/margin')
    obstacles = tuple(tuple(tuple(p) for p in poly) for poly in raw['obstacles'])
    if any(len(poly) < 3 or any(len(p) != 2 or not all(finite(v) for v in p) for p in poly) for poly in obstacles):
        raise ValueError('Invalid obstacle polygon')
    scene = Scene(obstacles, tuple(bounds), margin)
    if not scene.free(state, platform.g):
        raise ValueError('Initial vehicle footprint is not free')
    return state, slot, scene


def advance_checked(state, speed, angle, scene, geometry):
    # Recheck between command ticks as well as at their ends.
    divisions = max(1, math.ceil(abs(speed * DT) / 0.02))
    for _ in range(divisions):
        state = advance(state, speed * DT / divisions, angle, geometry)
        if not scene.free(state, geometry):
            raise ValueError('Quantized hardware commands violate collision/articulation limits')
    return state


def compile_path(path, snapshot, profile, speed=0.12):
    platform = Platform(profile)
    state, slot, scene = snapshot_parts(snapshot, platform)
    if not finite(speed) or speed <= 0 or not path or path[0].state != state:
        raise ValueError('Invalid speed or path start')
    if not goal_reached(path[-1].state, slot, platform.g):
        raise ValueError('Planner did not reach the complete-rig goal')
    frames, ticks = [], 0

    def append(steering, left, right, count=1):
        nonlocal ticks
        ticks += count
        if ticks > MAX_TICKS:
            raise ValueError('Command compilation exceeded 600 seconds')
        command = dict(steering=steering, left_pwm=left, right_pwm=right)
        if frames and all(frames[-1][k] == v for k, v in command.items()):
            frames[-1]['ticks'] += count
        else:
            frames.append(dict(ticks=count, **command))

    def hold(step, seconds):
        append(step, 0, 0, math.ceil(seconds / DT))

    hold(0, profile['steering_settle_s'])
    previous_step, previous_direction, fractional_ticks = 0, 0, 0.0
    previous_planned = path[0].state
    for point in path[1:]:
        if point.direction not in (-1, 1) or not point.state.valid():
            raise ValueError('Invalid planned direction/state')
        step = platform.steering(point.steer)
        if abs(platform.angles[step] - point.steer) > 1e-9:
            raise ValueError('Replan with calibrated steering steps before compiling')
        row = platform.drive(point.direction * speed)
        direction = 1 if row['speed_mps'] > 0 else -1
        chord = math.hypot(point.state.x - previous_planned.x, point.state.y - previous_planned.y)
        curvature = math.tan(point.steer) / platform.g.wheelbase
        distance = chord if abs(curvature) < 1e-9 else 2 * math.asin(min(1.0, abs(curvature) * chord / 2)) / abs(curvature)
        if abs(wrap(point.state.yaw - previous_planned.yaw - direction * distance * curvature)) > 1e-5:
            raise ValueError('Planned states do not match their steering/direction')
        if direction != previous_direction:
            fractional_ticks = 0.0
        exact_ticks = distance / abs(row['speed_mps']) / DT + fractional_ticks
        count = round(exact_ticks)
        fractional_ticks = exact_ticks - count
        previous_planned = point.state
        if not count:
            continue
        settling = 0.0
        if direction != previous_direction:
            settling = profile['gear_settle_s']
        if step != previous_step:
            settling = max(settling, profile['steering_settle_s'])
        if settling:
            # Set a new steering target with the drive wheels stopped.
            hold(step, settling)
        append(step, row['left_pwm'], row['right_pwm'], count)
        for _ in range(count):
            state = advance_checked(state, row['speed_mps'], platform.angles[step], scene, platform.g)
        previous_step, previous_direction = step, direction
    hold(0, max(0.5, profile['steering_settle_s']))
    program = dict(schema='parking_commands_v1', dt_s=DT, profile=profile,
                   snapshot=snapshot, planned_path=[asdict(p) for p in path], frames=frames,
                   note='Predicted with calibrated kinematics; open-loop replay has no live localization.')
    program['prediction'] = validate_program(program)
    return program


def build_program(snapshot, profile, speed=0.12, timeout=30.0):
    platform = Platform(profile)
    state, slot, scene = snapshot_parts(snapshot, platform)
    path = plan(state, slot, scene, platform.g, timeout=timeout,
                steering_values=tuple(platform.angles.values()))
    return compile_path(path, snapshot, profile, speed)


def validate_program(program, hardware=False):
    if program.get('schema') != 'parking_commands_v1' or program.get('dt_s') != DT:
        raise ValueError('Unsupported command schema or command period')
    platform = Platform(program['profile'])
    if hardware and platform.data['calibration_status'] != 'measured':
        raise ValueError('Example calibration cannot be sent to hardware')
    state, slot, scene = snapshot_parts(program['snapshot'], platform)
    frames = program['frames']
    if not isinstance(frames, list) or not frames:
        raise ValueError('Empty command file')
    ticks, motion_ticks, previous_step, previous_direction, stopped = 0, 0, 0, 0, 0
    for frame in frames:
        step, left, right, count = (frame[k] for k in ('steering', 'left_pwm', 'right_pwm', 'ticks'))
        if (not integer(step, -7, 7) or not integer(left, -255, 255) or
                not integer(right, -255, 255) or not integer(count, 1, MAX_TICKS)):
            raise ValueError('Invalid integer command or duration')
        ticks += count
        if ticks > MAX_TICKS:
            raise ValueError('Command file exceeds 600 seconds')
        velocity = platform.speed(left, right)
        if step != previous_step:
            if velocity:
                raise ValueError('New steering targets must settle with drive wheels stopped')
            stopped = 0
        if velocity == 0:
            stopped += count
        else:
            direction = 1 if velocity > 0 else -1
            needed = math.ceil(platform.data['steering_settle_s'] / DT)
            if direction != previous_direction:
                needed = max(needed, math.ceil(platform.data['gear_settle_s'] / DT))
            if stopped and stopped < needed:
                raise ValueError('Insufficient steering/direction settling time')
            if not motion_ticks and stopped < needed:
                raise ValueError('Missing initial settling stop')
            if direction != previous_direction and stopped < needed:
                raise ValueError('Direction change lacks a settling stop')
            for _ in range(count):
                state = advance_checked(state, velocity, platform.angles[step], scene, platform.g)
            motion_ticks += count
            previous_direction, stopped = direction, 0
        previous_step = step
    if frames[-1]['steering'] != 0 or frames[-1]['left_pwm'] or frames[-1]['right_pwm']:
        raise ValueError('Program must end stopped with steering centred')
    if not goal_reached(state, slot, platform.g):
        raise ValueError('Commands do not reach the goal in the vehicle model')
    return dict(duration_s=round(ticks * DT, 3), motion_s=round(motion_ticks * DT, 3),
                frames=len(frames), terminal_state=asdict(state), model_goal_reached=True,
                calibration_status=platform.data['calibration_status'])


def transmit(program, serial_port, encode, monotonic, sleep):
    """Only caller-supplied I/O; the CLI alone can open the Arduino port."""
    validate_program(program, hardware=True)
    deadline = monotonic()
    try:
        for frame in program['frames']:
            packet = encode(frame['steering'], frame['left_pwm'], frame['right_pwm'])
            for _ in range(frame['ticks']):
                delay = deadline - monotonic()
                if delay > 0:
                    sleep(delay)
                if monotonic() - deadline > 0.15:
                    raise RuntimeError('Replay scheduling overrun; stopping instead of catching up')
                if serial_port.write(packet) != len(packet):
                    raise RuntimeError('Incomplete serial write')
                deadline += DT
        sleep(max(0, deadline - monotonic()))
    finally:
        serial_port.write(encode(0, 0, 0))
        serial_port.flush()
