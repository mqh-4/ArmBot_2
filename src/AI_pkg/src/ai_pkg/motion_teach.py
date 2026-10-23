"""Pure helpers that turn hand-taught servo samples into a playable motion bag.

The recorder samples real servo positions through
``/ros_robot_controller/bus_servo/get_state`` while the servos are unloaded, so
the arm can be moved by hand. These helpers trim idle time, resample to a fixed
rate and build the ``(id, position, duration)`` command plan carried by
``MultiRawIdPosDur``. No ROS imports live here so the logic stays unit-testable.
"""
import math

MIN_POSITION = 0
MAX_POSITION = 1000
MIN_DURATION = 0.02
MAX_DURATION = 0.5


def clamp_position(value):
    """Round and clamp a raw bus-servo tick to the encoder range."""
    return max(MIN_POSITION, min(MAX_POSITION, int(round(float(value)))))


def servo_ids(samples):
    ids = set()
    for _, positions in samples:
        ids.update(positions)
    return sorted(ids)


def max_delta(previous, current):
    """Largest per-servo tick change between two samples."""
    ids = set(previous) | set(current)
    if not ids:
        return 0
    return max(abs(current.get(i, previous.get(i, 0)) - previous.get(i, current.get(i, 0)))
               for i in ids)


def motion_bounds(samples, tick_threshold=3):
    """Index range of samples that actually move, or ``None`` for a static take."""
    if len(samples) < 2:
        return None
    moving = [index for index in range(1, len(samples))
              if max_delta(samples[index - 1][1], samples[index][1]) > tick_threshold]
    if not moving:
        return None
    return moving[0], moving[-1]


def trim_idle(samples, tick_threshold=3, pad_seconds=0.3):
    """Drop the still lead-in and tail so playback starts when motion starts."""
    if len(samples) < 2:
        return list(samples)
    bounds = motion_bounds(samples, tick_threshold)
    if bounds is None:
        return list(samples)
    first, last = bounds
    # The epsilon keeps a sample that sits exactly on the boundary in the take.
    start = samples[first][0] - pad_seconds - 1e-9
    end = samples[last][0] + pad_seconds + 1e-9
    trimmed = [(time, dict(positions)) for time, positions in samples
               if start <= time <= end]
    if len(trimmed) < 2:
        return [(samples[first][0], dict(samples[first][1])),
                (samples[last][0], dict(samples[last][1]))]
    return trimmed


def smooth(samples, alpha=0.35):
    """Exponential smoothing to take the serial-read jitter out of a hand take."""
    if not 0 < alpha <= 1:
        raise ValueError('alpha must be in (0, 1]')
    if not samples:
        return []
    state = dict(samples[0][1])
    result = []
    for time, positions in samples:
        for servo_id, value in positions.items():
            state[servo_id] = (value if servo_id not in state
                               else state[servo_id] + alpha * (value - state[servo_id]))
        result.append((time, {servo_id: state[servo_id] for servo_id in positions}))
    return result


def resample(samples, rate_hz):
    """Resample onto a uniform time grid with per-servo linear interpolation."""
    if rate_hz <= 0:
        raise ValueError('rate_hz must be positive')
    if len(samples) < 2:
        return [(time, dict(positions)) for time, positions in samples]
    ids = servo_ids(samples)
    start, end = samples[0][0], samples[-1][0]
    if end <= start:
        return [(start, {servo_id: samples[-1][1].get(servo_id, 0) for servo_id in ids})]

    grid = []
    step = 1.0 / rate_hz
    count = max(1, int(round((end - start) * rate_hz)))
    for index in range(count + 1):
        grid.append(start + index * step)
    # Pin the final sample to the real capture time so the take never runs long.
    grid[-1] = end

    result = []
    index = 0
    for time in grid:
        while index + 2 < len(samples) and samples[index + 1][0] <= time:
            index += 1
        time_a, positions_a = samples[index]
        time_b, positions_b = samples[min(index + 1, len(samples) - 1)]
        span = time_b - time_a
        ratio = 0.0 if span <= 0 else min(1.0, max(0.0, (time - time_a) / span))
        interpolated = {}
        for servo_id in ids:
            value_a = positions_a.get(servo_id, positions_b.get(servo_id, 0))
            value_b = positions_b.get(servo_id, value_a)
            interpolated[servo_id] = value_a + (value_b - value_a) * ratio
        result.append((time, interpolated))
    return result


def command_plan(samples, entry_duration=1.0, min_delta=2):
    """Build ``(offset_seconds, [(id, position, duration), ...])`` for the bag.

    The first entry commands every servo to the taught start pose over
    ``entry_duration`` so playback eases in instead of jumping. Later entries
    only carry servos that moved at least ``min_delta`` ticks, and each
    servo's duration covers the time since its previous command.
    """
    if not samples:
        raise ValueError('no samples')
    if entry_duration <= 0:
        raise ValueError('entry_duration must be positive')
    if min_delta < 1:
        raise ValueError('min_delta must be at least 1 tick')

    ids = servo_ids(samples)
    if not ids:
        raise ValueError('no servo ids in samples')

    start = samples[0][0]
    first = samples[0][1]
    initial = [(servo_id, clamp_position(first[servo_id]), entry_duration)
               for servo_id in ids if servo_id in first]
    if not initial:
        raise ValueError('first sample has no servo positions')

    plan = [(0.0, initial)]
    last_value = {servo_id: position for servo_id, position, _ in initial}
    last_time = {servo_id: start for servo_id, _, _ in initial}

    for index in range(1, len(samples)):
        time, positions = samples[index]
        changed = []
        for servo_id in ids:
            if servo_id not in positions:
                continue
            position = clamp_position(positions[servo_id])
            if abs(position - last_value.get(servo_id, position)) < min_delta:
                continue
            duration = time - last_time.get(servo_id, start)
            duration = max(MIN_DURATION, min(MAX_DURATION, duration))
            changed.append((servo_id, position, duration))
            last_value[servo_id] = position
            last_time[servo_id] = time
        if changed:
            plan.append((time - start, changed))
    return plan


def plan_duration(plan):
    return plan[-1][0] if plan else 0.0


def plan_summary(plan):
    """Counts used by the recorder's final report."""
    messages = len(plan)
    commands = sum(len(entries) for _, entries in plan)
    per_servo = {}
    for _, entries in plan:
        for servo_id, _, _ in entries:
            per_servo[servo_id] = per_servo.get(servo_id, 0) + 1
    return {'messages': messages, 'commands': commands,
            'duration': round(plan_duration(plan), 3), 'per_servo': per_servo}


def find_extrema(samples, alpha=0.3, min_span=0.3, min_amplitude=3.0):
    """Indices of direction changes per servo, used as cosine keyframes."""
    if len(samples) < 3:
        return {servo_id: [0, len(samples) - 1]
                for servo_id in servo_ids(samples)} or {}

    ids = servo_ids(samples)
    times = [time for time, _ in samples]
    result = {}
    for servo_id in ids:
        values = []
        state = None
        for _, positions in samples:
            value = positions.get(servo_id)
            if value is None:
                value = state if state is not None else 0.0
            state = value
            values.append(value)

        smoothed = []
        state = values[0]
        for value in values:
            state += alpha * (value - state)
            smoothed.append(state)

        keys = [0]
        direction = 0
        for index in range(1, len(smoothed)):
            delta = smoothed[index] - smoothed[index - 1]
            if abs(delta) < 1e-9:
                continue
            sign = 1 if delta > 0 else -1
            if direction == 0:
                direction = sign
                continue
            if sign == direction:
                continue
            candidate = index - 1
            if (times[candidate] - times[keys[-1]] >= min_span
                    and abs(smoothed[candidate] - smoothed[keys[-1]]) >= min_amplitude):
                keys.append(candidate)
            direction = sign
        keys.append(len(samples) - 1)
        result[servo_id] = sorted(set(keys))
    return result


def cosine_ease(samples, alpha=0.3, min_span=0.3, min_amplitude=3.0):
    """Rebuild each servo curve with cosine segments between turning points.

    Velocity reaches zero at every keyframe, so the result is continuous in
    position and velocity even when the measured samples were noisy.
    """
    if len(samples) < 3:
        return list(samples)
    keys = find_extrema(samples, alpha, min_span, min_amplitude)
    times = [time for time, _ in samples]
    ids = sorted(keys)
    curves = {}
    for servo_id in ids:
        values = [positions.get(servo_id, 0.0) for _, positions in samples]
        eased = []
        for first, second in zip(keys[servo_id], keys[servo_id][1:]):
            if second <= first:
                continue
            start_time, end_time = times[first], times[second]
            start_value, end_value = values[first], values[second]
            span = end_time - start_time
            for index in range(first, second):
                ratio = 0.0 if span <= 0 else (times[index] - start_time) / span
                ratio = min(1.0, max(0.0, ratio))
                weight = 0.5 - 0.5 * math.cos(math.pi * ratio)
                eased.append(start_value + (end_value - start_value) * weight)
        eased.append(values[-1])
        if len(eased) < len(samples):
            eased.extend([values[-1]] * (len(samples) - len(eased)))
        curves[servo_id] = eased[:len(samples)]
    return [(times[index],
             {servo_id: curves[servo_id][index] for servo_id in ids})
            for index in range(len(samples))]
