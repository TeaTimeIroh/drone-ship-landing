"""Work out how the drone answered velocity commands from a response_check recording. No ROS.

For every change of the command the analysis looks along the commanded direction and
reports, once for the true motion and once for the autopilot's own picture of it:

- starting: how long until the drone reaches half and 90% of the commanded speed,
  and the highest speed it reaches;
- stopping: how long until it is down to half and 10% of the speed it had, and how
  far it travels after being told to stop.
"""
import csv
import math
from dataclasses import dataclass

SMOOTH = 0.3     # s, positions this far apart are differenced to get a speed


@dataclass
class Response:
    kind: str            # 'start' or 'stop'
    axis: str            # 'east' or 'north'
    speed: float         # m/s commanded (start) or held before the command (stop)
    t50: float           # s to cross half of that speed
    t_end: float         # s to reach 90% of it (start) or fall to 10% of it (stop)
    peak: float          # m/s, highest speed along the axis (start) or furthest reverse speed (stop)
    travel: float        # m moved along the axis in the 5 s after the command


def load(path):
    with open(path, newline='') as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def speeds(rows, source, axis):
    """Speed along `axis` from positions, as (time, speed) pairs."""
    key = f'{source}_{axis}'
    out, j = [], 0
    for i, row in enumerate(rows):
        while j < i and row['time'] - rows[j + 1]['time'] >= SMOOTH:
            j += 1
        dt = row['time'] - rows[j]['time']
        if dt >= SMOOTH * 0.5:
            out.append((row['time'] - dt / 2.0, (row[key] - rows[j][key]) / dt))
    return out


def first_crossing(series, t0, level, rising):
    for t, v in series:
        if t >= t0 and ((v >= level) if rising else (v <= level)):
            return t - t0
    return float('nan')


def responses(rows, source):
    """One Response per change of the command, from the positions of `source` ('true' or 'autopilot')."""
    found = []
    for i in range(1, len(rows)):
        for axis in ('east', 'north'):
            before, after = rows[i - 1][f'command_{axis}'], rows[i][f'command_{axis}']
            if before == after:
                continue
            t0 = rows[i]['time']
            series = speeds(rows, source, axis)
            window = [(t, v) for t, v in series if t0 <= t <= t0 + 5.0]
            key = f'{source}_{axis}'
            later = [r for r in rows if r['time'] <= t0 + 5.0]
            travel = later[-1][key] - rows[i][key]
            if after != 0.0:
                sign = math.copysign(1.0, after)
                signed = [(t, v * sign) for t, v in series]
                found.append(Response('start', axis, abs(after),
                                      first_crossing(signed, t0, 0.5 * abs(after), True),
                                      first_crossing(signed, t0, 0.9 * abs(after), True),
                                      max((v * sign for _, v in window), default=float('nan')), travel * sign))
            else:
                sign = math.copysign(1.0, before)
                signed = [(t, v * sign) for t, v in series]
                found.append(Response('stop', axis, abs(before),
                                      first_crossing(signed, t0, 0.5 * abs(before), False),
                                      first_crossing(signed, t0, 0.1 * abs(before), False),
                                      min((v * sign for _, v in window), default=float('nan')), travel * sign))
    return found


def lag(rows, axis, limit=1.0, step=0.05):
    """How far the autopilot's speed runs behind the true speed, in s (negative: ahead)."""
    true, believed = speeds(rows, 'true', axis), speeds(rows, 'autopilot', axis)
    if len(true) < 20:
        return float('nan')

    def at(series, t):
        for (t1, v1), (t2, v2) in zip(series, series[1:]):
            if t1 <= t <= t2:
                return v1 + (v2 - v1) * (t - t1) / (t2 - t1) if t2 > t1 else v1
        return None

    best, best_error = float('nan'), float('inf')
    shift = -limit
    while shift <= limit + 1e-9:
        errors = []
        for t, v in true:
            b = at(believed, t + shift)
            if b is not None:
                errors.append((b - v) ** 2)
        if errors and sum(errors) / len(errors) < best_error:
            best, best_error = shift, sum(errors) / len(errors)
        shift += step
    return best


def to_markdown(rows, title='Response to velocity commands'):
    lines = [f'# {title}', '',
             '| Command | Seen by | Half speed after | 90% (start) or 10% (stop) after | Peak or reverse speed | Moved in 5 s |',
             '|---|---|---|---|---|---|']
    for source, name in (('true', 'Gazebo (truth)'), ('autopilot', 'autopilot')):
        for r in responses(rows, source):
            what = f'{r.kind} {r.axis}, {r.speed:.1f} m/s'
            lines.append(f'| {what} | {name} | {r.t50:.2f} s | {r.t_end:.2f} s | {r.peak:+.2f} m/s | {r.travel:.2f} m |')
    lines.append('')
    for axis in ('east', 'north'):
        lines.append(f"- The autopilot's {axis} speed runs {lag(rows, axis):+.2f} s behind the true speed "
                     '(negative means ahead).')
    heights = [r['true_up'] for r in rows]
    lines.append(f'- True height during the recording: {min(heights):.2f} to {max(heights):.2f} m '
                 '(above the Gazebo ground, at the drone\'s centre).')
    return '\n'.join(lines) + '\n'
