"""What each stick did, from a stick_check recording. No ROS."""
import csv
import math

from ship_landing.levelling import heading, rotate


def load(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append({k: (v if k == 'step' else float(v)) for k, v in row.items()})
    return rows


def lean(row):
    """How the drone leans: (degrees towards its nose, degrees towards its left side)."""
    q = (row['qx'], row['qy'], row['qz'], row['qw'])
    up = rotate(q, (0.0, 0.0, 1.0))                  # the drone's up axis in the world
    yaw = heading(q)
    forward = up[0] * math.cos(yaw) + up[1] * math.sin(yaw)
    left = -up[0] * math.sin(yaw) + up[1] * math.cos(yaw)
    return math.degrees(math.asin(max(-1.0, min(1.0, forward)))), math.degrees(math.asin(max(-1.0, min(1.0, left))))


def steps(rows):
    """Rows grouped by step, in order."""
    groups = []
    for row in rows:
        if not groups or groups[-1][0] != row['step']:
            groups.append((row['step'], []))
        groups[-1][1].append(row)
    return groups


def lean_points(rows):
    """(axis, stick size, lean in degrees at the end of the push, seconds to 63% of it) per push."""
    points = []
    for label, group in steps(rows):
        first, last = group[0], group[-1]
        roll, pitch = first['roll_stick'], first['pitch_stick']
        if len(group) < 10 or (roll != 0.0) == (pitch != 0.0) or first['throttle_stick'] != 0.0:
            continue
        # A positive roll stick lowers the right side, a positive pitch stick raises the nose.
        if roll != 0.0:
            stick, series = roll, [(r['time'], -lean(r)[1]) for r in group]
        else:
            stick, series = pitch, [(r['time'], -lean(r)[0]) for r in group]
        sign = math.copysign(1.0, stick)
        tail = [v * sign for t, v in series if last['time'] - t <= 0.5]
        final = sum(tail) / len(tail)
        start = series[0][1] * sign
        rise = next((t - series[0][0] for t, v in series if (v * sign - start) >= 0.63 * (final - start)), float('nan'))
        points.append(('roll' if roll != 0.0 else 'pitch', abs(stick), final, rise, sign))
    return points


def calibration(rows):
    """Lines for the summary: lean per stick position, once pushes of several sizes exist."""
    points = lean_points(rows)
    sizes = sorted({p[1] for p in points if p[0] == 'roll'})
    if len(sizes) < 3:
        return []
    lines = ['', '## Lean for each stick position', '',
             '| Stick | Size | Direction | Lean at the end | 63% of the change after |', '|---|---|---|---|---|']
    for axis, size, final, rise, sign in points:
        lines.append(f'| {axis} | {size:.2f} | {"+" if sign > 0 else "-"} | {final:.2f}° | {rise:.2f} s |')
    # Straight line through the roll pushes: lean = slope * (stick - dead zone).
    xs = [p[1] for p in points if p[0] == 'roll']
    ys = [p[2] for p in points if p[0] == 'roll']
    n = len(xs)
    mean_x, mean_y = sum(xs) / n, sum(ys) / n
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / sum((x - mean_x) ** 2 for x in xs)
    dead_zone = mean_x - mean_y / slope
    worst = max(abs(y - slope * (x - dead_zone)) for x, y in zip(xs, ys))
    lines += ['', f'- Roll stick: lean = {slope:.1f}° x (stick - {dead_zone:.3f}). Full stick would be '
                  f'{slope * (1.0 - dead_zone):.1f}°. The pushes lie within {worst:.2f}° of that line.']
    pitch = [p for p in points if p[0] == 'pitch']
    if pitch:
        off = max(abs(p[2] - slope * (p[1] - dead_zone)) for p in pitch)
        lines.append(f'- Pitch stick: its pushes lie within {off:.2f}° of the same line.')
    return lines


def to_markdown(rows, title='Stick check'):
    lines = [f'# {title}', '',
             '| Step | Sticks (roll, pitch, throttle) | Mode | Lean at the end: nose-down, left-down | Moved: forward, left, up | Speed at the end: forward, left, up |',
             '|---|---|---|---|---|---|']
    for label, group in steps(rows):
        if len(group) < 5:
            continue
        first, last = group[0], group[-1]
        yaw = heading((first['qx'], first['qy'], first['qz'], first['qw']))

        def body(east, north):
            return (east * math.cos(yaw) + north * math.sin(yaw), -east * math.sin(yaw) + north * math.cos(yaw))

        moved = body(last['east'] - first['east'], last['north'] - first['north'])
        tail = [r for r in group if last['time'] - r['time'] <= 0.5]
        dt = max(tail[-1]['time'] - tail[0]['time'], 1e-6)
        speed = body((tail[-1]['east'] - tail[0]['east']) / dt, (tail[-1]['north'] - tail[0]['north']) / dt)
        leans = [lean(r) for r in tail]
        nose, left = sum(a for a, _ in leans) / len(leans), sum(b for _, b in leans) / len(leans)
        lines.append(
            f"| {label} | {first['roll_stick']:+.2f}, {first['pitch_stick']:+.2f}, {first['throttle_stick']:+.2f} "
            f"| {int(last['mode'])} | {nose:+.1f}°, {left:+.1f}° "
            f"| {moved[0]:+.2f}, {moved[1]:+.2f}, {last['up'] - first['up']:+.2f} m "
            f"| {speed[0]:+.2f}, {speed[1]:+.2f}, {(tail[-1]['up'] - tail[0]['up']) / dt:+.2f} m/s |")
    lines += ['', '- Mode 2 is ALT_HOLD, mode 4 is GUIDED.',
              '- "Nose-down" and "left-down" are positive when that side of the drone is lower, '
              'which is the way it then accelerates.',
              '- Forward and left are taken from the heading at the start of each step.']
    lines += calibration(rows)
    return '\n'.join(lines) + '\n'
