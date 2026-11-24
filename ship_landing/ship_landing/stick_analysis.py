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
            f"| {label} | {first['roll_stick']:+.1f}, {first['pitch_stick']:+.1f}, {first['throttle_stick']:+.1f} "
            f"| {int(last['mode'])} | {nose:+.1f}°, {left:+.1f}° "
            f"| {moved[0]:+.2f}, {moved[1]:+.2f}, {last['up'] - first['up']:+.2f} m "
            f"| {speed[0]:+.2f}, {speed[1]:+.2f}, {(tail[-1]['up'] - tail[0]['up']) / dt:+.2f} m/s |")
    lines += ['', '- Mode 2 is ALT_HOLD, mode 4 is GUIDED.',
              '- "Nose-down" and "left-down" are positive when that side of the drone is lower, '
              'which is the way it then accelerates.',
              '- Forward and left are taken from the heading at the start of each step.']
    return '\n'.join(lines) + '\n'
