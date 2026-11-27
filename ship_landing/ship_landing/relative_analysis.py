"""How well the camera-only tracker followed the truth, from a relative_check recording. No ROS."""
import csv
import math

SPAN = 0.3      # s over which the true relative position is differenced to get a true velocity
BANDS = [('above 2 m', 2.0, 99.0), ('0.6 to 2 m', 0.6, 2.0), ('below 0.6 m', -1.0, 0.6)]


def load(path):
    with open(path, newline='') as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def with_true_velocity(rows):
    """Add the pad's true velocity relative to the drone, centred on each row."""
    out, lo, hi = [], 0, 0
    for row in rows:
        t = row['time']
        while rows[lo]['time'] < t - SPAN / 2 and lo < len(rows) - 1:
            lo += 1
        while hi < len(rows) - 1 and rows[hi]['time'] < t + SPAN / 2:
            hi += 1
        dt = rows[hi]['time'] - rows[lo]['time']
        if dt < SPAN * 0.6:
            continue
        out.append(dict(row, true_velocity_east=(rows[hi]['true_east'] - rows[lo]['true_east']) / dt,
                        true_velocity_north=(rows[hi]['true_north'] - rows[lo]['true_north']) / dt))
    return out


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float('nan')


def percentile(values, share):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(share * len(ordered)))] if ordered else float('nan')


def errors(rows):
    position = [math.hypot(r['east'] - r['true_east'], r['north'] - r['true_north']) for r in rows]
    velocity = [math.hypot(r['velocity_east'] - r['true_velocity_east'],
                           r['velocity_north'] - r['true_velocity_north']) for r in rows]
    height = [r['height'] - r['true_height'] for r in rows if not math.isnan(r['height'])]
    return position, velocity, height


def velocity_lag(rows, limit=0.6, step=0.05):
    """Seconds by which the tracker's velocity runs behind the truth (negative: ahead)."""
    times = [r['time'] for r in rows]

    def error_at(shift):
        total, n, j = 0.0, 0, 0
        for r in rows:
            target = r['time'] + shift
            while j < len(rows) - 1 and times[j] < target:
                j += 1
            if abs(times[j] - target) > 0.06:
                continue
            total += ((rows[j]['velocity_east'] - r['true_velocity_east']) ** 2
                      + (rows[j]['velocity_north'] - r['true_velocity_north']) ** 2)
            n += 1
        return total / n if n > 50 else float('inf')

    best, best_error, shift = float('nan'), float('inf'), -limit
    while shift <= limit + 1e-9:
        e = error_at(shift)
        if e < best_error:
            best, best_error = shift, e
        shift += step
    return best


def to_markdown(rows, title='Camera-only pad tracking against the truth'):
    rows = with_true_velocity(rows)
    seen = [r for r in rows if r['age'] < 0.5]
    lines = [f'# {title}', '',
             f'- Recording: {rows[-1]["time"] - rows[0]["time"]:.0f} s, pad in view (seen in the last 0.5 s) '
             f'for {100.0 * len(seen) / len(rows):.0f}% of it',
             f'- True relative speed between pad and drone: up to {max(math.hypot(r["true_velocity_east"], r["true_velocity_north"]) for r in seen):.2f} m/s',
             '',
             '| Height above the pad | Share of the time | Position error: typical, 95% | Velocity error: typical, 95% | Height error: average |',
             '|---|---|---|---|---|']
    for name, low, high in [('all heights', -1.0, 99.0)] + BANDS:
        part = [r for r in seen if low <= r['true_height'] < high]
        if len(part) < 20:
            lines.append(f'| {name} | {100.0 * len(part) / len(seen):.0f}% | too little data | | |')
            continue
        position, velocity, height = errors(part)
        average_height = sum(height) / len(height) if height else float('nan')
        lines.append(f'| {name} | {100.0 * len(part) / len(seen):.0f}% '
                     f'| {100 * rms(position):.1f} cm, {100 * percentile(position, 0.95):.1f} cm '
                     f'| {100 * rms(velocity):.1f} cm/s, {100 * percentile(velocity, 0.95):.1f} cm/s '
                     f'| {100 * average_height:+.1f} cm |')
    lines += ['', f"- The tracker's velocity runs {velocity_lag(seen):+.2f} s behind the true one (negative means ahead).",
              '- "Typical" is the root mean square. Position and velocity are of the pad relative to the drone, '
              'in the horizontal plane.',
              '- Height error is the tracker\'s height minus the true height of the drone\'s centre above the pad.']
    return '\n'.join(lines) + '\n'
