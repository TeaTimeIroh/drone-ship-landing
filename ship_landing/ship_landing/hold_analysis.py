"""How well the drone held over the pad, from a hold_over_pad recording. No ROS."""
import csv
import math


def load(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append({k: (v if k in ('phase', 'landing') else float(v)) for k, v in row.items()})
    return rows


def percentile(values, share):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(share * len(ordered)))]


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def to_markdown(rows, title='Holding over the pad by lean angle'):
    hold = [r for r in rows if r['phase'] == 'hold']
    if len(hold) < 40:
        return f'# {title}\n\nThe hold phase is too short to analyse ({len(hold)} rows).\n'
    start = hold[0]['time']
    gap = [(r['time'] - start, math.hypot(r['true_east'], r['true_north'])) for r in hold]
    arrival = next((t for t, d in gap if d < 0.3), None)
    lines = [f'# {title}', '',
             f'- Hold phase: {gap[-1][0]:.0f} s, starting {gap[0][1]:.2f} m from the pad centre, '
             f'{hold[0]["true_height"]:.1f} m above the pad']
    if arrival is None:
        lines.append(f'- The drone never came within 0.3 m of the pad centre; closest {min(d for _, d in gap):.2f} m')
        return '\n'.join(lines) + '\n'
    after = [(r, d) for r, (t, d) in zip(hold, gap) if t > arrival + 3.0]
    lines.append(f'- First within 0.3 m of the centre after {arrival:.1f} s')
    if len(after) < 40:
        lines.append('- Too little time after arriving to judge the hold.')
        return '\n'.join(lines) + '\n'
    distances = [d for _, d in after]
    tracker = [math.hypot(r['east'] - r['true_east'], r['north'] - r['true_north']) for r, _ in after]
    lean = [math.hypot(r['lean_forward'], r['lean_left']) for r, _ in after]
    seen = sum(1 for r, _ in after if r['age'] < 0.5) / len(after)
    mean_east = sum(r['true_east'] for r, _ in after) / len(after)
    mean_north = sum(r['true_north'] for r, _ in after) / len(after)
    lines += [
        f'- From 3 s after arriving ({after[-1][0]["time"] - after[0][0]["time"]:.0f} s): distance from the pad centre '
        f'median {100 * percentile(distances, 0.5):.1f} cm, 95% within {100 * percentile(distances, 0.95):.1f} cm, '
        f'worst {100 * max(distances):.1f} cm',
        f'- Average position of the pad relative to the drone in that time: {100 * mean_east:+.1f} cm east, '
        f'{100 * mean_north:+.1f} cm north (a steady offset shows here)',
        f"- The tracker's error in that time: typical {100 * rms(tracker):.1f} cm, worst {100 * max(tracker):.1f} cm; "
        f'pad in view {100 * seen:.0f}% of the time',
        f'- Lean commanded: typical {rms(lean):.1f} deg, largest {max(lean):.1f} deg',
        f'- Height above the pad: {min(r["true_height"] for r, _ in after):.2f} to '
        f'{max(r["true_height"] for r, _ in after):.2f} m',
    ]
    return '\n'.join(lines) + '\n'
