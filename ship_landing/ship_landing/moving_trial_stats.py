"""Summarise moving-pad landing trials. Pure Python, no ROS and no plotting.

A row is one landing attempt, as written by the moving_pad_trials node. The error is
the drone's true position relative to the pad centre, read from Gazebo after the pad
has been brought to rest with the drone standing on it.

Besides east and north, each error is also expressed relative to the way the pad was
travelling at touchdown: `along` is positive when the drone stands ahead of the pad
centre, `across` when it stands to the left. A drone that keeps arriving late shows up
as a negative average `along`.
"""
import csv
import math
import statistics

from ship_landing.trial_stats import error_stats

COLUMNS = ['batch', 'seed', 'trial', 'pattern', 'pad_speed', 'outcome', 'reason',
           'start_east', 'start_north', 'wait',
           'error_east', 'error_north', 'error', 'drone_z',
           'pad_v_east', 'pad_v_north', 'pad_velocity_change', 'along', 'across',
           'estimated_east', 'estimated_north',
           'track_time', 'final_attempts', 'stop_delay', 'land_mode', 'detections']

TEXT = ('batch', 'pattern', 'outcome', 'reason')
CHANGING = 0.08     # m/s: the pad's velocity changed by more than this between commitment and touchdown
MOVING = 0.05       # m/s: below this the pad has no direction of travel worth naming


def travel_frame(error_east, error_north, velocity, fallback=None):
    """(along, across) of an error relative to the pad's direction of travel, or (None, None).

    `velocity` is the pad's (east, north) velocity at touchdown; `fallback` is used when
    the pad was almost at rest then, as it is at the end of its line.
    """
    for v in (velocity, fallback):
        if v is None:
            continue
        speed = math.hypot(v[0], v[1])
        if speed >= MOVING:
            de, dn = v[0] / speed, v[1] / speed
            return (error_east * de + error_north * dn, de * error_north - dn * error_east)
    return (None, None)


def _number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def load_rows(paths):
    rows = []
    for path in paths:
        with open(path, newline='') as handle:
            for row in csv.DictReader(handle):
                for key in COLUMNS:
                    if key not in TEXT:
                        row[key] = _number(row.get(key))
                row['source'] = str(path)
                rows.append(row)
    return rows


def _mean(values):
    values = [v for v in values if v is not None]
    return statistics.fmean(values) if values else None


def _group(rows):
    errors = [r['error'] for r in rows]
    if not errors:
        return None
    return {'n': len(errors), 'median': statistics.median(errors), 'worst': max(errors)}


def summarise(rows):
    landed = [r for r in rows if r['outcome'] == 'landed' and r['error'] is not None]
    steady = [r for r in landed if r['pad_velocity_change'] is not None and r['pad_velocity_change'] < CHANGING]
    changing = [r for r in landed if r['pad_velocity_change'] is not None and r['pad_velocity_change'] >= CHANGING]
    with_frame = [r for r in landed if r['along'] is not None]
    estimate_gap = [math.hypot(r['estimated_east'] - r['error_east'], r['estimated_north'] - r['error_north'])
                    for r in landed if r['estimated_east'] is not None]
    batches = []
    for name in sorted({r['batch'] for r in rows}):
        group = [r for r in landed if r['batch'] == name]
        batches.append((name, sum(1 for r in rows if r['batch'] == name), error_stats([r['error'] for r in group])))
    return {
        'attempts': len(rows),
        'landed': len(landed),
        'missed': sum(1 for r in rows if r['outcome'] == 'missed'),
        'failed': sum(1 for r in rows if r['outcome'] == 'failed'),
        'reasons': sorted({r['reason'] for r in rows if r['reason']}),
        'patterns': sorted({f"{r['pattern']} at {r['pad_speed']:g} m/s" for r in rows if r['pad_speed'] is not None}),
        'error': error_stats([r['error'] for r in landed]),
        'over_15cm': sum(1 for r in landed if r['error'] > 0.15),
        'bias_along': _mean([r['along'] for r in with_frame]),
        'bias_across': _mean([r['across'] for r in with_frame]),
        'spread_along': statistics.pstdev([r['along'] for r in with_frame]) if len(with_frame) > 1 else None,
        'spread_across': statistics.pstdev([r['across'] for r in with_frame]) if len(with_frame) > 1 else None,
        'framed': len(with_frame),
        'steady': _group(steady),
        'changing': _group(changing),
        'estimate_gap': statistics.median(estimate_gap) if estimate_gap else None,
        'track_time': _mean([r['track_time'] for r in landed]),
        'track_time_max': max((r['track_time'] for r in landed if r['track_time'] is not None), default=None),
        'final_attempts': _mean([r['final_attempts'] for r in landed]),
        'stop_delay': _mean([r['stop_delay'] for r in landed]),
        'land_mode': sum(1 for r in rows if r['land_mode']),
        'batches': batches,
    }


def _cm(value):
    return 'n/a' if value is None else f'{100 * value:.1f} cm'


def to_markdown(summary):
    s = summary
    lines = ['# Moving pad landing trials', '']
    lines.append(f"- Pad motion: {', '.join(s['patterns']) or 'not recorded'}")
    lines.append(f"- Attempts: {s['attempts']}, landed on the pad and measured: {s['landed']}, "
                 f"touched down off the pad: {s['missed']}, failed: {s['failed']}")
    if s['reasons']:
        lines.append('- Failure reasons: ' + '; '.join(s['reasons']))
    e = s['error']
    if e is None:
        return '\n'.join(lines + ['', 'No landing was measured.', ''])
    lines.append(f"- Touchdown error from the pad centre: median {_cm(e['median'])}, mean {_cm(e['mean'])}, "
                 f"95th percentile {_cm(e['p95'])}, worst {_cm(e['max'])}")
    lines.append(f"- More than 15 cm off: {s['over_15cm']} of {s['landed']}")
    if s['bias_along'] is not None:
        side = 'ahead of' if s['bias_along'] > 0 else 'behind'
        lines.append(f"- Along the pad's direction of travel ({s['framed']} landings): on average "
                     f"{_cm(abs(s['bias_along']))} {side} the centre, spread {_cm(s['spread_along'])}. "
                     f"Across it: on average {_cm(abs(s['bias_across']))} to the "
                     f"{'left' if s['bias_across'] > 0 else 'right'}, spread {_cm(s['spread_across'])}")
    for label, key in (('held its velocity', 'steady'), ('changed its velocity', 'changing')):
        g = s[key]
        if g is not None:
            lines.append(f"- Landings where the pad {label} between commitment and touchdown: {g['n']}, "
                         f"median {_cm(g['median'])}, worst {_cm(g['worst'])}")
    if s['estimate_gap'] is not None:
        lines.append(f"- The drone's own estimate of its offset at touchdown differed from the truth by "
                     f"{_cm(s['estimate_gap'])} (median)")
    if s['track_time'] is not None:
        lines.append(f"- Time from the start point to touchdown: mean {s['track_time']:.1f} s, longest "
                     f"{s['track_time_max']:.1f} s (simulation time); final descents started per landing: "
                     f"{s['final_attempts']:.1f}")
    if s['stop_delay'] is not None:
        lines.append(f"- Motors stopped {s['stop_delay']:.1f} s after contact on average; LAND mode fallback used "
                     f"{s['land_mode']} times")
    lines += ['', '| Batch | Attempts | Landed | Median | Mean | 95th pct | Worst |', '|---|---|---|---|---|---|---|']
    for name, attempts, stats in s['batches']:
        if stats is None:
            lines.append(f'| {name} | {attempts} | 0 | | | | |')
        else:
            lines.append(f"| {name} | {attempts} | {stats['n']} | {_cm(stats['median'])} | {_cm(stats['mean'])} "
                         f"| {_cm(stats['p95'])} | {_cm(stats['max'])} |")
    return '\n'.join(lines + [''])
