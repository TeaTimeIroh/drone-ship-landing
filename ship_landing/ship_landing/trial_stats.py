"""Summarise landing-trial results. Pure Python, no ROS and no plotting.

A row is one landing attempt, as written by the landing_trials node. Errors are
measured from the pad centre to the drone's true touchdown position.
"""
import csv
import math
import statistics

PAD_HALF = 0.625        # m, half the side of the 1.25 m pad
MARKER_HALF = 0.5       # m, half the side of the 1.0 m marker

# One CSV row per landing attempt. Positions are in the Gazebo world frame (x east, y north).
COLUMNS = ['batch', 'seed', 'trial', 'outcome', 'reason',
           'start_forward', 'start_left', 'start_x', 'start_y', 'start_z', 'start_distance',
           'detector_range', 'detector_offset', 'true_height_above_pad',
           'land_x', 'land_y', 'land_z', 'error_x', 'error_y', 'error',
           'approach_time', 'handover_offset', 'handover_height']

NUMERIC = ('trial', 'seed', 'start_forward', 'start_left', 'start_x', 'start_y', 'start_z',
           'start_distance', 'detector_range', 'detector_offset', 'true_height_above_pad',
           'land_x', 'land_y', 'land_z', 'error_x', 'error_y', 'error', 'approach_time',
           'handover_offset', 'handover_height')


def _number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def load_rows(paths):
    """Read one or more CSV files into a list of dicts, numbers converted to float."""
    rows = []
    for path in paths:
        with open(path, newline='') as handle:
            for row in csv.DictReader(handle):
                for key in NUMERIC:
                    row[key] = _number(row.get(key))
                row['source'] = str(path)
                rows.append(row)
    return rows


def percentile(values, fraction):
    """Nearest-rank percentile: the smallest value with at least `fraction` of the data at or below it."""
    ordered = sorted(values)
    rank = max(1, math.ceil(fraction * len(ordered)))
    return ordered[rank - 1]


def error_stats(errors):
    if not errors:
        return None
    return {
        'n': len(errors),
        'mean': statistics.fmean(errors),
        'median': statistics.median(errors),
        'p95': percentile(errors, 0.95),
        'max': max(errors),
        'stdev': statistics.stdev(errors) if len(errors) > 1 else 0.0,
    }


def summarise(rows):
    """Overall and per-batch statistics for a list of trial rows."""
    measured = [r for r in rows if r['outcome'] == 'landed' and r['error'] is not None]
    summary = {
        'attempts': len(rows),
        'landed': len(measured),
        'failed': [r for r in rows if r['outcome'] != 'landed'],
        'error': error_stats([r['error'] for r in measured]),
        'on_pad': sum(1 for r in measured
                      if abs(r['error_x']) <= PAD_HALF and abs(r['error_y']) <= PAD_HALF),
        'on_marker': sum(1 for r in measured
                         if abs(r['error_x']) <= MARKER_HALF and abs(r['error_y']) <= MARKER_HALF),
        'batches': {},
        'bias': None,
        'range_ratio': None,
        'approach_time': None,
    }
    if measured:
        summary['bias'] = (statistics.fmean(r['error_x'] for r in measured),
                           statistics.fmean(r['error_y'] for r in measured))
    times = [r['approach_time'] for r in measured if r['approach_time'] is not None]
    if times:
        summary['approach_time'] = (statistics.fmean(times), max(times))
    ratios = [r['detector_range'] / r['true_height_above_pad'] for r in rows
              if r['detector_range'] and r['true_height_above_pad'] and r['true_height_above_pad'] > 0.5]
    if ratios:
        summary['range_ratio'] = (statistics.fmean(ratios), min(ratios), max(ratios), len(ratios))
    for row in measured:
        summary['batches'].setdefault(row['batch'], []).append(row['error'])
    summary['batches'] = {name: error_stats(errors) for name, errors in sorted(summary['batches'].items())}
    return summary


def to_markdown(summary):
    """Format a summary as a short Markdown report."""
    cm = lambda metres: f'{100 * metres:.1f} cm'      # noqa: E731
    lines = ['# Landing trials', '']
    lines.append(f'- Attempts: {summary["attempts"]}, landed and measured: {summary["landed"]}, '
                 f'failed: {len(summary["failed"])}')
    stats = summary['error']
    if stats is None:
        lines.append('- No successful landings to report.')
        return '\n'.join(lines) + '\n'
    lines.append(f'- Touchdown error from the pad centre: median {cm(stats["median"])}, '
                 f'mean {cm(stats["mean"])}, 95th percentile {cm(stats["p95"])}, worst {cm(stats["max"])}')
    lines.append(f'- Inside the 1.0 m marker: {summary["on_marker"]}/{summary["landed"]}; '
                 f'on the 1.25 m pad: {summary["on_pad"]}/{summary["landed"]}')
    east, north = summary['bias']
    lines.append(f'- Average miss direction: {cm(east)} east, {cm(north)} north '
                 f'(a consistent offset here points to a systematic cause)')
    if summary['approach_time']:
        mean_time, max_time = summary['approach_time']
        lines.append(f'- Time from start point to touchdown: mean {mean_time:.1f} s, longest {max_time:.1f} s '
                     f'(simulation time)')
    if summary['range_ratio']:
        mean_ratio, low, high, count = summary['range_ratio']
        lines.append(f'- Detector range / true height of the drone above the pad at the start point: '
                     f'mean {mean_ratio:.3f} (range {low:.3f} to {high:.3f}, {count} readings). '
                     f'The camera hangs roughly 0.1 m below the point Gazebo reports, '
                     f'so a perfect detector would read slightly under 1.')
    lines += ['', '| Batch | Landings | Median | Mean | 95th pct | Worst |', '|---|---|---|---|---|---|']
    for name, batch in summary['batches'].items():
        lines.append(f'| {name} | {batch["n"]} | {cm(batch["median"])} | {cm(batch["mean"])} | '
                     f'{cm(batch["p95"])} | {cm(batch["max"])} |')
    if summary['failed']:
        lines += ['', '## Failed attempts', '']
        for row in summary['failed']:
            lines.append(f'- batch {row["batch"]}, trial {int(row["trial"])}: {row["reason"] or "no reason recorded"}')
    return '\n'.join(lines) + '\n'
