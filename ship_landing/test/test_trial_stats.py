"""Check the landing-trial statistics on a small hand-made result set."""
import csv

import pytest

from ship_landing.trial_stats import COLUMNS, error_stats, load_rows, percentile, summarise, to_markdown


def write(path, rows):
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, COLUMNS)
        writer.writeheader()
        for values in rows:
            row = dict.fromkeys(COLUMNS, '')
            row.update(values)
            writer.writerow(row)
    return path


def landed(batch, trial, error_x, error_y, **extra):
    return dict(batch=batch, trial=trial, outcome='landed', error_x=error_x, error_y=error_y,
                error=round((error_x ** 2 + error_y ** 2) ** 0.5, 4), **extra)


def test_percentile_is_nearest_rank():
    values = [0.01 * n for n in range(1, 21)]             # 1 cm .. 20 cm
    assert percentile(values, 0.95) == pytest.approx(0.19)
    assert percentile(values, 0.50) == pytest.approx(0.10)
    assert percentile([0.07], 0.95) == 0.07


def test_error_stats():
    stats = error_stats([0.03, 0.04, 0.05, 0.20])
    assert stats['n'] == 4
    assert stats['median'] == pytest.approx(0.045)
    assert stats['mean'] == pytest.approx(0.08)
    assert stats['max'] == 0.20
    assert error_stats([]) is None


def test_summary_of_two_batches(tmp_path):
    first = write(tmp_path / 'a.csv', [
        landed('a', 1, 0.03, 0.04, detector_range=5.2, true_height_above_pad=5.0, approach_time=20),
        landed('a', 2, -0.06, 0.08, detector_range=5.3, true_height_above_pad=5.0, approach_time=30),
    ])
    second = write(tmp_path / 'b.csv', [
        landed('b', 1, 0.55, 0.0),                         # off the marker, still on the pad
        dict(batch='b', trial=2, outcome='failed', reason='marker lost during the descent'),
    ])
    summary = summarise(load_rows([first, second]))
    assert summary['attempts'] == 4 and summary['landed'] == 3
    assert len(summary['failed']) == 1
    assert summary['on_marker'] == 2 and summary['on_pad'] == 3
    assert summary['error']['max'] == pytest.approx(0.55)
    assert summary['bias'][0] == pytest.approx((0.03 - 0.06 + 0.55) / 3)
    assert summary['range_ratio'][0] == pytest.approx((5.2 / 5.0 + 5.3 / 5.0) / 2)
    assert summary['approach_time'] == (25.0, 30.0)
    assert set(summary['batches']) == {'a', 'b'}
    assert summary['batches']['a']['n'] == 2

    report = to_markdown(summary)
    assert 'Attempts: 4, landed and measured: 3, failed: 1' in report
    assert 'marker lost during the descent' in report
    assert 'worst 55.0 cm' in report


def test_summary_with_no_successful_landing(tmp_path):
    path = write(tmp_path / 'c.csv', [dict(batch='c', trial=1, outcome='failed', reason='marker not found')])
    summary = summarise(load_rows([path]))
    assert summary['error'] is None
    assert 'No successful landings' in to_markdown(summary)
