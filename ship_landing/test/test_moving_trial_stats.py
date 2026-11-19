"""Tests for the moving-pad trial statistics. No ROS or simulator needed."""
import csv

import pytest

from ship_landing.moving_trial_stats import COLUMNS, load_rows, summarise, to_markdown, travel_frame


def row(**values):
    base = dict.fromkeys(COLUMNS, '')
    base.update(batch='batch1', seed=1, pattern='line', pad_speed=0.5, outcome='landed', land_mode=0)
    base.update(values)
    return base


def write(tmp_path, rows):
    path = tmp_path / 'batch_1.csv'
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    return path


def test_travel_frame_follows_the_pads_direction():
    # Pad going east: a drone east of the centre is ahead, one north of it is to the left.
    assert travel_frame(0.10, 0.02, (0.5, 0.0)) == pytest.approx((0.10, 0.02))
    # Pad going west: the same drone is behind, and to the right.
    assert travel_frame(0.10, 0.02, (-0.5, 0.0)) == pytest.approx((-0.10, -0.02))
    # Pad going north.
    assert travel_frame(0.10, 0.02, (0.0, 0.5)) == pytest.approx((0.02, -0.10))


def test_travel_frame_falls_back_when_the_pad_is_at_rest():
    assert travel_frame(0.10, 0.0, (0.01, 0.0), fallback=(-0.4, 0.0)) == pytest.approx((-0.10, 0.0))
    assert travel_frame(0.10, 0.0, (0.01, 0.0)) == (None, None)
    assert travel_frame(0.10, 0.0, None, None) == (None, None)


def test_summary_counts_and_splits(tmp_path):
    rows = [
        row(trial=1, error_east=0.03, error_north=0.0, error=0.03, along=-0.03, across=0.0,
            pad_velocity_change=0.01, estimated_east=0.02, estimated_north=0.0, track_time=17.0,
            final_attempts=1, stop_delay=1.7),
        row(trial=2, error_east=0.05, error_north=0.0, error=0.05, along=-0.05, across=0.0,
            pad_velocity_change=0.02, estimated_east=0.04, estimated_north=0.0, track_time=19.0,
            final_attempts=1, stop_delay=1.5),
        row(trial=3, error_east=-0.25, error_north=0.01, error=0.25, along=0.25, across=-0.01,
            pad_velocity_change=0.35, estimated_east=-0.05, estimated_north=0.0, track_time=25.0,
            final_attempts=2, stop_delay=1.9),
        row(trial=4, outcome='missed', reason='touched down off the pad', error_east=0.9, error_north=0.0, error=0.9),
        row(trial=5, outcome='failed', reason='the pad was not found'),
    ]
    summary = summarise(load_rows([write(tmp_path, rows)]))
    assert (summary['attempts'], summary['landed'], summary['missed'], summary['failed']) == (5, 3, 1, 1)
    assert summary['error']['median'] == pytest.approx(0.05)
    assert summary['error']['max'] == pytest.approx(0.25)          # the miss is not counted as a landing
    assert summary['over_15cm'] == 1
    assert summary['steady']['n'] == 2 and summary['steady']['worst'] == pytest.approx(0.05)
    assert summary['changing']['n'] == 1 and summary['changing']['median'] == pytest.approx(0.25)
    assert summary['bias_along'] == pytest.approx((-0.03 - 0.05 + 0.25) / 3)
    assert summary['track_time_max'] == pytest.approx(25.0)
    text = to_markdown(summary)
    assert 'landed on the pad and measured: 3' in text
    assert 'touched down off the pad: 1' in text
    assert 'line at 0.5 m/s' in text
    assert '| batch1 | 5 | 3 |' in text


def test_summary_of_a_run_without_landings(tmp_path):
    summary = summarise(load_rows([write(tmp_path, [row(trial=1, outcome='failed', reason='the pad was not found')])]))
    assert summary['error'] is None
    assert 'No landing was measured' in to_markdown(summary)
