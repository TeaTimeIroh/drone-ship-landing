"""The lean controller: its sign conventions, and holding over a pad in the model."""
import math
import statistics

import pytest

from lean_hold_sim import Sim, circle, line, run, still
from ship_landing.lean_control import PadHold, StickMap
from ship_landing.relative_tracking import RelativeTrack

NORTH = math.pi / 2.0


def track_at(east, north, velocity=(0.0, 0.0)):
    track = RelativeTrack()
    track.update(0.0, east, north, 3.0)
    track.velocity = list(velocity)
    return track


def test_stick_map_matches_the_calibration():
    sticks = StickMap()
    assert sticks.stick(0.0) == 0.0
    # Measured: a 0.20 stick gave 5.0 degrees, a 0.40 stick 11.3 to 11.4 degrees.
    assert sticks.stick(5.05) == pytest.approx(0.20, abs=0.005)
    assert sticks.stick(-11.37) == pytest.approx(-0.40, abs=0.005)


def test_leans_towards_the_pad():
    """Facing north: a pad to the east needs the right side down, a pad to the north the nose down."""
    hold = PadHold()
    east = hold.command(track_at(2.0, 0.0), NORTH)
    assert east.roll_stick > 0.05 and abs(east.pitch_stick) < 1e-6 and east.lean_left < 0
    north = hold.command(track_at(0.0, 2.0), NORTH)
    assert north.pitch_stick < -0.05 and abs(north.roll_stick) < 1e-6 and north.lean_forward > 0
    # Facing east instead, the pad to the east is straight ahead.
    ahead = hold.command(track_at(2.0, 0.0), 0.0)
    assert ahead.pitch_stick < -0.05 and abs(ahead.roll_stick) < 1e-6


def test_brakes_when_the_gap_is_closing_too_fast():
    """Over the pad already, but the pad still rushing west relative to the drone: lean west."""
    command = PadHold().command(track_at(0.0, 0.0, velocity=(-1.0, 0.0)), NORTH)
    assert command.roll_stick < -0.05


def test_lean_is_limited():
    command = PadHold().command(track_at(50.0, -50.0), NORTH)
    assert abs(command.lean_forward) <= 10.0 + 1e-9 and abs(command.lean_left) <= 10.0 + 1e-9


def held(pad, seeds=6, **kw):
    """(median, worst) distance from the pad centre once the drone has arrived, and the arrival time."""
    medians, worsts, arrivals = [], [], []
    for seed in range(seeds):
        out = run(Sim(pad=pad, seed=seed, noise_per_metre=0.003, **kw))
        first = next((t for t, d, _ in out if d < 0.3), None)
        assert first is not None, 'never came within 0.3 m of the pad'
        tail = [d for t, d, _ in out if t > first + 3.0]
        medians.append(statistics.median(tail))
        worsts.append(max(tail))
        arrivals.append(first)
    return statistics.fmean(medians), max(worsts), max(arrivals)


def test_holds_over_a_still_pad():
    median, worst, arrival = held(still, height=2.0)
    assert arrival < 8.0 and median < 0.03 and worst < 0.10


def test_holds_over_a_still_pad_from_high_up():
    """At 6 m each detection is noisier, so the hold is looser, but it must stay a hold."""
    median, worst, _ = held(still, height=6.0)
    assert median < 0.08 and worst < 0.25


def test_stays_with_a_pad_on_a_line():
    median, worst, _ = held(line, height=2.0)
    assert median < 0.06 and worst < 0.35       # the worst moments are the turn-arounds


def test_stays_with_a_pad_on_a_circle():
    median, worst, _ = held(circle, height=2.0)
    assert median < 0.08 and worst < 0.20


def test_tolerates_a_slower_drone_and_a_wrong_stick_map():
    _, worst, _ = held(circle, height=2.0, lean_lag=0.4, stick_delay=0.15)
    assert worst < 0.35
    _, worst, _ = held(circle, height=2.0, true_slope=26.0, true_dead_zone=0.06)
    assert worst < 0.35
