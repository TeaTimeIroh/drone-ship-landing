"""Tests for the moving pad's path generator. No ROS or simulator needed."""
import math

import pytest

from ship_landing.pad_motion import MotionParams, PadMotion

DT = 0.05


def run(motion, seconds, follow=True, start=(0.0, 0.0)):
    """Step the generator and integrate its commands as a perfect pad would.

    Returns a list of (time, east, north, v_east, v_north).
    """
    east, north = start
    track = []
    for i in range(int(seconds / DT)):
        v_east, v_north = motion.step(DT, (east, north) if follow else None)
        east += v_east * DT
        north += v_north * DT
        track.append(((i + 1) * DT, east, north, v_east, v_north))
    return track


def test_line_stays_inside_its_travel_and_reaches_both_ends():
    track = run(PadMotion(MotionParams(pattern='line', speed=0.5, travel=2.5)), 60.0)
    easts = [row[1] for row in track]
    assert max(easts) == pytest.approx(2.5, abs=0.05)
    assert min(easts) == pytest.approx(-2.5, abs=0.05)
    assert all(abs(row[2]) < 1e-9 for row in track)        # no sideways motion


def test_line_cruises_at_the_requested_speed():
    track = run(PadMotion(MotionParams(pattern='line', speed=0.5, travel=2.5)), 60.0)
    speeds = [math.hypot(row[3], row[4]) for row in track]
    assert max(speeds) <= 0.5 + 1e-6
    cruising = sum(1 for s in speeds if s > 0.499)
    assert cruising > 0.5 * len(speeds)                    # most of the time at full speed


def test_speed_changes_are_gentle():
    params = MotionParams(pattern='line', speed=0.5, travel=2.5, accel=0.25)
    track = run(PadMotion(params), 60.0, follow=False)
    for before, after in zip(track, track[1:]):
        change = math.hypot(after[3] - before[3], after[4] - before[4])
        assert change <= params.accel * DT + 1e-9


def test_line_heading_turns_the_line():
    track = run(PadMotion(MotionParams(pattern='line', heading_deg=90.0)), 30.0)
    assert all(abs(row[1]) < 1e-6 for row in track)        # nothing east-west
    assert max(row[2] for row in track) > 2.0              # it does go north


def test_circle_keeps_its_radius_and_speed():
    params = MotionParams(pattern='circle', speed=0.5, radius=1.5)
    track = run(PadMotion(params), 60.0)
    for _, east, north, v_east, v_north in track:
        assert math.hypot(east, north + 1.5) == pytest.approx(1.5, abs=0.02)   # centre is 1.5 m south
    late = [math.hypot(row[3], row[4]) for row in track if row[0] > 5.0]
    assert all(s == pytest.approx(0.5, abs=0.01) for s in late)
    assert min(row[2] for row in track) == pytest.approx(-3.0, abs=0.05)       # goes all the way round


def test_zero_speed_keeps_the_pad_still():
    for pattern in ('line', 'circle'):
        track = run(PadMotion(MotionParams(pattern=pattern, speed=0.0)), 5.0)
        assert all(abs(row[1]) < 1e-9 and abs(row[2]) < 1e-9 for row in track)


def test_speed_can_be_changed_while_running():
    motion = PadMotion(MotionParams(pattern='line', speed=0.5, travel=2.5))
    run(motion, 3.0)
    motion.params.speed = 0.0
    track = run(motion, 10.0, follow=False)
    assert math.hypot(track[-1][3], track[-1][4]) == pytest.approx(0.0, abs=1e-9)
    motion.params.speed = 1.0
    track = run(motion, 10.0, follow=False)
    assert max(math.hypot(row[3], row[4]) for row in track) == pytest.approx(1.0, abs=1e-6)


def test_faster_line_still_turns_inside_its_travel():
    params = MotionParams(pattern='line', speed=1.5, travel=2.5, accel=0.5)
    track = run(PadMotion(params), 60.0)
    assert max(abs(row[1]) for row in track) <= 2.5 + 0.1


def test_measured_position_pulls_the_pad_back_to_the_path():
    # The pad starts half a metre off the line; the correction brings it back.
    motion = PadMotion(MotionParams(pattern='line', speed=0.5, travel=2.5))
    track = run(motion, 20.0, start=(0.0, 0.5))
    assert abs(track[-1][2]) < 0.01


def test_correction_is_limited():
    motion = PadMotion(MotionParams(pattern='line', speed=0.0))
    v_east, v_north = motion.step(DT, measured=(0.0, 3.0))
    assert math.hypot(v_east, v_north) == pytest.approx(0.2, abs=1e-9)


def test_unknown_pattern_is_rejected():
    with pytest.raises(ValueError):
        PadMotion(MotionParams(pattern='zigzag'))
