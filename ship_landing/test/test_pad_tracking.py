"""Tests for following and landing on a moving pad, in the closed-loop world of moving_pad_sim.py.

The numbers these tests accept are loose on purpose. The world here is a rough model;
what the drone really achieves is measured in the simulator, not here.
"""
import math
import statistics

import pytest

from moving_pad_sim import World, fly
from ship_landing.levelling import PoseBuffer, pad_in_world
from ship_landing.pad_tracking import MovingPadLander, MovingPadParams, PadTrack

SEEDS = range(12)


def landings(**world):
    return [fly(World(seed=seed, **world)) for seed in SEEDS]


def feed(track, position_at, seconds, step=0.1, start=0.0):
    t = start
    while t < start + seconds:
        east, north = position_at(t)
        track.update(t, east, north, 0.12)
        t += step
    return t - step


# ---- the estimate ------------------------------------------------------------------------

def test_track_learns_a_steady_velocity():
    track = PadTrack()
    last = feed(track, lambda t: (4.0 + 0.5 * t, 3.0 - 0.2 * t), 5.0)
    assert track.ready
    assert track.velocity == pytest.approx([0.5, -0.2], abs=0.01)
    assert track.predict(last + 1.0) == pytest.approx((4.0 + 0.5 * (last + 1.0), 3.0 - 0.2 * (last + 1.0)), abs=0.02)
    assert track.unsteadiness() < 0.02


def test_track_keeps_up_with_a_pad_that_brakes():
    track = PadTrack()
    last = feed(track, lambda t: (0.5 * t - 0.125 * t * t, 0.0), 6.0)       # 0.25 m/s^2 of braking
    assert track.velocity[0] == pytest.approx(0.5 - 0.25 * last, abs=0.03)
    assert track.accel[0] == pytest.approx(-0.25, abs=0.03)


def test_track_notices_a_change_of_pace():
    track = PadTrack()
    last = feed(track, lambda t: (0.5 * t, 0.0), 5.0)
    assert track.unsteadiness() < 0.02
    feed(track, lambda t: (0.5 * t - 0.125 * (t - last) ** 2, 0.0), 0.8, start=last + 0.1)
    assert track.unsteadiness() > 0.06


def test_track_ignores_late_frames_and_restarts_after_a_long_gap():
    track = PadTrack()
    feed(track, lambda t: (t, 0.0), 2.0)
    before = (list(track.position), track.count)
    track.update(0.5, 99.0, 99.0, 0.0)                    # an old frame arriving late
    assert (track.position, track.count) == before
    track.update(10.0, 7.0, 7.0, 0.0)                     # seen again after a long gap
    assert track.count == 1 and not track.ready
    assert track.position == [7.0, 7.0] and track.velocity == [0.0, 0.0]
    assert math.isinf(PadTrack().age(1.0))


def test_track_survives_missing_frames():
    track = PadTrack()
    t = 0.0
    for i in range(80):
        t = 0.1 * i
        if i % 3 != 0:                                    # every third frame is missing
            track.update(t, 0.5 * t, 0.0, 0.12)
    assert track.velocity[0] == pytest.approx(0.5, abs=0.01)


def test_detection_is_placed_in_the_world():
    # Drone at (1, 2, 5) facing north and level; pad seen 3 m ahead, 1 m to the right, camera 4.9 m above it.
    north = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    east, north_, up = pad_in_world(1.0, -3.0, 4.9, (1.0, 2.0, 5.0), north, camera_down=0.10)
    assert (east, north_, up) == pytest.approx((2.0, 5.0, 0.0), abs=1e-9)


def test_pose_buffer_interpolates_position():
    buffer = PoseBuffer()
    q = (0.0, 0.0, 0.0, 1.0)
    buffer.add(1.0, (0.0, 0.0, 5.0), q)
    buffer.add(1.1, (0.2, 0.0, 5.0), q)
    position, _ = buffer.at(1.05)
    assert position == pytest.approx((0.1, 0.0, 5.0))
    assert buffer.at(3.0) is None
    assert buffer.latest()[0] == (0.2, 0.0, 5.0)


# ---- whole landings ----------------------------------------------------------------------

def test_lands_on_a_pad_that_stands_still():
    results = landings(pad_speed=0.0)
    assert all(r.state == 'LANDED' and r.on_pad for r in results)
    assert statistics.median(r.error for r in results) < 0.08


def test_lands_on_a_pad_moving_back_and_forth():
    results = landings(pattern='line', pad_speed=0.5)
    assert all(r.state == 'LANDED' and r.on_pad for r in results)
    assert statistics.median(r.error for r in results) < 0.12
    assert all(r.states[:4] == ('SEARCH', 'FOLLOW', 'DESCEND', 'FINAL') for r in results)


def test_lands_on_a_pad_moving_in_a_circle():
    results = landings(pattern='circle', pad_speed=0.5)
    assert all(r.state == 'LANDED' and r.on_pad for r in results)
    assert statistics.median(r.error for r in results) < 0.20


def test_lands_on_a_faster_pad():
    results = landings(pattern='line', pad_speed=1.0)
    assert all(r.state == 'LANDED' and r.on_pad for r in results)


@pytest.mark.parametrize('world', [
    dict(lag=0.6, command_delay=0.25),        # a more sluggish drone than assumed
    dict(speed_gain=0.85),                    # a drone that flies slower than commanded
    dict(frame_loss=0.4),                     # a camera that misses many frames
    dict(start=(0.0, 0.0, 8.0)),              # from higher up
])
def test_still_reaches_the_pad_when_the_world_is_not_as_assumed(world):
    results = landings(**world)
    assert all(r.state == 'LANDED' and r.on_pad for r in results)


def test_matches_the_pads_speed_at_touchdown():
    results = landings(pattern='line', pad_speed=0.5)
    assert statistics.median(r.relative_speed for r in results) < 0.15


def test_gives_up_when_the_pad_is_never_seen():
    result = fly(World(camera_stops_at=0.0))
    assert result.state == 'FAILED' and 'not found' in result.reason


def test_gives_up_when_the_camera_dies_on_the_way_down():
    result = fly(World(camera_stops_at=12.0))
    assert result.state == 'FAILED' and 'not seen' in result.reason


def test_a_camera_outage_on_the_way_down_does_not_stop_the_landing():
    for gap in ((9.0, 10.5), (12.0, 16.0)):               # a short one, and one long enough to lose the pad
        results = landings(camera_gap=gap)
        assert all(r.state == 'LANDED' and r.on_pad for r in results), gap


def test_on_the_pad_the_drone_asks_for_the_velocity_it_already_has():
    lander = MovingPadLander(MovingPadParams())
    t = 0.0
    for i in range(40):                                   # pad going east at 0.5 m/s, drone hovering
        t = 0.1 * i
        lander.update(t, (0.0, 0.0, 5.0), (t, 0.5 * t, 0.0, 0.0))
    lander._enter('CONTACT', t)
    # The pad has since turned round and carries the drone west at 0.3 m/s. The estimate
    # still says east; the command must follow the drone, not the estimate.
    command = None
    for i in range(1, 11):
        command = lander.update(t + 0.1 * i, (2.0 - 0.03 * i, 0.0, 0.3), None)
    assert command.east == pytest.approx(-0.3, abs=0.02)
    assert command.north == pytest.approx(0.0, abs=0.02)
    assert command.up < 0.0
