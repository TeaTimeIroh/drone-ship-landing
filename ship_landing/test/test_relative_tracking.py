"""The camera-only tracker against a made-up drone and pad whose true motion is known."""
import math
import random
import statistics

import pytest

from ship_landing.relative_tracking import G, RelativeParams, RelativeTrack, acceleration_from_lean
from test_levelling import NORTH, attitude_for


def fly(pad, feed_forward=True, noise=0.015, loss=0.2, delay=0.08, seconds=40.0, seed=1):
    """Run the tracker. Returns lists of position and velocity errors after the first 5 s."""
    rng = random.Random(seed)
    track = RelativeTrack(RelativeParams())
    dt = 0.02
    drone, velocity = [0.5, -0.3], [0.0, 0.0]
    position_errors, velocity_errors = [], []
    pending = []
    for step in range(int(seconds / dt)):
        t = step * dt
        # The drone weaves about as it would while chasing a pad.
        accel = (1.2 * math.sin(0.9 * t), 0.9 * math.cos(0.6 * t + 1.0))
        for i in (0, 1):
            drone[i] += velocity[i] * dt + 0.5 * accel[i] * dt * dt
            velocity[i] += accel[i] * dt
        pad_position, pad_velocity = pad(t)
        true_position = (pad_position[0] - drone[0], pad_position[1] - drone[1])
        true_velocity = (pad_velocity[0] - velocity[0], pad_velocity[1] - velocity[1])
        measured = acceleration_from_lean(attitude_for(accel[0], accel[1], NORTH))
        track.predict(t, measured if feed_forward else (0.0, 0.0))
        if step % 5 == 0 and rng.random() > loss:
            pending.append((t + delay, t, true_position[0] + rng.gauss(0, noise), true_position[1] + rng.gauss(0, noise)))
        while pending and pending[0][0] <= t:
            _, stamp, east, north = pending.pop(0)
            track.update(stamp, east, north, 3.0)
        if t > 5.0:
            position_errors.append(math.hypot(track.position[0] - true_position[0], track.position[1] - true_position[1]))
            velocity_errors.append(math.hypot(track.velocity[0] - true_velocity[0], track.velocity[1] - true_velocity[1]))
    return position_errors, velocity_errors


def still(t):
    return (2.0, 1.0), (0.0, 0.0)


def line(t):
    return (2.0 + 0.5 * t, 1.0), (0.5, 0.0)


def circle(t):
    w = 0.5 / 1.5
    return (1.5 * math.cos(w * t), 1.5 * math.sin(w * t)), (-0.5 * math.sin(w * t), 0.5 * math.cos(w * t))


def rms(values):
    return math.sqrt(statistics.fmean(v * v for v in values))


def test_lean_gives_the_acceleration():
    for accel in ((1.0, 0.0), (0.0, -2.0), (1.5, 0.7)):
        assert acceleration_from_lean(attitude_for(accel[0], accel[1], NORTH)) == pytest.approx(accel, abs=1e-6)
    assert acceleration_from_lean((0.0, 0.0, 0.0, 1.0)) == pytest.approx((0.0, 0.0))
    assert G == pytest.approx(9.81)


@pytest.mark.parametrize('pad, position_limit, velocity_limit', [
    (still, 0.03, 0.08), (line, 0.03, 0.08), (circle, 0.03, 0.08),
])
def test_follows_position_and_velocity(pad, position_limit, velocity_limit):
    position_errors, velocity_errors = fly(pad)
    assert rms(position_errors) < position_limit
    assert rms(velocity_errors) < velocity_limit


def test_knowing_the_lean_is_what_makes_the_velocity_usable():
    """Without the drone's own acceleration every lean looks like the pad changing speed."""
    _, with_lean = fly(line)
    _, without = fly(line, feed_forward=False)
    assert rms(without) > 3.0 * rms(with_lean)


def test_restarts_after_losing_the_pad():
    track = RelativeTrack()
    for i in range(10):
        track.predict(0.1 * i)
        track.update(0.1 * i, 1.0 + 0.05 * i, 0.0, 3.0)
    assert track.ready and track.velocity[0] > 0.2
    track.predict(5.0)
    track.update(5.0, -2.0, 0.5, 3.0)
    assert not track.ready
    assert track.position == [-2.0, 0.5] and track.velocity == [0.0, 0.0]
