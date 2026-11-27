"""Landing by leaning: the throttle map, the stages on the way down, and whole landings in the model."""
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(__file__))

from lean_hold_sim import Landing, circle, land, line, still  # noqa: E402

from ship_landing.lean_landing import LandingParams, LeanLander, ThrottleMap  # noqa: E402
from ship_landing.relative_tracking import RelativeTrack  # noqa: E402


def track_at(east, north, height, now=10.0):
    """A tracker that has just seen the pad at rest at this offset for a second."""
    track = RelativeTrack()
    for k in range(10):
        t = now - 0.9 + 0.1 * k
        track.predict(t, (0.0, 0.0))
        track.update(t, east, north, height)
    track.predict(now, (0.0, 0.0))
    return track


def test_throttle_stick_holds_height_inside_the_dead_zone_and_matches_the_measured_rate():
    throttle = ThrottleMap()
    assert throttle.stick(0.0) == 0.0
    assert abs(throttle.stick(0.33) - 0.3) < 0.01          # measured in step 5d-1: +0.3 gave 0.33 m/s
    assert abs(throttle.stick(-0.5) + 0.36) < 0.01
    assert throttle.stick(-10.0) == -1.0


def test_descends_only_inside_the_cone():
    lander = LeanLander()
    lander.state = 'DESCEND'
    inside = lander.update(10.0, track_at(0.10, 0.0, 3.0), math.pi / 2)
    assert inside.throttle < -0.2
    lander = LeanLander()
    lander.state = 'DESCEND'
    outside = lander.update(10.0, track_at(0.60, 0.0, 3.0), math.pi / 2)
    assert outside.throttle == 0.0
    assert outside.roll != 0.0 or outside.pitch != 0.0       # still steering towards the pad


def test_waits_above_the_pad_until_centred_then_commits():
    lander = LeanLander()
    lander.state, lander.state_since = 'WAIT', 10.0
    track = track_at(0.20, 0.0, 0.5)
    for k in range(11):
        lander.update(10.0 + 0.05 * k, track, math.pi / 2)
    assert lander.state == 'WAIT'
    track = track_at(0.01, 0.0, 0.5, now=12.0)
    for k in range(11):
        lander.update(12.0 + 0.05 * k, track, math.pi / 2)
    assert lander.state == 'COMMIT'


def test_contact_presses_down_gently_before_cutting_the_throttle():
    lander = LeanLander()
    lander.state, lander.state_since = 'COMMIT', 10.0
    lander.lowest_time = 10.0
    first = lander.update(10.0, track_at(0.0, 0.0, 0.22), math.pi / 2)
    assert lander.state == 'CONTACT'
    assert -0.5 < first.throttle < -0.2 and first.roll == 0.0 and first.pitch == 0.0
    assert lander.update(11.0, None, 0.0).throttle == -1.0


def test_pad_lost_high_up_means_climb_and_then_giving_up():
    lander = LeanLander()
    lander.state = 'DESCEND'
    track = track_at(0.0, 0.0, 3.0)
    track.predict(12.0, (0.0, 0.0))
    sticks = lander.update(12.0, track, math.pi / 2)
    assert lander.state == 'CLIMB' and sticks.throttle > 0.2 and sticks.roll == 0.0
    track.predict(19.0, (0.0, 0.0))
    lander.update(19.0, track, math.pi / 2)
    assert lander.state == 'FAILED'


def landings(pad, n=20, **changes):
    return [land(Landing(pad=pad, seed=seed, start=(pad(0.0)[0][0] + 2.5, pad(0.0)[0][1] - 1.5), **changes))
            for seed in range(n)]


def test_lands_on_a_still_pad_within_a_few_centimetres():
    results = landings(still)
    assert all(r.landed for r in results)
    assert max(r.error for r in results) < 0.05


def test_lands_on_the_moving_pad():
    for pad, median, worst in ((line, 0.05, 0.25), (circle, 0.08, 0.12)):
        results = landings(pad)
        assert all(r.landed for r in results)
        errors = [r.error for r in results]
        assert statistics.median(errors) < median and max(errors) < worst


def test_still_lands_when_the_stick_maps_are_off():
    results = landings(circle, true_slope=26.0, true_dead_zone=0.06, throttle_full_rate=2.0)
    assert all(r.landed for r in results)
    assert max(r.error for r in results) < 0.20
