"""Closed-loop checks of the pad-landing logic against a simple drone and camera model.

The model is deliberately pessimistic about the camera: it is fixed to the body, so it
tilts when the drone accelerates and the pad appears displaced, and it has delay, noise,
dropped frames and a limited field of view.
"""
import math
import random

import pytest

from ship_landing.landing_control import (
    Command, LandingParams, PadLander, PadOffset, descent_command, horizontal_command,
    inside_cone, offset_from_camera)

DT = 0.1                    # s, control and camera period
HALF_FOV = math.radians(50)
GRAVITY = 9.81


def test_camera_axes_map_to_drone_axes():
    # Seen in the simulator: pad to the right and towards the top of the image.
    offset = offset_from_camera(x=3.05, y=-2.04, z=5.3)
    assert offset.forward == pytest.approx(2.04)
    assert offset.left == pytest.approx(-3.05)
    assert offset.height == pytest.approx(5.3)


def test_camera_position_is_added_to_the_offset():
    # Camera 2 cm ahead of and 1 cm to the right of the drone's centre, pad straight below the camera:
    # seen from the drone's centre, the pad is 2 cm ahead and 1 cm to the right.
    offset = offset_from_camera(x=0.0, y=0.0, z=1.0, camera_forward=0.02, camera_left=-0.01)
    assert offset.forward == pytest.approx(0.02)
    assert offset.left == pytest.approx(-0.01)


def test_horizontal_speed_is_capped():
    cmd = horizontal_command(PadOffset(forward=30.0, left=-40.0, height=5.0), LandingParams(max_speed=1.0))
    assert math.hypot(cmd.forward, cmd.left) == pytest.approx(1.0)
    assert cmd.forward > 0 and cmd.left < 0 and cmd.up == 0.0


def test_descends_only_inside_the_cone():
    params = LandingParams()
    centred, off_centre = PadOffset(0.1, 0.0, 3.0), PadOffset(1.0, 0.0, 3.0)
    assert inside_cone(centred, params) and not inside_cone(off_centre, params)
    assert descent_command(centred, params).up < 0
    assert descent_command(off_centre, params).up == 0.0


def simulate(start, seed=0, tilt_coupling=True, dropout=0.1, noise=0.02, delay_steps=2,
             blackout=None, pad_visible=True, max_time=180.0):
    """Fly the lander from `start` = (pad ahead, pad left, height) and return what happened."""
    rng = random.Random(seed)
    lander = PadLander()
    ahead, left, height = start          # true offset of the pad from the drone
    v_fwd = v_left = v_up = 0.0
    history, t, trail = [], 0.0, []
    while not lander.done and t < max_time:
        # --- camera -------------------------------------------------------------------
        a_fwd = a_left = 0.0
        if trail:
            a_fwd, a_left = (v_fwd - trail[-1][0]) / DT, (v_left - trail[-1][1]) / DT
        trail.append((v_fwd, v_left))
        seen_ahead, seen_left = ahead, left
        if tilt_coupling:
            # Accelerating forward pitches the nose down; the camera then looks ahead of
            # the point below the drone, so the pad appears less far ahead.
            seen_ahead = height * math.tan(math.atan2(ahead, height) - a_fwd / GRAVITY)
            seen_left = height * math.tan(math.atan2(left, height) - a_left / GRAVITY)
        in_view = (pad_visible and height > 0.6
                   and abs(math.atan2(seen_ahead, height)) < HALF_FOV
                   and abs(math.atan2(seen_left, height)) < HALF_FOV)
        blacked_out = blackout is not None and blackout[0] <= t < blackout[1]
        frame = None
        if in_view and not blacked_out and rng.random() > dropout:
            scale = noise * max(height, 1.0) / 5.0
            frame = PadOffset(seen_ahead + rng.gauss(0, scale), seen_left + rng.gauss(0, scale),
                              height * (1.0 + rng.gauss(0, 0.01)))
        history.append(frame)
        delayed = history[-1 - delay_steps] if len(history) > delay_steps else None

        # --- controller ---------------------------------------------------------------
        cmd = lander.update(t, delayed)

        # --- drone: velocity follows the command with a 0.5 s lag ------------------------
        blend = DT / 0.5
        v_fwd += (cmd.forward - v_fwd) * blend
        v_left += (cmd.left - v_left) * blend
        v_up += (cmd.up - v_up) * blend
        ahead -= v_fwd * DT
        left -= v_left * DT
        height += v_up * DT
        t += DT
    return {'state': lander.state, 'reason': lander.reason, 'time': t,
            'error': math.hypot(ahead, left), 'height': height, 'speed': math.hypot(v_fwd, v_left)}


@pytest.mark.parametrize('start', [(2.0, -3.0, 5.0), (-4.0, 3.0, 8.0), (0.2, 0.1, 5.0), (5.0, 0.0, 6.0)])
@pytest.mark.parametrize('seed', [0, 1, 2])
def test_reaches_handover_centred_over_the_pad(start, seed):
    result = simulate(start, seed=seed)
    assert result['state'] == 'HANDOVER', result
    assert result['error'] < 0.25, result
    assert result['height'] < 1.6, result
    assert result['speed'] < 0.3, result
    assert result['time'] < 90, result


def test_survives_a_two_second_camera_blackout():
    result = simulate((2.0, -3.0, 5.0), blackout=(6.0, 8.0))
    assert result['state'] == 'HANDOVER' and result['error'] < 0.25, result


def test_gives_up_when_the_marker_never_appears():
    result = simulate((2.0, -3.0, 5.0), pad_visible=False)
    assert result['state'] == 'FAILED' and result['reason'] == 'marker not found'
    assert result['time'] < 25


def test_gives_up_when_the_marker_disappears_for_good():
    result = simulate((2.0, -3.0, 6.0), blackout=(14.0, 1e9))
    assert result['state'] == 'FAILED', result
    assert result['height'] > 1.5       # it did not keep descending blind


def test_no_command_once_finished():
    lander = PadLander()
    lander.update(0.0, PadOffset(0.0, 0.0, 0.5))
    for step in range(1, 30):
        cmd = lander.update(step * DT, PadOffset(0.0, 0.0, 0.5))
    assert lander.state == 'HANDOVER'
    assert cmd == Command()
