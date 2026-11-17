"""Tests for the tilt correction and for the analysis that checks it on a flight log.

The synthetic flight here is built from physics, not from the code under test: the
drone leans in the direction it accelerates, the camera is bolted to it, and the pad's
position in the image follows from that geometry.
"""
import math

import pytest

from ship_landing.landing_control import offset_from_camera
from ship_landing.levelling import AttitudeBuffer, heading, level, levelled_offset, rotate, tilt
from ship_landing.tilt_analysis import (
    Detection, Geometry, Pose, analyse, euler, from_euler, to_markdown, variant_quat, verdict)

G = 9.81
NORTH = math.pi / 2


def quat_from_axes(x_axis, y_axis, z_axis):
    """Quaternion (x, y, z, w) of the rotation whose columns are the given body axes in the world."""
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    w = math.sqrt(max(0.0, 1.0 + m00 + m11 + m22)) / 2.0
    return ((m21 - m12) / (4.0 * w), (m02 - m20) / (4.0 * w), (m10 - m01) / (4.0 * w), w)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def attitude_for(accel_east, accel_north, yaw=NORTH):
    """Attitude of a multirotor accelerating horizontally: its up axis leans into the acceleration."""
    up = unit((accel_east, accel_north, G))
    nose = (math.cos(yaw), math.sin(yaw), 0.0)
    left = unit(cross(up, nose))
    forward = cross(left, up)
    return quat_from_axes(forward, left, up)


def camera_view(position, q, pad, camera_body):
    """What a downward body-fixed camera measures: the pad in the camera optical frame."""
    lens = tuple(p + c for p, c in zip(position, rotate(q, camera_body)))
    to_pad = tuple(a - b for a, b in zip(pad, lens))
    inverse = (-q[0], -q[1], -q[2], q[3])
    bx, by, bz = rotate(inverse, to_pad)
    return (-by, -bx, -bz)          # x right in the image, y down in the image, z out of the lens


def synthetic_flight(geometry, yaw=NORTH, image_delay=0.0, flip_roll_in_log=False):
    """A drone swinging east-west then north-south near the pad at 5 m, logged as tilt_check would."""
    pad = (geometry.pad_east, geometry.pad_north, 0.0)
    camera_body = (geometry.camera_forward, geometry.camera_left, -geometry.camera_down)

    def state(t):
        amplitude, omega = 1.5, 2.0 * math.pi / 5.0      # whole swings in each 20 s leg
        if t < 20.0:
            offset = (amplitude * math.sin(omega * t), 0.0)
            accel = (-amplitude * omega ** 2 * math.sin(omega * t), 0.0)
        else:
            offset = (0.0, amplitude * math.sin(omega * t))
            accel = (0.0, -amplitude * omega ** 2 * math.sin(omega * t))
        # Centred 1.5 m west and 1 m south of the pad, as tilt_check flies it.
        position = (pad[0] - 1.5 + offset[0], pad[1] - 1.0 + offset[1], 5.0)
        return position, attitude_for(accel[0], accel[1], yaw)

    poses, detections = [], []
    for i in range(int(40.0 / 0.033)):
        t = i * 0.033
        position, q = state(t)
        if flip_roll_in_log:
            q = variant_quat(q, 'roll reversed')
        poses.append(Pose(t, position, q))
    for i in range(5, 395):
        t = i * 0.1
        position, q = state(t)
        x, y, z = camera_view(position, q, pad, camera_body)
        detections.append(Detection(t + image_delay, x, y, z))
    return detections, poses


# ---- levelling ---------------------------------------------------------------------------

def test_level_attitude_gives_the_same_answer_as_before():
    q = attitude_for(0.0, 0.0)
    new = levelled_offset(0.4, -1.2, 5.0, q, 0.02, -0.01, 0.10)
    old = offset_from_camera(0.4, -1.2, 5.0, 0.02, -0.01)
    assert new.forward == pytest.approx(old.forward, abs=1e-9)
    assert new.left == pytest.approx(old.left, abs=1e-9)
    assert new.height == pytest.approx(5.0, abs=1e-9)


@pytest.mark.parametrize('accel', [(2.0, 0.0), (-2.0, 0.0), (0.0, 2.0), (0.0, -2.0), (1.5, -1.0)])
@pytest.mark.parametrize('yaw', [NORTH, 0.0, 2.5])
def test_pad_straight_below_a_tilted_drone_is_reported_straight_below(accel, yaw):
    q = attitude_for(accel[0], accel[1], yaw)
    assert math.degrees(tilt(q)) > 5.0
    x, y, z = camera_view((3.0, 2.0, 5.0), q, (3.0, 2.0, 0.0), (0.0, 0.0, 0.0))
    raw = offset_from_camera(x, y, z)
    assert raw.horizontal > 0.4                      # uncorrected, the pad looks far off
    fixed = levelled_offset(x, y, z, q)
    assert fixed.horizontal < 1e-6
    assert fixed.height == pytest.approx(5.0, abs=1e-6)


def test_levelled_offset_matches_the_world_for_an_offset_pad():
    # Drone facing north at (0, 0, 5), accelerating east; pad 2 m north and 1 m east.
    q = attitude_for(2.5, 0.0)
    camera_body = (0.02, -0.01, -0.10)
    x, y, z = camera_view((0.0, 0.0, 5.0), q, (1.0, 2.0, 0.0), camera_body)
    fixed = levelled_offset(x, y, z, q, 0.02, -0.01, 0.10)
    assert fixed.forward == pytest.approx(2.0, abs=1e-6)     # north is ahead
    assert fixed.left == pytest.approx(-1.0, abs=1e-6)       # east is to the right


def test_heading_and_level_frame():
    q = attitude_for(0.0, 0.0, yaw=NORTH)
    assert heading(q) == pytest.approx(NORTH)
    assert level(q, (1.0, 0.0, 0.0)) == pytest.approx((1.0, 0.0, 0.0))


def test_euler_round_trip():
    q = attitude_for(1.5, -1.0, yaw=2.5)
    again = from_euler(*euler(q))
    assert all(a == pytest.approx(b, abs=1e-9) for a, b in zip(q, again))


def test_attitude_buffer_interpolates_and_forgets():
    buffer = AttitudeBuffer(keep=2.0)
    flat, leaning = attitude_for(0.0, 0.0), attitude_for(2.0, 0.0)
    buffer.add(10.0, flat)
    buffer.add(10.1, leaning)
    middle = buffer.at(10.05)
    assert tilt(middle) == pytest.approx(tilt(leaning) / 2.0, rel=0.02)
    assert buffer.at(10.3) is not None               # just after the newest sample: use it
    assert buffer.at(11.0) is None                   # too long after: no answer
    assert buffer.at(5.0) is None
    buffer.add(13.0, flat)
    assert buffer.times[0] >= 11.0                   # old samples dropped
    assert AttitudeBuffer().at(1.0) is None


# ---- analysis of a flight log ------------------------------------------------------------

def test_analysis_confirms_a_correct_convention():
    geometry = Geometry()
    result = analyse(*synthetic_flight(geometry), geometry)
    assert result.best_variant == 'as published'
    assert result.tilted_samples > 100
    assert result.rms_raw_tilted > 0.5
    assert result.rms_tilted['as published'] < 0.01
    assert result.rms_tilted['roll reversed'] > result.rms_raw_tilted * 0.9
    assert result.heading_autopilot_deg == pytest.approx(90.0, abs=0.1)
    assert result.heading_camera_deg == pytest.approx(90.0, abs=2.0)
    assert 'is the best variant' in verdict(result)


def test_analysis_catches_a_reversed_roll():
    geometry = Geometry()
    result = analyse(*synthetic_flight(geometry, flip_roll_in_log=True), geometry)
    assert result.best_variant == 'roll reversed'
    assert 'NOT the best variant' in verdict(result)


def test_analysis_finds_a_clock_offset():
    geometry = Geometry()
    result = analyse(*synthetic_flight(geometry, image_delay=0.10), geometry)
    assert result.best_shift == pytest.approx(-0.10, abs=0.021)
    assert result.rms_at_best_shift < 0.02
    assert 'clocks are not perfectly aligned' in verdict(result)


def test_analysis_works_at_another_heading():
    geometry = Geometry()
    result = analyse(*synthetic_flight(geometry, yaw=0.3), geometry)
    assert result.best_variant == 'as published'
    assert result.heading_camera_deg == pytest.approx(math.degrees(0.3), abs=2.0)


def test_report_is_written_for_empty_and_full_logs():
    assert 'Nothing to conclude' in to_markdown(analyse([], []))
    geometry = Geometry()
    text = to_markdown(analyse(*synthetic_flight(geometry), geometry))
    assert '| levelled, as published |' in text and '| raw, tilt ignored |' in text
