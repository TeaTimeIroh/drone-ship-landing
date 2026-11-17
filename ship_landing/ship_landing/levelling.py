"""Turn what the tilted camera sees into where the pad is in a level frame. No ROS in here.

The camera is fixed to the drone's body. When the drone rolls or pitches, the camera
swings with it, and a pad that is straight below appears off to one side: at 5 m
height, 10 degrees of tilt moves it by almost 0.9 m. The drone's attitude is known,
so the measurement can be rotated back into a frame that stays level:

    x  forward along the drone's heading, horizontal
    y  left, horizontal
    z  up

Attitudes are quaternions (x, y, z, w) that rotate a vector from the drone's body
frame (x forward, y left, z up) into the world frame (x east, y north, z up), which is
what ArduPilot publishes on pose/filtered.
"""
import bisect
import math

from ship_landing.landing_control import PadOffset


def normalise(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2 w (u x v) + 2 u x (u x v), with u = (x, y, z)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2.0 * (w * cx + ccx), vy + 2.0 * (w * cy + ccy), vz + 2.0 * (w * cz + ccz))


def heading(q):
    """Direction the nose points, in radians from east towards north."""
    fx, fy, _ = rotate(q, (1.0, 0.0, 0.0))
    return math.atan2(fy, fx)


def tilt(q):
    """Angle between the drone's up axis and vertical, in radians."""
    _, _, uz = rotate(q, (0.0, 0.0, 1.0))
    return math.acos(max(-1.0, min(1.0, uz)))


def level(q, v_body):
    """Express a body-frame vector in the level frame that shares the drone's heading."""
    wx, wy, wz = rotate(q, v_body)
    yaw = heading(q)
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * wx + s * wy, -s * wx + c * wy, wz)


def camera_to_body(x, y, z):
    """Camera optical frame (x right in the image, y down in the image, z out of the lens)
    to the body frame, for the camera pointing straight down with image-up towards the nose."""
    return (-y, -x, -z)


def levelled_offset(x, y, z, q, camera_forward=0.0, camera_left=0.0, camera_down=0.0):
    """The pad's offset from the drone in the level frame.

    x, y, z is the detector's measurement in the camera frame, q the drone's attitude
    at the moment the image was taken. The camera_* values say where the camera sits
    relative to the drone's centre, in the body frame. `height` is the camera's height
    above the pad, measured vertically.
    """
    bx, by, bz = camera_to_body(x, y, z)
    _, _, up = level(q, (bx, by, bz))
    forward, left, _ = level(q, (bx + camera_forward, by + camera_left, bz - camera_down))
    return PadOffset(forward=forward, left=left, height=-up)


class AttitudeBuffer:
    """Recent attitudes by time, so the one matching an image's timestamp can be looked up."""

    def __init__(self, keep=3.0):
        self.keep = keep         # s of history
        self.times = []
        self.quats = []

    def add(self, t, q):
        if self.times and t <= self.times[-1]:
            if t < self.times[-1] - 1.0:      # time went backwards: the simulator restarted
                self.times, self.quats = [], []
            else:
                return
        self.times.append(t)
        self.quats.append(normalise(q))
        while self.times and self.times[0] < t - self.keep:
            self.times.pop(0)
            self.quats.pop(0)

    def at(self, t, max_gap=0.5):
        """Attitude at time t, interpolated. None if there is nothing within max_gap seconds."""
        if not self.times:
            return None
        i = bisect.bisect_left(self.times, t)
        if i == 0:
            return self.quats[0] if self.times[0] - t <= max_gap else None
        if i == len(self.times):
            return self.quats[-1] if t - self.times[-1] <= max_gap else None
        t0, t1 = self.times[i - 1], self.times[i]
        if t1 - t0 > max_gap:
            return None
        return blend(self.quats[i - 1], self.quats[i], (t - t0) / (t1 - t0))


def blend(q0, q1, fraction):
    """Interpolate between two nearby attitudes."""
    if sum(a * b for a, b in zip(q0, q1)) < 0.0:      # same rotation, opposite sign
        q1 = tuple(-c for c in q1)
    return normalise(tuple(a + fraction * (b - a) for a, b in zip(q0, q1)))


def pad_in_world(x, y, z, position, q, camera_forward=0.0, camera_left=0.0, camera_down=0.0):
    """Place a detection in the autopilot's local frame: (east, north, up) of the pad's surface.

    x, y, z is the detector's measurement in the camera frame; position (east, north, up)
    and q are the drone's pose at the moment the image was taken.
    """
    offset = levelled_offset(x, y, z, q, camera_forward, camera_left, camera_down)
    yaw = heading(q)
    c, s = math.cos(yaw), math.sin(yaw)
    return (position[0] + c * offset.forward - s * offset.left,
            position[1] + s * offset.forward + c * offset.left,
            position[2] - camera_down - offset.height)


class PoseBuffer:
    """Recent poses by time, so the one matching an image's timestamp can be looked up."""

    def __init__(self, keep=3.0):
        self.keep = keep
        self.times = []
        self.poses = []          # (position, quaternion)

    def add(self, t, position, q):
        if self.times and t <= self.times[-1]:
            if t < self.times[-1] - 1.0:      # time went backwards: the simulator restarted
                self.times, self.poses = [], []
            else:
                return
        self.times.append(t)
        self.poses.append((tuple(position), normalise(q)))
        while self.times and self.times[0] < t - self.keep:
            self.times.pop(0)
            self.poses.pop(0)

    def latest(self):
        return self.poses[-1] if self.poses else None

    def at(self, t, max_gap=0.5):
        """(position, quaternion) at time t, interpolated. None if nothing is within max_gap seconds."""
        if not self.times:
            return None
        i = bisect.bisect_left(self.times, t)
        if i == 0:
            return self.poses[0] if self.times[0] - t <= max_gap else None
        if i == len(self.times):
            return self.poses[-1] if t - self.times[-1] <= max_gap else None
        t0, t1 = self.times[i - 1], self.times[i]
        if t1 - t0 > max_gap:
            return None
        f = (t - t0) / (t1 - t0)
        (p0, q0), (p1, q1) = self.poses[i - 1], self.poses[i]
        return tuple(a + f * (b - a) for a, b in zip(p0, p1)), blend(q0, q1, f)
