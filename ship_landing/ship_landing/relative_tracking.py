"""Where the pad is and how it moves, relative to the drone, from the camera alone. No ROS.

The moving-pad landing so far placed every detection in the autopilot's local frame
and tracked the pad there. That leans on the autopilot knowing its own position and
speed. This tracker does without: its state is the pad's position and velocity
*relative to the drone*, in axes that point east and north.

Two things feed it:

- detections, already rotated by the drone's attitude (levelling.py), a few times a
  second and a little late;
- the drone's own horizontal acceleration, read off its lean: a drone that holds its
  height and leans by an angle accelerates at g times the tangent of that angle. The
  lean comes from the inertial sensors and needs no position.

Between detections the relative velocity changes by the pad's acceleration minus
the drone's. Each detection then pulls position and velocity towards what was seen,
and a small share of the surprise goes into the pad's acceleration, which is the
third thing the tracker keeps (an alpha-beta-gamma filter). Knowing the drone's own
acceleration matters: without it every lean the drone makes would first look like
the pad speeding up. The pad's acceleration matters for a pad that turns: to stay
over it the drone has to accelerate with it, and this is the only place that
number can come from.
"""
import math
from collections import deque
from dataclasses import dataclass

from ship_landing.levelling import rotate

G = 9.81


def acceleration_from_lean(q):
    """Horizontal acceleration (east, north) in m/s^2 of a drone holding its height with attitude q."""
    up = rotate(q, (0.0, 0.0, 1.0))          # the direction the rotors push, in the world
    if up[2] < 0.2:
        return (0.0, 0.0)                    # on its side or worse: no sensible answer
    return (G * up[0] / up[2], G * up[1] / up[2])


@dataclass
class RelativeParams:
    alpha: float = 0.35           # share of a detection's surprise that goes into the position
    beta: float = 0.12            # share that goes into the velocity, per frame time
    gamma: float = 0.012          # share that goes into the pad's acceleration, per frame time squared
    frame_time: float = 0.1       # s between camera frames
    max_pad_accel: float = 1.0    # m/s^2, the pad's acceleration is kept within this
    height_smoothing: float = 0.6
    restart_after: float = 1.5    # s without a detection after which the velocity is no longer trusted
    max_speed: float = 4.0        # m/s, relative speeds beyond this are clipped


class RelativeTrack:
    """Pad position and velocity relative to the drone: east, north, and height above the pad."""

    def __init__(self, params=None):
        self.params = params or RelativeParams()
        self.reset()

    def reset(self):
        self.time = None              # time the state refers to
        self.position = [0.0, 0.0]    # pad minus drone, m
        self.velocity = [0.0, 0.0]    # pad minus drone, m/s
        self.pad_accel = [0.0, 0.0]   # the pad's own acceleration, m/s^2
        self.height = None            # drone centre above the pad's surface, m
        self.last_seen = None
        self.count = 0                # detections since the last restart
        self.history = deque()        # (time, east, north) of past states, to match late detections

    @property
    def ready(self):
        return self.count >= 5

    def age(self, now):
        """Seconds since the pad was last seen."""
        return float('inf') if self.last_seen is None else now - self.last_seen

    def predict(self, now, drone_accel=(0.0, 0.0)):
        """Move the state on to `now`, given the drone's own horizontal acceleration."""
        if self.time is None:
            return
        dt = now - self.time
        if dt <= 0.0:
            return
        dt = min(dt, 0.5)
        for i in (0, 1):
            relative_accel = self.pad_accel[i] - drone_accel[i]
            self.position[i] += self.velocity[i] * dt + 0.5 * relative_accel * dt * dt
            self.velocity[i] += relative_accel * dt
            self.velocity[i] = max(-self.params.max_speed, min(self.params.max_speed, self.velocity[i]))
        self.time = now
        self.history.append((now, self.position[0], self.position[1]))
        while self.history and self.history[0][0] < now - 1.0:
            self.history.popleft()

    def _state_at(self, t):
        """The position the tracker held at time t, from its history."""
        if not self.history or t >= self.history[-1][0]:
            return self.position[0], self.position[1]
        previous = self.history[0]
        for entry in self.history:
            if entry[0] >= t:
                span = entry[0] - previous[0]
                w = (t - previous[0]) / span if span > 0 else 0.0
                return (previous[1] + w * (entry[1] - previous[1]), previous[2] + w * (entry[2] - previous[2]))
            previous = entry
        return previous[1], previous[2]

    def update(self, stamp, east, north, height):
        """Take in a detection made at time `stamp`: the pad seen (east, north) of the drone."""
        p = self.params
        if self.time is None or self.age(stamp) > p.restart_after:
            self.reset()
            self.time = stamp
            self.position = [east, north]
            self.height = height
            self.last_seen = stamp
            self.count = 1
            self.history.append((stamp, east, north))
            return
        # Compare the detection with what the tracker believed at the moment of the
        # image, and apply the difference to the state as it is now.
        believed = self._state_at(stamp)
        surprise = (east - believed[0], north - believed[1])
        for i in (0, 1):
            self.position[i] += p.alpha * surprise[i]
            self.velocity[i] += p.beta / p.frame_time * surprise[i]
            self.pad_accel[i] += p.gamma / (p.frame_time * p.frame_time) * surprise[i]
            self.pad_accel[i] = max(-p.max_pad_accel, min(p.max_pad_accel, self.pad_accel[i]))
        self.history = deque((t, e + p.alpha * surprise[0], n + p.alpha * surprise[1]) for t, e, n in self.history)
        self.height = height if self.height is None else (
            p.height_smoothing * self.height + (1.0 - p.height_smoothing) * height)
        self.last_seen = stamp
        self.count += 1

    def speed(self):
        return math.hypot(*self.velocity)
