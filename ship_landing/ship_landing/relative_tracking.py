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
    # A detection's error grows with the distance to the pad, so close to it the tracker can
    # afford to believe each one more and notice a change in the pad's motion sooner. The three
    # shares above apply from `far_height` up, the three below from `near_height` down.
    near_alpha: float = 0.45
    near_beta: float = 0.20
    near_gamma: float = 0.025
    near_height: float = 1.0      # m
    far_height: float = 4.0       # m
    frame_time: float = 0.1       # s between camera frames
    max_pad_accel: float = 1.0    # m/s^2, the pad's acceleration is kept within this
    height_alpha: float = 0.4     # the same two shares for the height and its rate of change
    height_beta: float = 0.10
    max_climb: float = 3.0        # m/s
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
        self.climb = 0.0              # its rate of change, m/s (negative coming down)
        self.last_seen = None
        self.count = 0                # detections since the last restart
        self.history = deque()        # (time, east, north, height) of past states, to match late detections

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
        if self.height is not None:
            self.height += self.climb * dt
        self.time = now
        self.history.append((now, self.position[0], self.position[1], self.height))
        while self.history and self.history[0][0] < now - 1.0:
            self.history.popleft()

    def _state_at(self, t):
        """The position the tracker held at time t, from its history."""
        if not self.history or t >= self.history[-1][0]:
            return self.position[0], self.position[1], self.height
        previous = self.history[0]
        for entry in self.history:
            if entry[0] >= t:
                span = entry[0] - previous[0]
                w = (t - previous[0]) / span if span > 0 else 0.0
                return tuple(previous[k] + w * (entry[k] - previous[k]) for k in (1, 2, 3))
            previous = entry
        return previous[1], previous[2], previous[3]

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
            self.history.append((stamp, east, north, height))
            return
        # Compare the detection with what the tracker believed at the moment of the
        # image, and apply the difference to the state as it is now.
        believed = self._state_at(stamp)
        surprise = (east - believed[0], north - believed[1])
        alpha, beta, gamma = self.gains(height)
        for i in (0, 1):
            self.position[i] += alpha * surprise[i]
            self.velocity[i] += beta / p.frame_time * surprise[i]
            self.pad_accel[i] += gamma / (p.frame_time * p.frame_time) * surprise[i]
            self.pad_accel[i] = max(-p.max_pad_accel, min(p.max_pad_accel, self.pad_accel[i]))
        rise = height - believed[2]
        self.height += p.height_alpha * rise
        self.climb += p.height_beta / p.frame_time * rise
        self.climb = max(-p.max_climb, min(p.max_climb, self.climb))
        self.history = deque((t, e + alpha * surprise[0], n + alpha * surprise[1], h + p.height_alpha * rise)
                             for t, e, n, h in self.history)
        self.last_seen = stamp
        self.count += 1

    def gains(self, height):
        """(alpha, beta, gamma) for a detection made from this height."""
        p = self.params
        w = max(0.0, min(1.0, (p.far_height - height) / (p.far_height - p.near_height)))
        return (p.alpha + w * (p.near_alpha - p.alpha), p.beta + w * (p.near_beta - p.beta),
                p.gamma + w * (p.near_gamma - p.gamma))

    def speed(self):
        return math.hypot(*self.velocity)
