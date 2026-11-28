"""Where the moving pad should be and how fast it should go. No ROS in here.

A PadMotion keeps a reference point that travels along a path, and turns it into the
east and north speeds to command. Positions are in metres from the pad's start point.

    line    back and forth through the start point, `travel` metres each way, at a
            steady `speed` with gentle turn-arounds. `heading_deg` is the direction of
            the line: 0 is east-west, 90 is north-south.
    circle  a circle of `radius` metres that starts at the start point heading east and
            curves to the south, at a steady `speed`.
    straight  straight ahead along `heading_deg` without end, at a steady `speed`: a ship
            on its course. Changing the speed speeds it up or slows it down at `accel`.

With speed 0 the pad stays where it is. The speed may be changed while running; the
pattern and its size may not.
"""
import math
from dataclasses import dataclass

PATTERNS = ('line', 'circle', 'straight')


@dataclass
class MotionParams:
    pattern: str = 'line'
    speed: float = 0.5           # m/s along the path
    travel: float = 2.5          # m, line: distance each way from the start point
    radius: float = 1.5          # m, circle
    accel: float = 0.25          # m/s^2, how quickly the speed along the path changes
    heading_deg: float = 0.0     # line direction, 0 = east, 90 = north
    correction_gain: float = 1.0     # 1/s, pull towards the reference when the true position is known
    max_correction: float = 0.2      # m/s, limit on that pull


def _towards(value, target, step):
    """Move value towards target by at most step."""
    if target > value:
        return min(target, value + step)
    return max(target, value - step)


class PadMotion:
    def __init__(self, params=None):
        self.params = params or MotionParams()
        if self.params.pattern not in PATTERNS:
            raise ValueError(f'unknown pattern {self.params.pattern!r}, expected one of {PATTERNS}')
        if self.params.accel <= 0.0:
            raise ValueError('accel must be positive')
        if self.params.pattern == 'circle' and self.params.radius <= 0.0:
            raise ValueError('radius must be positive')
        self.along = 0.0         # line: signed distance from the start point; circle: distance travelled
        self.rate = 0.0          # speed along the path; signed for the line
        self.direction = 1.0     # line: which way the pad is currently sent
        self.reference = (0.0, 0.0)
        self.reference_velocity = (0.0, 0.0)

    def step(self, dt, measured=None):
        """Advance by dt seconds and return the (east, north) speed to command.

        `measured` is the pad's true (east, north) position if it is known. It is used
        only to stop the pad drifting away from the reference over time.
        """
        p = self.params
        # The measurement describes where the pad is now, so compare it with where the
        # reference is now, before moving the reference on.
        pull_east = pull_north = 0.0
        if measured is not None:
            pull_east = p.correction_gain * (self.reference[0] - measured[0])
            pull_north = p.correction_gain * (self.reference[1] - measured[1])
            size = math.hypot(pull_east, pull_north)
            if size > p.max_correction:
                pull_east *= p.max_correction / size
                pull_north *= p.max_correction / size

        speed = abs(p.speed)
        if p.pattern == 'line':
            self._step_line(dt, speed)
        elif p.pattern == 'straight':
            self._step_straight(dt, speed)
        else:
            self._step_circle(dt, speed)
        return self.reference_velocity[0] + pull_east, self.reference_velocity[1] + pull_north

    def _step_line(self, dt, speed):
        p = self.params
        # Turn round early enough to come to rest at the end of the line.
        stopping_distance = self.rate ** 2 / (2.0 * p.accel)
        if self.direction * self.rate >= 0.0 and \
                self.direction * self.along + stopping_distance >= p.travel:
            self.direction = -self.direction
        self.rate = _towards(self.rate, self.direction * speed, p.accel * dt)
        self.along += self.rate * dt

        heading = math.radians(p.heading_deg)
        ux, uy = math.cos(heading), math.sin(heading)
        self.reference = (self.along * ux, self.along * uy)
        self.reference_velocity = (self.rate * ux, self.rate * uy)

    def _step_straight(self, dt, speed):
        p = self.params
        self.rate = _towards(self.rate, speed, p.accel * dt)
        self.along += self.rate * dt
        heading = math.radians(p.heading_deg)
        ux, uy = math.cos(heading), math.sin(heading)
        self.reference = (self.along * ux, self.along * uy)
        self.reference_velocity = (self.rate * ux, self.rate * uy)

    def _step_circle(self, dt, speed):
        p = self.params
        self.rate = _towards(self.rate, speed, p.accel * dt)
        self.along += self.rate * dt
        angle = self.along / p.radius
        self.reference = (p.radius * math.sin(angle), p.radius * (math.cos(angle) - 1.0))
        self.reference_velocity = (self.rate * math.cos(angle), -self.rate * math.sin(angle))
