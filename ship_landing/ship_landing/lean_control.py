"""Hold the drone over the pad by leaning it, from the camera-only tracker. No ROS.

The landings so far told the autopilot how fast to fly and left the rest to it. Here
the autopilot only keeps the drone level and at its height (ALT_HOLD); this module
decides how far to lean, and the lean is sent as the stick positions a pilot would
give.

The rule, for each horizontal direction:

1. From how far the pad is, pick how fast the gap should close (proportional, with a
   speed limit so that a far pad is approached steadily).
2. Compare that with how fast the gap is actually closing, from the tracker, and
   accelerate by the difference times a gain.
3. Add the pad's own acceleration, so that the drone turns with a turning pad
   without first having to fall behind.

Acceleration becomes lean (a drone holding its height accelerates at g times the
tangent of its lean), lean is limited, and lean becomes stick through the line
measured in the stick calibration.
"""
import math
from dataclasses import dataclass

from ship_landing.relative_tracking import G


@dataclass
class StickMap:
    """Stick position for a lean, as measured: lean = slope x (stick - dead_zone)."""

    slope: float = 31.6        # degrees of lean per unit of stick beyond the dead zone
    dead_zone: float = 0.040   # stick travel either side of centre that does nothing

    def stick(self, lean_deg):
        if abs(lean_deg) < 0.05:
            return 0.0
        return math.copysign(abs(lean_deg) / self.slope + self.dead_zone, lean_deg)


@dataclass
class HoldParams:
    closing_gain: float = 1.0      # 1/s, closing speed per metre of gap
    max_closing_speed: float = 1.5 # m/s
    speed_gain: float = 2.5        # 1/s, acceleration per m/s of closing-speed error
    use_pad_accel: bool = True
    max_lean: float = 10.0         # degrees


@dataclass
class LeanCommand:
    roll_stick: float       # positive lowers the right side (the drone goes right)
    pitch_stick: float      # positive raises the nose (the drone goes backwards)
    accel_east: float
    accel_north: float
    lean_forward: float     # degrees, nose down
    lean_left: float        # degrees, left side down


class PadHold:
    def __init__(self, params=None, sticks=None):
        self.params = params or HoldParams()
        self.sticks = sticks or StickMap()

    def acceleration(self, track):
        """Wanted acceleration of the drone (east, north), m/s^2."""
        p = self.params
        gap = math.hypot(*track.position)
        closing = min(p.closing_gain * gap, p.max_closing_speed)
        accel = []
        for i in (0, 1):
            # The pad should come towards the drone at `closing`: its wanted relative
            # velocity points from the pad back to the drone.
            wanted = -closing * track.position[i] / gap if gap > 1e-6 else 0.0
            a = p.speed_gain * (track.velocity[i] - wanted)
            if p.use_pad_accel:
                a += track.pad_accel[i]
            accel.append(a)
        return accel

    def command(self, track, yaw):
        """Stick positions for the tracker's present state; yaw is the heading from east towards north."""
        east, north = self.acceleration(track)
        forward = east * math.cos(yaw) + north * math.sin(yaw)
        left = -east * math.sin(yaw) + north * math.cos(yaw)
        limit = self.params.max_lean
        lean_forward = max(-limit, min(limit, math.degrees(math.atan(forward / G))))
        lean_left = max(-limit, min(limit, math.degrees(math.atan(left / G))))
        return LeanCommand(roll_stick=-self.sticks.stick(lean_left), pitch_stick=-self.sticks.stick(lean_forward),
                           accel_east=east, accel_north=north, lean_forward=lean_forward, lean_left=lean_left)
