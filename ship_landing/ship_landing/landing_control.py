"""Decide how to move so the drone ends up over the landing pad and descends onto it.

Pure Python, no ROS, so the logic can be tested in a closed-loop simulation.

Frames. The detector reports the pad in the camera optical frame (x right in the
image, y down in the image, z out of the lens). The camera looks straight down with
the top of the image towards the drone's nose, so:

    pad is ahead of the drone by   -y
    pad is left of the drone by    -x
    drone is above the pad by       z

Commands are velocities in the drone's own frame: forward, left, up, in m/s.
"""
import math
from dataclasses import dataclass


@dataclass
class PadOffset:
    forward: float    # m, pad ahead of the drone
    left: float       # m, pad to the left of the drone
    height: float     # m, drone above the pad

    @property
    def horizontal(self):
        return math.hypot(self.forward, self.left)


@dataclass
class Command:
    forward: float = 0.0
    left: float = 0.0
    up: float = 0.0


@dataclass
class LandingParams:
    gain: float = 0.5               # 1/s, horizontal speed per metre of offset
    max_speed: float = 1.0          # m/s, horizontal speed limit
    descent_fast: float = 0.5       # m/s, descent rate above slow_below
    descent_slow: float = 0.3       # m/s, descent rate below slow_below
    slow_below: float = 2.0         # m
    align_tolerance: float = 0.3    # m, "over the pad" before the descent starts
    cone_base: float = 0.15         # m, allowed offset at zero height while descending
    cone_slope: float = 0.10        # m of extra allowed offset per metre of height
    handover_height: float = 1.0    # m, below this the autopilot's LAND mode finishes the job


def offset_from_camera(x, y, z):
    """Convert the detector's camera-frame position of the pad to a drone-frame offset."""
    return PadOffset(forward=-y, left=-x, height=z)


def horizontal_command(offset, params):
    """Velocity towards the pad, proportional to the offset and capped at max_speed."""
    forward, left = params.gain * offset.forward, params.gain * offset.left
    speed = math.hypot(forward, left)
    if speed > params.max_speed:
        forward, left = forward * params.max_speed / speed, left * params.max_speed / speed
    return Command(forward=forward, left=left)


def is_aligned(offset, params):
    """Close enough over the pad to begin descending."""
    return offset.horizontal < params.align_tolerance


def inside_cone(offset, params):
    """Within the cone that narrows towards the pad: the lower, the tighter."""
    return offset.horizontal < params.cone_base + params.cone_slope * offset.height


def descent_command(offset, params):
    """Keep centring; descend only while inside the cone."""
    cmd = horizontal_command(offset, params)
    if inside_cone(offset, params):
        rate = params.descent_fast if offset.height > params.slow_below else params.descent_slow
        cmd.up = -rate
    return cmd


def ready_for_handover(offset, params):
    """Low enough, and centred, to let the autopilot land straight down."""
    return offset.height < params.handover_height and inside_cone(offset, params)


class PadLander:
    """Mission logic for landing on the pad, as a small state machine.

    Call `update` at a steady rate with the current time and the latest pad offset
    (or None if there is no fresh detection). It returns the velocity to command.

    States:
        SEARCH    hover and wait for the marker
        ALIGN     fly over the pad at constant height
        DESCEND   keep centring and come down inside a narrowing cone
        HANDOVER  finished: low and centred, the autopilot should land now
        FAILED    finished: see `reason`
    """

    def __init__(self, params=None, search_timeout=20.0, align_timeout=60.0, descend_timeout=120.0,
                 align_hold=0.5, coast_time=0.5, lost_reset=3.0, lost_abort=10.0, climb_rate=0.3):
        self.params = params or LandingParams()
        self.search_timeout = search_timeout
        self.align_timeout = align_timeout
        self.descend_timeout = descend_timeout
        self.align_hold = align_hold        # s the drone must stay aligned before descending
        self.coast_time = coast_time        # s to keep the last command when a frame is missed
        self.lost_reset = lost_reset        # s without the marker before reacting
        self.lost_abort = lost_abort        # s without the marker before giving up
        self.climb_rate = climb_rate        # m/s, climbing to find the marker again

        self.state = 'SEARCH'
        self.reason = ''
        self.entered = None
        self.last_seen = None
        self.last_offset = None
        self.last_command = Command()
        self.aligned_since = None

    @property
    def done(self):
        return self.state in ('HANDOVER', 'FAILED')

    def _enter(self, state, now, reason=''):
        self.state, self.entered, self.reason = state, now, reason
        self.aligned_since = None

    def update(self, now, offset):
        if self.entered is None:
            self.entered = now
        if offset is not None:
            self.last_seen, self.last_offset = now, offset
        lost_for = math.inf if self.last_seen is None else now - self.last_seen
        in_state = now - self.entered

        command = Command()
        if self.state == 'SEARCH':
            if offset is not None:
                self._enter('ALIGN', now)
            elif in_state > self.search_timeout:
                self._enter('FAILED', now, 'marker not found')

        elif self.state == 'ALIGN':
            if offset is not None:
                command = horizontal_command(offset, self.params)
                if not is_aligned(offset, self.params):
                    self.aligned_since = None
                elif self.aligned_since is None:
                    self.aligned_since = now
                elif now - self.aligned_since >= self.align_hold:
                    self._enter('DESCEND', now)
            elif lost_for < self.coast_time:
                command = self.last_command
            elif lost_for > self.lost_reset:
                self._enter('SEARCH', now)
            if self.state == 'ALIGN' and in_state > self.align_timeout:
                self._enter('FAILED', now, 'could not get over the pad')

        elif self.state == 'DESCEND':
            near_ground = self.last_offset.height < self.params.handover_height + 0.5
            if offset is not None:
                if ready_for_handover(offset, self.params):
                    self._enter('HANDOVER', now)
                else:
                    command = descent_command(offset, self.params)
            elif lost_for < self.coast_time:
                command = self.last_command
            elif near_ground:
                # Expected: close to the ground the marker no longer fits in the image.
                self._enter('HANDOVER', now, 'marker left the image near the ground')
            elif lost_for > self.lost_abort:
                self._enter('FAILED', now, 'marker lost during the descent')
            elif lost_for > self.lost_reset:
                command = Command(up=self.climb_rate)     # climb to widen the view
            if self.state == 'DESCEND' and in_state > self.descend_timeout:
                self._enter('FAILED', now, 'descent took too long')

        self.last_command = command if not self.done else Command()
        return self.last_command
