"""Land on the pad by leaning and by the throttle stick, from the camera-only tracker. No ROS.

lean_control.PadHold keeps the drone over the pad. This adds the way down:

    APPROACH   hold the height until the drone is over the pad and moving with it
    DESCEND    come down while inside a cone that narrows towards the pad; pause when outside
    WAIT       close above the pad, hold until centred and matched in speed for a moment
    COMMIT     the short drop that is left, without pausing
    CONTACT    on the pad: level sticks, throttle stick at the bottom, until the motors stop
    CLIMB      pad lost from view: level the drone and climb to find it again
    FAILED     pad not found again; the caller takes the drone back

The drone is steered the whole way by PadHold, including during COMMIT, because the
small marker stays in view almost to the pad. Height is the tracker's: the drone's
centre above the pad's surface.
"""
import math
from dataclasses import dataclass, field

from ship_landing.lean_control import HoldParams, PadHold, StickMap


@dataclass
class ThrottleMap:
    """Throttle stick for a climb rate in ALT_HOLD: nothing inside the dead zone, then linear."""

    dead_zone: float = 0.2     # stick travel either side of centre that holds the height
    full_rate: float = 2.5     # m/s at full stick

    def stick(self, climb_rate):
        if abs(climb_rate) < 0.01:
            return 0.0
        return math.copysign(min(1.0, self.dead_zone + (1.0 - self.dead_zone) * abs(climb_rate) / self.full_rate),
                             climb_rate)


@dataclass
class LandingParams:
    arrive_gap: float = 0.30         # m, "over the pad" for starting the descent
    arrive_speed: float = 0.30       # m/s, relative
    cone_base: float = 0.08          # m, allowed gap at the pad
    cone_slope: float = 0.06         # m of extra allowed gap per metre of height
    descent_fast: float = 0.6        # m/s, high up
    descent_slow: float = 0.3        # m/s, close to the pad
    slow_below: float = 1.5          # m
    wait_height: float = 0.50        # m, drone centre above the pad (its legs are then about 0.3 m above it)
    approach_gain: float = 1.0       # 1/s, descent rate per metre still to go to the waiting height
    commit_gap: float = 0.05         # m
    commit_speed: float = 0.10       # m/s, relative
    commit_hold: float = 0.4         # s these must hold
    patience: float = 8.0            # s of waiting after which the two limits start to widen
    widen_per_second: float = 0.01   # m (and m/s) wider per second beyond that, up to three times the limit
    commit_descent: float = 0.5      # m/s
    contact_height: float = 0.24     # m, the drone stands on the pad with its centre 0.20 m above it
    stall_time: float = 0.6          # s without getting lower during COMMIT also counts as contact
    press_time: float = 0.6          # s of pressing down gently on contact before the throttle goes to the bottom
    lost_after: float = 0.8          # s without a detection
    climb_rate: float = 0.4          # m/s while looking for the pad again
    give_up_after: float = 6.0       # s
    hold: HoldParams = field(default_factory=HoldParams)


@dataclass
class Sticks:
    roll: float = 0.0
    pitch: float = 0.0
    throttle: float = 0.0


class LeanLander:
    def __init__(self, params=None, sticks=None, throttle=None):
        self.params = params or LandingParams()
        self.hold = PadHold(self.params.hold, sticks or StickMap())
        self.throttle = throttle or ThrottleMap()
        self.state = 'APPROACH'
        self.state_since = None
        self.good_since = None
        self.lowest = None
        self.lowest_time = None
        self.final_attempts = 0
        self.contact_time = None
        self.reason = ''

    def _enter(self, state, now, reason=''):
        self.state, self.state_since, self.reason = state, now, reason
        self.good_since = None
        if state == 'CONTACT':
            self.contact_time = now
        if state == 'COMMIT':
            self.final_attempts += 1
            self.lowest, self.lowest_time = None, now

    def _held(self, condition, now, seconds):
        """True once `condition` has been true for `seconds` without a break."""
        if not condition:
            self.good_since = None
            return False
        if self.good_since is None:
            self.good_since = now
        return now - self.good_since >= seconds

    def _slack(self, now):
        """How much wider the commit limits are by now: a drone that cannot quite centre must still land."""
        p = self.params
        waited = now - self.state_since - p.patience
        return max(0.0, min(2.0 * p.commit_gap, p.widen_per_second * waited))

    def contact_sticks(self, now):
        """Level, pressing down gently at first in case the pad is still a little way below, then throttle off."""
        p = self.params
        if now - self.contact_time < p.press_time:
            return Sticks(0.0, 0.0, self.throttle.stick(-p.commit_descent))
        return Sticks(0.0, 0.0, -1.0)

    def update(self, now, track, yaw):
        """Stick positions for this moment. `track` is a RelativeTrack moved on to `now`."""
        p = self.params
        if self.state_since is None:
            self.state_since = now
        if self.state == 'CONTACT':
            return self.contact_sticks(now)
        if self.state == 'FAILED':
            return Sticks()

        age = track.age(now)
        if self.state == 'CLIMB':
            if age < 0.3 and track.ready:
                self._enter('APPROACH', now, 'pad found again')
            elif now - self.state_since > p.give_up_after:
                self._enter('FAILED', now, f'the pad was not seen for {p.give_up_after:.0f} s')
                return Sticks()
            else:
                return Sticks(0.0, 0.0, self.throttle.stick(p.climb_rate))
        if age > p.lost_after or not track.ready or track.height is None:
            if self.state == 'COMMIT' and track.height is not None and track.height < p.wait_height:
                # Too close for the camera on the way down: this is the pad arriving, not the pad lost.
                self._enter('CONTACT', now, 'reached the pad (marker too close to see)')
                return self.contact_sticks(now)
            self._enter('CLIMB', now, 'pad lost from view')
            return Sticks(0.0, 0.0, self.throttle.stick(p.climb_rate))

        command = self.hold.command(track, yaw)
        gap, speed, height = math.hypot(*track.position), track.speed(), track.height
        climb = 0.0

        if self.state == 'APPROACH':
            if self._held(gap < p.arrive_gap and speed < p.arrive_speed, now, 1.0):
                self._enter('DESCEND', now)
        elif self.state == 'DESCEND':
            if height <= p.wait_height + 0.05:
                self._enter('WAIT', now)
            elif gap < p.cone_base + p.cone_slope * height:
                rate = p.descent_fast if height > p.slow_below else p.descent_slow
                climb = -min(rate, max(0.08, p.approach_gain * (height - p.wait_height)))
        elif self.state == 'WAIT':
            if height > p.wait_height + 0.25:
                self._enter('DESCEND', now)
            elif self._held(gap < p.commit_gap + self._slack(now) and speed < p.commit_speed + self._slack(now),
                            now, p.commit_hold):
                self._enter('COMMIT', now)
            else:
                climb = max(-0.15, min(0.15, p.approach_gain * (p.wait_height - height)))
        if self.state == 'COMMIT':
            climb = -p.commit_descent
            if self.lowest is None or height < self.lowest - 0.01:
                self.lowest, self.lowest_time = height, now
            if height <= p.contact_height:
                self._enter('CONTACT', now, 'reached the pad height')
            elif now - self.lowest_time > p.stall_time:
                self._enter('CONTACT', now, 'stopped getting lower')
            if self.state == 'CONTACT':
                return self.contact_sticks(now)
        return Sticks(command.roll_stick, command.pitch_stick, self.throttle.stick(climb))
