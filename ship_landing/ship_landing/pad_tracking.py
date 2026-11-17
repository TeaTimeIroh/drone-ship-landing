"""Follow a moving pad and land on it. Pure Python, no ROS.

The static-pad controller steers by the pad's offset alone. Against a moving pad that
always trails behind, and it cannot finish with the autopilot's LAND mode, which lands
on a fixed spot. This module does three things differently:

1. It keeps an estimate of the pad's position AND velocity (PadTrack), built from
   detections that the caller has already placed in the autopilot's local frame.
2. It commands the pad's estimated velocity plus a correction for the remaining offset
   (MovingPadLander), so that the offset goes to zero on a pad moving at steady speed.
3. It flies all the way down. Below about 0.6 m the marker no longer fits in the image,
   so the last stretch follows the estimate without new detections.

Frame: east, north, up in metres, the same frame as the autopilot's position. Only
differences within this frame matter, so a constant or slowly drifting error in the
autopilot's position cancels out.
"""
import math
from collections import deque
from dataclasses import dataclass


@dataclass
class TrackParams:
    smoothing: float = 0.8       # 0..1 per camera frame; higher is smoother and slower to react
    frame_time: float = 0.1      # s, nominal time between detections
    frames_to_start: int = 8     # detections needed before the estimate is used
    restart_after: float = 2.0   # s without detections before the estimate starts afresh
    height_smoothing: float = 0.7
    max_accel: float = 0.6       # m/s^2, limit on the estimated acceleration
    accel_horizon: float = 0.5   # s, how far ahead the acceleration is trusted; beyond it the speed is held


class PadTrack:
    """Estimate of where the pad is, how fast it moves and how its speed is changing.

    A fixed-gain filter per horizontal axis with position, velocity and acceleration.
    Each detection first predicts the pad forward, then moves the three values part of
    the way towards what was measured. Tracking the acceleration is what lets the
    estimate keep up while the pad turns round or goes in a circle; without it the
    velocity estimate always runs behind.
    """

    def __init__(self, params=None):
        self.params = params or TrackParams()
        self.reset()

    def reset(self):
        self.time = None             # time of the newest detection used
        self.position = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.accel = [0.0, 0.0]
        self.pad_up = None           # altitude of the pad's surface
        self.count = 0
        self.accel_history = deque()     # (time, a_east, a_north) of the last second

    @property
    def ready(self):
        return self.count >= self.params.frames_to_start

    def age(self, now):
        """Seconds since the newest detection."""
        return math.inf if self.time is None else now - self.time

    def update(self, t, east, north, up):
        """Take in a detection made at time t."""
        p = self.params
        if self.time is not None and t <= self.time:
            return                                   # late or repeated frame
        if self.time is None or t - self.time > p.restart_after:
            self.reset()
            self.time, self.position, self.pad_up, self.count = t, [east, north], up, 1
            return
        dt = t - self.time
        # Gains of a critically damped filter, adjusted when frames were missed.
        theta = p.smoothing ** (dt / p.frame_time)
        g = 1.0 - theta ** 3
        h = 1.5 * (1.0 - theta * theta) * (1.0 - theta)
        k = 0.5 * (1.0 - theta) ** 3
        for axis, measured in enumerate((east, north)):
            predicted = self.position[axis] + self.velocity[axis] * dt + 0.5 * self.accel[axis] * dt * dt
            residual = measured - predicted
            self.position[axis] = predicted + g * residual
            self.velocity[axis] += self.accel[axis] * dt + h * residual / dt
            self.accel[axis] += 2.0 * k * residual / (dt * dt)
        size = math.hypot(*self.accel)
        if size > p.max_accel:
            self.accel = [a * p.max_accel / size for a in self.accel]
        keep = p.height_smoothing ** (dt / p.frame_time)
        self.pad_up = keep * self.pad_up + (1.0 - keep) * up
        self.time = t
        self.count += 1
        self.accel_history.append((t, self.accel[0], self.accel[1]))
        while self.accel_history[0][0] < t - 1.0:
            self.accel_history.popleft()

    def _ahead(self, now, extra=0.0):
        """Time to look ahead for the velocity part, and the capped time for the acceleration part."""
        dt = max(0.0, now - self.time) + extra
        return dt, min(dt, self.params.accel_horizon)

    def predict(self, now):
        """Where the pad is expected to be at time `now`: (east, north)."""
        dt, da = self._ahead(now)
        return tuple(self.position[i] + self.velocity[i] * dt + self.accel[i] * da * (dt - 0.5 * da)
                     for i in range(2))

    def velocity_at(self, now, lead=0.0):
        """The pad's expected velocity `lead` seconds after `now`: (east, north)."""
        _, da = self._ahead(now, lead)
        return tuple(self.velocity[i] + self.accel[i] * da for i in range(2))

    def unsteadiness(self, window=0.6):
        """How much the estimated acceleration has changed over the last `window` seconds, in m/s^2.

        Small while the pad holds its speed, turns steadily or brakes evenly. Large just
        after it starts or stops doing any of those, which is when the estimate is least
        reliable. Infinite until there is enough history.
        """
        if not self.ready or not self.accel_history:
            return math.inf
        newest = self.accel_history[-1]
        if newest[0] - self.accel_history[0][0] < 0.8 * window:
            return math.inf
        oldest = next(row for row in self.accel_history if row[0] >= newest[0] - window)
        return math.hypot(newest[1] - oldest[1], newest[2] - oldest[2])


@dataclass
class Command:
    east: float = 0.0
    north: float = 0.0
    up: float = 0.0


@dataclass
class MovingPadParams:
    gain: float = 1.0                # 1/s, extra speed per metre of offset
    lead_time: float = 0.4           # s, how far ahead the pad's velocity is taken, to offset the drone's lag
    integral_gain: float = 0.3       # 1/s^2, slowly removes an offset that persists
    max_integral: float = 0.3        # m/s
    integral_zone: float = 0.5       # m, the integral only works on offsets smaller than this
    max_closing_speed: float = 1.0   # m/s, speed relative to the pad while catching up
    max_speed: float = 3.0           # m/s, overall horizontal limit
    descent_fast: float = 0.5        # m/s, above slow_below
    descent_slow: float = 0.3        # m/s, below slow_below
    slow_below: float = 2.0          # m
    cone_base: float = 0.15          # m, allowed offset at zero height while descending
    cone_slope: float = 0.10         # m of extra allowed offset per metre of height
    follow_hold: float = 1.0         # s inside the cone before the descent starts
    commit_height: float = 0.85      # m, the drone waits at this height for a good moment to commit
    commit_hold: float = 0.4         # s the pad must be steady and the drone centred before committing
    steady_change: float = 0.06      # m/s^2, the pad's acceleration must have changed less than this lately
    final_descent: float = 0.5       # m/s, rate of the committed descent
    abort_offset: float = 0.3        # m, a seen offset larger than this cancels the commitment
    contact_height: float = 0.20     # m, from here down the drone only matches the pad's speed
    stall_time: float = 0.5          # s without losing height, while descending, means touchdown
    coast_time: float = 0.7          # s without detections before climbing to look again
    climb_after: float = 2.0         # s without detections before the estimate is no longer followed
    give_up_after: float = 10.0      # s without detections before abandoning
    climb_rate: float = 0.5          # m/s
    search_timeout: float = 30.0     # s
    final_timeout: float = 12.0      # s


class MovingPadLander:
    """Mission logic for landing on a moving pad.

    Call `update` at a steady rate (10 Hz) with the time, the drone's position and, when
    a new detection has arrived, the pad's measured position with the time its image
    was taken. It returns the velocity to command in the east/north/up frame.

    States:
        SEARCH    hover until the pad has been seen often enough to know its velocity
        FOLLOW    catch up with the pad at constant height and stay over it
        DESCEND   keep following; come down while inside a cone that narrows with height
        FINAL     committed descent, the last part without sight of the marker
        CONTACT   on the pad (or about to be): go wherever the pad carries the drone, and press down
        FAILED    see `reason`

    The caller stops the motors while in CONTACT; this class never does.
    """

    def __init__(self, params=None, track=None, camera_down=0.10):
        self.params = params or MovingPadParams()
        self.track = track or PadTrack()
        self.camera_down = camera_down
        self.state = 'SEARCH'
        self.reason = ''
        self.entered = None
        self.last_time = None
        self.integral = [0.0, 0.0]
        self.inside_since = None
        self.offset = None           # (east, north) from drone to pad, latest
        self.height = None           # camera above the pad, latest
        self.stall_from = None       # (time, altitude) for touchdown detection
        self.drone_path = deque()    # (time, east, north) of the last half second, for the drone's own velocity

    @property
    def done(self):
        return self.state == 'FAILED'

    def _enter(self, state, now, reason=''):
        self.state, self.entered, self.reason = state, now, reason
        self.inside_since = None
        self.stall_from = None

    def allowed_offset(self, height):
        return self.params.cone_base + self.params.cone_slope * max(0.0, height)

    def update(self, now, drone, detection=None):
        """drone = (east, north, up); detection = (image time, east, north, pad surface up) or None."""
        p = self.params
        if self.entered is None:
            self.entered = now
        dt = 0.0 if self.last_time is None else max(0.0, min(0.5, now - self.last_time))
        self.last_time = now
        if detection is not None:
            self.track.update(*detection)
        age = self.track.age(now)
        self.drone_path.append((now, drone[0], drone[1]))
        while self.drone_path[0][0] < now - 0.5:
            self.drone_path.popleft()

        if self.state == 'FAILED':
            return Command()
        if self.state == 'SEARCH':
            if self.track.ready and age < p.coast_time:
                self._enter('FOLLOW', now)
            elif now - self.entered > p.search_timeout:
                self._enter('FAILED', now, 'the pad was not found')
                return Command()
            else:
                return Command()

        # From here on there is an estimate to steer by.
        target = self.track.predict(now)
        east, north = target[0] - drone[0], target[1] - drone[1]
        self.offset = (east, north)
        distance = math.hypot(east, north)
        self.height = drone[2] - self.camera_down - self.track.pad_up
        # Aim the command a little into the future, because the drone answers with a lag.
        ve, vn = self.track.velocity_at(now, p.lead_time)

        if self.state == 'CONTACT':
            return self._contact_command()

        # Catch-up velocity: proportional to the offset, plus a slow integral.
        # The integral is for a steady bias, such as the drone flying a little slower than
        # commanded. It is frozen while the pad changes speed, when the offset is not a bias.
        if distance < p.integral_zone and self.track.unsteadiness() < p.steady_change:
            for axis, error in enumerate((east, north)):
                self.integral[axis] += p.integral_gain * error * dt
            size = math.hypot(*self.integral)
            if size > p.max_integral:
                self.integral = [c * p.max_integral / size for c in self.integral]
        closing = [p.gain * east + self.integral[0], p.gain * north + self.integral[1]]
        size = math.hypot(*closing)
        if size > p.max_closing_speed:
            closing = [c * p.max_closing_speed / size for c in closing]
        command = Command(east=ve + closing[0], north=vn + closing[1])
        size = math.hypot(command.east, command.north)
        if size > p.max_speed:
            command.east, command.north = command.east * p.max_speed / size, command.north * p.max_speed / size

        if self.state == 'FINAL':
            return self._final(now, drone, command, distance, age)

        # FOLLOW and DESCEND need to keep seeing the pad.
        if age > p.give_up_after:
            self._enter('FAILED', now, f'the pad was not seen for {p.give_up_after:.0f} s')
            return Command()
        if age > p.climb_after:
            # The estimate is too old to follow. Stop and climb: a wider view finds the pad
            # sooner than chasing where it was last heading.
            return Command(up=p.climb_rate)
        if age > p.coast_time:
            command.up = p.climb_rate                # keep following the estimate, start climbing
            return command

        inside = distance < self.allowed_offset(self.height)
        if self.state == 'FOLLOW':
            if not inside:
                self.inside_since = None
            elif self.inside_since is None:
                self.inside_since = now
            elif now - self.inside_since >= p.follow_hold:
                self._enter('DESCEND', now)
            return command

        # DESCEND
        if self.height < p.commit_height - 0.15:
            command.up = p.climb_rate                # a cancelled final descent: back up to where the marker is visible
            return command
        if self.height <= p.commit_height:
            # Wait here, just above where the marker goes out of view, until the pad has
            # been steady and the drone centred for a moment. Then commit.
            if not (inside and self.track.unsteadiness() < p.steady_change):
                self.inside_since = None
            elif self.inside_since is None:
                self.inside_since = now
            elif now - self.inside_since >= p.commit_hold:
                self._enter('FINAL', now)
                command.up = -p.final_descent
            return command
        if inside:
            command.up = -(p.descent_fast if self.height > p.slow_below else p.descent_slow)
        return command

    def _final(self, now, drone, command, distance, age):
        p = self.params
        # While the marker is still in view, a large offset means the pad did something
        # unexpected: go back to following instead of landing beside it.
        if age < 0.3 and self.height > p.contact_height + 0.2 and (
                distance > p.abort_offset or self.track.unsteadiness() > 2.0 * p.steady_change):
            self._enter('DESCEND', now)
            return command
        if now - self.entered > p.final_timeout:
            self._enter('FAILED', now, 'the final descent did not reach the pad')
            return Command()
        command.up = -p.final_descent

        # Touchdown: low enough by the estimate, or no longer losing height.
        if self.stall_from is None or self.stall_from[1] - drone[2] > 0.03:
            self.stall_from = (now, drone[2])
        # A stall only counts near the pad: higher up the drone may simply still be slowing its climb.
        stalled = (now - self.stall_from[0] >= p.stall_time and now - self.entered > 1.0
                   and self.height < p.contact_height + 0.25)
        if self.height <= p.contact_height or stalled:
            self._enter('CONTACT', now, 'stopped descending' if stalled else 'reached the pad height')
            return self._contact_command()
        return command

    def own_velocity(self):
        """The drone's horizontal velocity over the last few tenths of a second, from its positions."""
        if len(self.drone_path) < 2:
            return (0.0, 0.0)
        (t0, e0, n0), (t1, e1, n1) = self.drone_path[0], self.drone_path[-1]
        if t1 - t0 < 0.15:
            return (0.0, 0.0)
        return ((e1 - e0) / (t1 - t0), (n1 - n0) / (t1 - t0))

    def _contact_command(self):
        """On the pad the drone is carried along, so ask for the velocity it already has.

        Asking for the pad's estimated velocity instead goes wrong as soon as the pad
        changes pace: the estimate is no longer updated, the autopilot sees a growing
        velocity error, leans against the pad, and its landing detector, which will not
        accept a leaning drone as landed, never lets the motors be stopped.
        """
        east, north = self.own_velocity()
        return Command(east=east, north=north, up=-self.params.final_descent)
