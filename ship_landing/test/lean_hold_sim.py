"""A drone flown by stick positions, for testing the lean controller. Not a test itself.

The drone follows what the stick calibration and check flights measured: lean =
31.6 deg x (stick - 0.04), reached with a time constant of about 0.27 s after a short
delay, and acceleration g x tan(lean). The camera sees the pad ten times a second,
a little late, with noise that grows with height, and sometimes not at all.
"""
import math
import random
from dataclasses import dataclass

from ship_landing.lean_control import HoldParams, PadHold, StickMap
from ship_landing.lean_landing import LandingParams, LeanLander
from ship_landing.relative_tracking import G, RelativeParams, RelativeTrack

DT = 0.01


@dataclass
class Sim:
    pad: callable                    # t -> ((east, north), (ve, vn))
    start: tuple = (0.0, 0.0)
    height: float = 6.0
    seed: int = 0
    lean_lag: float = 0.27           # s
    stick_delay: float = 0.08        # s from sending a stick to the lean starting to change
    camera_delay: float = 0.10       # s
    noise_per_metre: float = 0.004   # m of detection noise per metre of height
    frame_loss: float = 0.15
    true_slope: float = 31.6
    true_dead_zone: float = 0.040
    drag: float = 0.05               # 1/s, a little air resistance
    seconds: float = 40.0
    control_period: float = 0.05
    params: HoldParams = None
    tracking: RelativeParams = None


def run(sim):
    """Fly it. Returns a list of (time, distance from the pad centre, drone speed relative to the pad)."""
    rng = random.Random(sim.seed)
    hold = PadHold(sim.params or HoldParams(), StickMap())
    track = RelativeTrack(sim.tracking)
    yaw = math.pi / 2.0                                  # facing north
    position, velocity = list(sim.start), [0.0, 0.0]
    lean = [0.0, 0.0]                                    # degrees: nose down, left side down
    sticks, applied, stick_queue, frames = (0.0, 0.0), (0.0, 0.0), [], []
    out = []
    for step in range(int(sim.seconds / DT)):
        t = step * DT
        pad_position, pad_velocity = sim.pad(t)
        # sticks -> wanted lean, in the body frame; with yaw = north, forward is north and left is west
        stick_queue.append((t + sim.stick_delay, sticks))
        while stick_queue and stick_queue[0][0] <= t:
            _, applied = stick_queue.pop(0)

        def lean_for(stick):
            if abs(stick) <= sim.true_dead_zone:
                return 0.0
            return math.copysign(sim.true_slope * (abs(stick) - sim.true_dead_zone), stick)
        want_left_down = -lean_for(applied[0])        # positive roll stick lowers the right side
        want_nose_down = -lean_for(applied[1])        # positive pitch stick raises the nose
        lean[0] += (want_nose_down - lean[0]) * DT / sim.lean_lag
        lean[1] += (want_left_down - lean[1]) * DT / sim.lean_lag
        forward = G * math.tan(math.radians(lean[0]))
        left = G * math.tan(math.radians(lean[1]))
        accel = (forward * math.cos(yaw) - left * math.sin(yaw) - sim.drag * velocity[0],
                 forward * math.sin(yaw) + left * math.cos(yaw) - sim.drag * velocity[1])
        for i in (0, 1):
            position[i] += velocity[i] * DT + 0.5 * accel[i] * DT * DT
            velocity[i] += accel[i] * DT
        # what the tracker is told: the lean (so the thrust part of the acceleration), not the drag
        measured_accel = (forward * math.cos(yaw) - left * math.sin(yaw), forward * math.sin(yaw) + left * math.cos(yaw))
        track.predict(t, measured_accel)
        if step % 10 == 0 and rng.random() > sim.frame_loss:
            noise = sim.noise_per_metre * sim.height
            frames.append((t + sim.camera_delay, t, pad_position[0] - position[0] + rng.gauss(0, noise),
                           pad_position[1] - position[1] + rng.gauss(0, noise)))
        while frames and frames[0][0] <= t:
            _, stamp, east, north = frames.pop(0)
            track.update(stamp, east, north, sim.height)
        if step % round(sim.control_period / DT) == 0:
            if track.ready:
                command = hold.command(track, yaw)
                sticks = (command.roll_stick, command.pitch_stick)
            else:
                sticks = (0.0, 0.0)
        out.append((t, math.hypot(pad_position[0] - position[0], pad_position[1] - position[1]),
                    math.hypot(pad_velocity[0] - velocity[0], pad_velocity[1] - velocity[1])))
    return out


def still(t):
    return (3.0, 2.0), (0.0, 0.0)


def circle(t):
    w = 0.5 / 1.5
    return (4.0 + 1.5 * math.sin(w * t), 3.0 + 1.5 * (math.cos(w * t) - 1.0) * -1.0 - 1.5 + 1.5), (0.5 * math.cos(w * t), 0.5 * math.sin(w * t))


def line(t):
    """Back and forth along east: 8 s at 0.5 m/s, 4 s to turn round at 0.25 m/s^2."""
    u = t % 24.0
    if u < 8:
        x, v = 0.5 * u, 0.5
    elif u < 12:
        d = u - 8
        x, v = 4 + 0.5 * d - 0.125 * d * d, 0.5 - 0.25 * d
    elif u < 20:
        d = u - 12
        x, v = 4 - 0.5 * d, -0.5
    else:
        d = u - 20
        x, v = -0.5 * d + 0.125 * d * d, -0.5 + 0.25 * d
    return (2.0 + x, 3.0), (v, 0.0)


# ---- landing ---------------------------------------------------------------------------

REST = 0.20            # m, the drone's centre above the pad when it stands on it
CAMERA_BELOW = 0.10    # m, the camera below the drone's centre
MIN_CAMERA = 0.15      # m, closer than this the small marker cannot be relied on


@dataclass
class Landing:
    pad: callable
    start: tuple = (0.0, 0.0)
    height: float = 6.2              # m, drone centre above the pad at the start
    seed: int = 0
    lean_lag: float = 0.27
    stick_delay: float = 0.08
    climb_lag: float = 0.30          # s, how quickly the climb rate follows the throttle stick
    camera_delay: float = 0.10
    noise_per_metre: float = 0.003
    frame_loss: float = 0.15
    true_slope: float = 31.6
    true_dead_zone: float = 0.040
    throttle_dead_zone: float = 0.2
    throttle_full_rate: float = 2.5
    drag: float = 0.05
    seconds: float = 120.0
    params: LandingParams = None
    tracking: RelativeParams = None


@dataclass
class Outcome:
    landed: bool
    error: float           # m from the pad centre at touchdown
    time: float
    state: str
    reason: str
    attempts: int
    sink: float            # m/s downwards at touchdown
    along: float = 0.0     # m, drone ahead of the pad centre along the pad's travel
    across: float = 0.0    # m, drone to the left of it


def land(sim):
    rng = random.Random(sim.seed)
    lander = LeanLander(sim.params)
    track = RelativeTrack(sim.tracking)
    yaw = math.pi / 2.0
    position, velocity = list(sim.start), [0.0, 0.0]
    height, climb = sim.height, 0.0
    lean = [0.0, 0.0]
    sticks, applied, stick_queue, frames = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), [], []

    def lean_for(stick):
        if abs(stick) <= sim.true_dead_zone:
            return 0.0
        return math.copysign(sim.true_slope * (abs(stick) - sim.true_dead_zone), stick)

    def rate_for(stick):
        if abs(stick) <= sim.throttle_dead_zone:
            return 0.0
        return math.copysign(sim.throttle_full_rate * (abs(stick) - sim.throttle_dead_zone)
                             / (1.0 - sim.throttle_dead_zone), stick)

    for step in range(int(sim.seconds / DT)):
        t = step * DT
        pad_position, pad_velocity = sim.pad(t)
        stick_queue.append((t + sim.stick_delay, sticks))
        while stick_queue and stick_queue[0][0] <= t:
            _, applied = stick_queue.pop(0)
        lean[0] += (-lean_for(applied[1]) - lean[0]) * DT / sim.lean_lag
        lean[1] += (-lean_for(applied[0]) - lean[1]) * DT / sim.lean_lag
        forward, left = G * math.tan(math.radians(lean[0])), G * math.tan(math.radians(lean[1]))
        thrust = (forward * math.cos(yaw) - left * math.sin(yaw), forward * math.sin(yaw) + left * math.cos(yaw))
        for i in (0, 1):
            accel = thrust[i] - sim.drag * velocity[i]
            position[i] += velocity[i] * DT + 0.5 * accel * DT * DT
            velocity[i] += accel * DT
        climb += (rate_for(applied[2]) - climb) * DT / sim.climb_lag
        height += climb * DT
        gap = math.hypot(pad_position[0] - position[0], pad_position[1] - position[1])
        if height <= REST:
            on_pad = abs(pad_position[0] - position[0]) < 0.625 and abs(pad_position[1] - position[1]) < 0.625
            speed = math.hypot(*pad_velocity) or 1.0
            east, north = position[0] - pad_position[0], position[1] - pad_position[1]
            outcome = Outcome(on_pad, gap, t, lander.state, lander.reason, lander.final_attempts, -climb)
            outcome.along = (east * pad_velocity[0] + north * pad_velocity[1]) / speed
            outcome.across = (north * pad_velocity[0] - east * pad_velocity[1]) / speed
            return outcome
        track.predict(t, thrust)
        camera_height = height - CAMERA_BELOW
        if step % 10 == 0 and rng.random() > sim.frame_loss and camera_height >= MIN_CAMERA:
            east, north = pad_position[0] - position[0], pad_position[1] - position[1]
            if abs(east) < 1.4 * camera_height - 0.11 and abs(north) < 1.0 * camera_height - 0.11:
                noise = sim.noise_per_metre * camera_height
                frames.append((t + sim.camera_delay, t, east + rng.gauss(0, noise), north + rng.gauss(0, noise),
                               height * 1.02))
        while frames and frames[0][0] <= t:
            _, stamp, east, north, seen_height = frames.pop(0)
            track.update(stamp, east, north, seen_height)
        if step % 5 == 0:
            command = lander.update(t, track, yaw)
            sticks = (command.roll, command.pitch, command.throttle)
            if lander.state == 'FAILED':
                return Outcome(False, float('nan'), t, 'FAILED', lander.reason, lander.final_attempts, 0.0)
    return Outcome(False, float('nan'), sim.seconds, lander.state, 'ran out of time', lander.final_attempts, 0.0)
