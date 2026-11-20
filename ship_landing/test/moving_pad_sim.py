"""A small closed-loop world for testing the moving-pad landing logic. No ROS.

The drone model is shaped on a recorded flight (docs/tilt_check): it answers a
velocity command after a short delay, with limited acceleration and a lag. The camera
gives ten frames a second, some of them missing, each a little late, reading slightly
long, and nothing once the camera is too close to the pad to see even the small marker
whole, or too far to the side of it.
"""
import math
import random
from collections import deque
from dataclasses import dataclass

from ship_landing.pad_motion import MotionParams, PadMotion
from ship_landing.pad_tracking import MovingPadLander, MovingPadParams

DT = 0.01
PAD_START = (4.0, 3.0)
PAD_TOP = 0.12
PAD_HALF = 0.625
REST = 0.195           # drone origin above the surface it stands on
CAMERA_DOWN = 0.10
SMALL_MARKER_REACH = 0.11   # m from the pad centre to a corner of the small marker, which must be in the image


@dataclass
class World:
    pattern: str = 'line'
    pad_speed: float = 0.5
    pad_accel: float = 0.25
    seed: int = 0
    command_delay: float = 0.15      # s before a command starts to act
    lag: float = 0.35                # s
    accel_limit: float = 2.5         # m/s^2
    speed_gain: float = 1.0          # the drone flies this fraction of the commanded speed
    frame_loss: float = 0.2          # fraction of camera frames with no detection
    image_delay: tuple = (0.01, 0.15)
    range_scale: float = 1.02        # the detector reads this much long
    noise: float = 0.01              # m at zero height, grows with height
    min_camera_height: float = 0.15  # m, below this not even the small marker can be relied on
    start: tuple = (0.0, 0.0, 5.0)
    camera_stops_at: float = math.inf
    camera_gap: tuple = None         # (from, to) in seconds with no frames at all
    params: MovingPadParams = None


@dataclass
class Outcome:
    state: str
    reason: str
    time: float
    error_east: float = math.nan
    error_north: float = math.nan
    on_pad: bool = False
    relative_speed: float = math.nan     # m/s between drone and pad at touchdown
    states: tuple = ()

    @property
    def error(self):
        return math.hypot(self.error_east, self.error_north)


def fly(world, duration=150.0):
    rng = random.Random(world.seed)
    motion = PadMotion(MotionParams(pattern=world.pattern, speed=world.pad_speed, accel=world.pad_accel))
    lander = MovingPadLander(world.params or MovingPadParams(), camera_down=CAMERA_DOWN)
    # Let the pad run for a while first, so landings start at different points of its path.
    pad = [0.0, 0.0]
    for _ in range(int(rng.uniform(5.0, 25.0) / DT)):
        ve, vn = motion.step(DT, tuple(pad))
        pad[0] += ve * DT
        pad[1] += vn * DT

    pos = list(world.start)
    vel = [0.0, 0.0, 0.0]
    commands = deque()                   # (time, command) waiting for the delay
    acting = (0.0, 0.0, 0.0)
    arriving = deque()                   # (arrival time, detection)
    states = []
    landed_at = None
    touchdown = None
    pad_v = (0.0, 0.0)

    steps = int(duration / DT)
    for i in range(steps):
        t = i * DT
        # ---- pad
        pad_v = motion.step(DT, tuple(pad))
        pad[0] += pad_v[0] * DT
        pad[1] += pad_v[1] * DT
        pad_world = (PAD_START[0] + pad[0], PAD_START[1] + pad[1])

        # ---- drone
        while commands and commands[0][0] <= t:
            acting = commands.popleft()[1]
        over_pad = abs(pos[0] - pad_world[0]) < PAD_HALF and abs(pos[1] - pad_world[1]) < PAD_HALF
        floor = (PAD_TOP if over_pad else 0.0) + REST
        if landed_at is None:
            for axis in range(3):
                want = acting[axis] * (world.speed_gain if axis < 2 else 1.0)
                accel = (want - vel[axis]) / world.lag
                accel = max(-world.accel_limit, min(world.accel_limit, accel))
                vel[axis] += accel * DT
                pos[axis] += vel[axis] * DT
            if pos[2] <= floor:
                pos[2] = floor
                if vel[2] < 0.0:
                    landed_at = t
                    touchdown = (pos[0] - pad_world[0], pos[1] - pad_world[1], over_pad,
                                 math.hypot(vel[0] - pad_v[0], vel[1] - pad_v[1]))
                    carried = over_pad
        else:
            vel = [pad_v[0], pad_v[1], 0.0] if carried else [0.0, 0.0, 0.0]
            pos[0] += vel[0] * DT
            pos[1] += vel[1] * DT

        # ---- camera, ten frames a second
        in_gap = world.camera_gap is not None and world.camera_gap[0] <= t < world.camera_gap[1]
        if i % 10 == 0 and t < world.camera_stops_at and not in_gap:
            camera_height = pos[2] - CAMERA_DOWN - PAD_TOP
            east, north = pad_world[0] - pos[0], pad_world[1] - pos[1]
            in_view = (camera_height >= world.min_camera_height
                       and abs(east) < 1.4 * camera_height - SMALL_MARKER_REACH
                       and abs(north) < 1.0 * camera_height - SMALL_MARKER_REACH)
            if in_view and rng.random() >= world.frame_loss:
                sigma = world.noise + 0.004 * camera_height
                scale = world.range_scale
                measured_east = pos[0] + scale * east + rng.gauss(0.0, sigma)
                measured_north = pos[1] + scale * north + rng.gauss(0.0, sigma)
                # In the autopilot's altitude frame, which is zero where the drone stood at take-off.
                measured_up = (pos[2] - REST) - CAMERA_DOWN - scale * camera_height + rng.gauss(0.0, sigma)
                arriving.append((t + rng.uniform(*world.image_delay),
                                 (t, measured_east, measured_north, measured_up)))

        # ---- controller, ten times a second
        if i % 10 == 5:
            detection = None
            while arriving and arriving[0][0] <= t:
                detection = arriving.popleft()[1]
                if arriving and arriving[0][0] <= t:
                    lander.track.update(*detection)      # more than one arrived: use them all
            reported = (round(pos[0], 2), round(pos[1], 2), round(pos[2] - REST, 2))   # as the autopilot reports
            command = lander.update(t, reported, detection)
            commands.append((t + world.command_delay, (command.east, command.north, command.up)))
            if not states or states[-1] != lander.state:
                states.append(lander.state)
            if lander.state == 'FAILED':
                return Outcome('FAILED', lander.reason, t, states=tuple(states))
            # The motors are stopped a second after the lander reports contact while landed.
            if landed_at is not None and lander.state == 'CONTACT' and t - landed_at > 1.0:
                return Outcome('LANDED', lander.reason, t, touchdown[0], touchdown[1], touchdown[2],
                               touchdown[3], tuple(states))
    return Outcome('TIMEOUT', lander.state, duration, states=tuple(states))
