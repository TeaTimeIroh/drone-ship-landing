"""Land on the pad many times from random start points and record how accurately.

Needs the simulator and the marker detector running (pad_sim.launch.py):

    ros2 run ship_landing landing_trials --ros-args -p use_sim_time:=true -p trials:=10

Each trial: take off, fly to a random start point 1.5 to 4 m from the pad, hover,
read the drone's true position from Gazebo, land on the pad exactly as land_on_pad
does, wait for the motors to stop, and read the true position again. One CSV row is
written per trial, as it finishes, so an interrupted run keeps its results.

The marker is used to reach the start point too, so the drone must be able to see it
after take-off. Two failed trials in a row end the run.
"""
import csv
import math
import random
from pathlib import Path

from ship_landing.flight_base import run
from ship_landing.ground_truth import gz_model_pose
from ship_landing.land_on_pad import LandOnPad
from ship_landing.landing_control import PadOffset, horizontal_command
from ship_landing.trial_stats import COLUMNS

IN_POSITION = 0.25          # m, close enough to the start point
IN_POSITION_HOLD = 1.5      # s to stay there before measuring
MEASURE_SETTLE = 2.0        # s of hover before reading the true position
MARKER_STALE = 1.0          # s without a detection before hovering in place
SETTLE_MIN = 2.0            # s on the ground before reading the true position
SETTLE_MAX = 12.0           # s to wait for the motors to stop
MAX_CONSECUTIVE_FAILURES = 2


class LandingTrials(LandOnPad):
    STATE_TIMEOUT = dict(LandOnPad.STATE_TIMEOUT, CONNECT=240.0, READY=240.0, ARM=120.0,
                         POSITION=90.0, MEASURE=30.0, SETTLE=40.0)
    AIRBORNE_STATES = ('CLIMB', 'POSITION', 'MEASURE', 'APPROACH')
    FIRST_MISSION_STATE = 'POSITION'

    def __init__(self):
        super().__init__('landing_trials')
        self.trials = int(self.declare_parameter('trials', 10).value)
        self.seed = int(self.declare_parameter('seed', 0).value)
        self.batch = str(self.declare_parameter('batch', '').value) or f'seed{self.seed}'
        self.output = Path(self.declare_parameter('output', 'landing_trials.csv').value)
        self.pad_x = float(self.declare_parameter('pad_x', 3.0).value)
        self.pad_y = float(self.declare_parameter('pad_y', 2.0).value)
        self.pad_top = float(self.declare_parameter('pad_top', 0.02).value)
        self.radius_min = float(self.declare_parameter('start_radius_min', 1.5).value)
        self.radius_max = float(self.declare_parameter('start_radius_max', 4.0).value)
        self.model = str(self.declare_parameter('model', 'iris').value)

        self.rng = random.Random(self.seed)
        self.trial = 0
        self.consecutive_failures = 0
        self.errors = []
        self.row = None
        self.target = None              # where the pad should appear from the start point
        self.in_position_since = None
        self.last_seen = None
        self.last_position_command = None
        self.approach_started = None

        self.output.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output, 'w', newline='') as handle:
            csv.DictWriter(handle, COLUMNS).writeheader()
        self.get_logger().info(f'{self.trials} trials, seed {self.seed}, results in {self.output}')
        self.begin_trial()

    # ---- trial bookkeeping -------------------------------------------------------------

    def begin_trial(self):
        self.trial += 1
        radius = self.rng.uniform(self.radius_min, self.radius_max)
        bearing = self.rng.uniform(0.0, 2.0 * math.pi)
        # The start point is described by where the pad appears from it.
        self.target = PadOffset(forward=radius * math.cos(bearing), left=radius * math.sin(bearing), height=0.0)
        self.row = dict.fromkeys(COLUMNS, '')
        self.row.update(batch=self.batch, seed=self.seed, trial=self.trial,
                        start_forward=round(self.target.forward, 3), start_left=round(self.target.left, 3))
        self.lander = None
        self.lander_state = None
        self.in_position_since = None
        self.last_position_command = None
        self.approach_started = None
        self.failed = False
        self.get_logger().info(
            f'--- trial {self.trial}/{self.trials}: start with the pad {self.target.forward:+.2f} m ahead, '
            f'{self.target.left:+.2f} m left')

    def end_trial(self, now):
        pose = gz_model_pose(self.model)
        if pose is None:
            self.get_logger().error('Could not read the true position from Gazebo')
            self.row['reason'] = self.row['reason'] or 'no ground truth at touchdown'
        else:
            error_x, error_y = pose.x - self.pad_x, pose.y - self.pad_y
            self.row.update(land_x=round(pose.x, 4), land_y=round(pose.y, 4), land_z=round(pose.z, 4),
                            error_x=round(error_x, 4), error_y=round(error_y, 4),
                            error=round(math.hypot(error_x, error_y), 4))
        landed = not self.row['reason'] and pose is not None
        self.row['outcome'] = 'landed' if landed else 'failed'
        with open(self.output, 'a', newline='') as handle:
            csv.DictWriter(handle, COLUMNS).writerow(self.row)

        if landed:
            self.consecutive_failures = 0
            self.errors.append(self.row['error'])
            self.get_logger().info(
                f'trial {self.trial}: landed {100 * self.row["error"]:.1f} cm from the pad centre '
                f'({100 * self.row["error_x"]:+.1f} cm east, {100 * self.row["error_y"]:+.1f} cm north)')
        else:
            self.consecutive_failures += 1
            self.get_logger().error(f'trial {self.trial}: failed, {self.row["reason"]}')

        if self.trial >= self.trials:
            self.wrap_up()
        elif self.consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
            self.get_logger().error(f'{MAX_CONSECUTIVE_FAILURES} failures in a row, stopping')
            self.failed = True
            self.wrap_up()
        else:
            self.begin_trial()
            self.enter('GUIDED', now)

    def wrap_up(self):
        if self.errors:
            ordered = sorted(self.errors)
            self.get_logger().info(
                f'{len(ordered)} of {self.trial} trials landed: median {100 * ordered[len(ordered) // 2]:.1f} cm, '
                f'worst {100 * ordered[-1]:.1f} cm from the pad centre. Results: {self.output}')
        self.finish()

    # ---- overrides ---------------------------------------------------------------------

    def fail(self, reason, now):
        """A failed trial lands and the run carries on; a failure on the ground ends the run."""
        self.get_logger().error(reason)
        self.row['reason'] = self.row['reason'] or reason
        if self.state in self.AIRBORNE_STATES:
            self.enter('LAND', now)
        else:
            self.failed = True
            self.row['outcome'] = 'failed'
            with open(self.output, 'a', newline='') as handle:
                csv.DictWriter(handle, COLUMNS).writerow(self.row)
            self.wrap_up()

    def on_touchdown(self, now):
        if self.approach_started is not None:
            self.row['approach_time'] = round(now.nanoseconds * 1e-9 - self.approach_started, 2)
        self.enter('SETTLE', now)

    def fly(self, now):
        if self.state == 'POSITION':
            self.go_to_start_point(now)
        elif self.state == 'MEASURE':
            self.measure_start_point(now)
        elif self.state == 'APPROACH':
            self.approach(now)
            if self.state == 'LAND' and self.last_offset is not None and not self.row['reason']:
                self.row['handover_offset'] = round(self.last_offset.horizontal, 3)
                self.row['handover_height'] = round(self.last_offset.height, 3)
        elif self.state == 'SETTLE':
            stopped = self.armed is False and self.time_in_state(now) > SETTLE_MIN
            if stopped or self.time_in_state(now) > SETTLE_MAX:
                self.end_trial(now)
        else:
            raise NotImplementedError(f'no handler for state {self.state}')

    # ---- the extra mission states ------------------------------------------------------

    def go_to_start_point(self, now):
        """POSITION: move until the pad appears at the chosen offset."""
        seconds = now.nanoseconds * 1e-9
        offset, self.new_offset = self.new_offset, None
        if offset is not None:
            self.last_seen = seconds
            error = PadOffset(offset.forward - self.target.forward, offset.left - self.target.left, offset.height)
            self.last_position_command = horizontal_command(error, self.landing_params)
            if error.horizontal > IN_POSITION:
                self.in_position_since = None
            elif self.in_position_since is None:
                self.in_position_since = seconds
            elif seconds - self.in_position_since >= IN_POSITION_HOLD:
                self.enter('MEASURE', now)
                return
        if self.last_position_command is None or seconds - (self.last_seen or -math.inf) > MARKER_STALE:
            self.send_body_velocity(now)                      # no marker: hover and wait
        else:
            command = self.last_position_command
            self.send_body_velocity(now, forward=command.forward, left=command.left)

    def measure_start_point(self, now):
        """MEASURE: hover, then record the true start position and what the detector says."""
        self.send_body_velocity(now)
        if self.time_in_state(now) < MEASURE_SETTLE:
            return
        pose = gz_model_pose(self.model)
        if pose is not None:
            self.row.update(start_x=round(pose.x, 3), start_y=round(pose.y, 3), start_z=round(pose.z, 3),
                            start_distance=round(math.hypot(pose.x - self.pad_x, pose.y - self.pad_y), 3),
                            true_height_above_pad=round(pose.z - self.pad_top, 3))
        if self.last_offset is not None:
            self.row.update(detector_range=round(self.last_offset.height, 3),
                            detector_offset=round(self.last_offset.horizontal, 3))
        self.new_offset = None          # the approach starts from a fresh detection
        self.approach_started = now.nanoseconds * 1e-9
        self.enter('APPROACH', now)


def main():
    run(LandingTrials)


if __name__ == '__main__':
    main()
