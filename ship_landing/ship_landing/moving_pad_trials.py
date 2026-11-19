"""Land on the moving pad many times and record how accurately.

Needs the moving pad world running (pad_sim.launch.py world:=moving_pad):

    ros2 run ship_landing moving_pad_trials --ros-args -p use_sim_time:=true -p trials:=10

Each trial: take off, fly to a random point 1.5 to 4 m from the middle of the pad's
path, wait a random time of up to 12 s so that the pad is caught at a different point of
its path each time, then land on it exactly as land_on_moving_pad does. After the motors
stop, the pad is brought to rest and the drone's and the pad's true positions are read
from Gazebo. The next trial takes off from the resting pad and sets it moving again.

The autopilot reports its position relative to "home", and it moves home to wherever
the motors are armed. From the second trial on that is a spot on the pad, so the start
point cannot simply be handed to the autopilot. The node knows where the drone really
stands before each take-off (the spawn point, then the last measured touchdown) and
uses that to convert the start point into the autopilot's frame of the moment.

One CSV row is written per trial as it finishes. A trial that does not end with the
drone standing on the pad ends the run: the simulator is restarted before the next
batch, so nothing is left lying in the pad's way. tools/run_moving_trials.sh does that.
"""
import csv
import math
import random
from pathlib import Path

from ship_landing.flight_base import run
from ship_landing.land_on_moving_pad import LandOnMovingPad
from ship_landing.moving_trial_stats import COLUMNS, travel_frame
from ship_landing.pad_tracking import MovingPadLander

GOTO_GAIN = 0.8             # 1/s
GOTO_SPEED = 1.5            # m/s
IN_POSITION = 0.3           # m, close enough to the start point
IN_POSITION_HOLD = 1.5      # s to stay there


class MovingPadTrials(LandOnMovingPad):
    STATE_TIMEOUT = dict(LandOnMovingPad.STATE_TIMEOUT, CONNECT=240.0, READY=240.0, ARM=120.0,
                         POSITION=90.0, WAIT=60.0)
    AIRBORNE_STATES = ('CLIMB', 'POSITION', 'WAIT', 'TRACK', 'SETTLE')
    FIRST_MISSION_STATE = 'POSITION'

    def __init__(self):
        super().__init__('moving_pad_trials')
        self.measure = True
        self.trials = int(self.declare_parameter('trials', 10).value)
        self.seed = int(self.declare_parameter('seed', 0).value)
        self.batch = str(self.declare_parameter('batch', '').value) or f'seed{self.seed}'
        self.output = Path(self.declare_parameter('output', 'moving_pad_trials.csv').value)
        # Recorded with each result, and used to set the pad moving again after a measurement.
        self.pattern = str(self.declare_parameter('pattern', 'line').value)
        self.pad_speed = float(self.declare_parameter('pad_speed', 0.5).value)
        self.radius_min = float(self.declare_parameter('start_radius_min', 1.5).value)
        self.radius_max = float(self.declare_parameter('start_radius_max', 4.0).value)
        # One leg of the default line takes 12 s. Waiting up to that long makes every point
        # of the pad's path equally likely at touchdown, whenever the pad was set moving.
        self.max_wait = float(self.declare_parameter('max_wait', 12.0).value)
        # Where the drone stands in the Gazebo world before the first take-off.
        self.ground_world = (float(self.declare_parameter('spawn_x', 0.0).value),
                             float(self.declare_parameter('spawn_y', 0.0).value))

        self.rng = random.Random(self.seed)
        self.trial = 0
        self.errors = []
        self.row = None
        self.start = None                # start point in the Gazebo world
        self.start_local = None          # the same point in the autopilot's frame of this flight
        self.wait = 0.0
        self.in_position_since = None
        self.pad_restarted = False
        self.track_started = None

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
        self.start = (self.pad_origin[0] + radius * math.cos(bearing),
                      self.pad_origin[1] + radius * math.sin(bearing))
        self.wait = self.rng.uniform(0.0, self.max_wait)
        self.row = dict.fromkeys(COLUMNS, '')
        self.row.update(batch=self.batch, seed=self.seed, trial=self.trial, pattern=self.pattern,
                        pad_speed=self.pad_speed, start_east=round(self.start[0], 3),
                        start_north=round(self.start[1], 3), wait=round(self.wait, 2), land_mode=0)
        # Everything the landing itself keeps between calls starts afresh.
        self.lander = MovingPadLander(self.pad_params, camera_down=self.camera[2])
        self.lander_state = None
        self.new_detection = None
        self.detections = self.dropped = 0
        self.last_log = None
        self.next_disarm = None
        self.pad_stop_sent = False
        self.measure_at = None
        self.final_attempts = 0
        self.pad_velocity_at_commit = None
        self.contact = None
        self.stop_delay = None
        self.used_land_mode = False
        self.in_position_since = None
        self.pad_restarted = False
        self.track_started = None
        self.start_local = None
        self.get_logger().info(
            f'--- trial {self.trial}/{self.trials}: start point {self.start[0]:+.2f} m east, '
            f'{self.start[1]:+.2f} m north, then wait {self.wait:.1f} s')

    def write_row(self):
        with open(self.output, 'a', newline='') as handle:
            csv.DictWriter(handle, COLUMNS).writerow(self.row)

    def touchdown_measured(self, now, result):
        row = self.row
        row.update(final_attempts=self.final_attempts, detections=self.detections,
                   land_mode=int(self.used_land_mode))
        if self.stop_delay is not None:
            row['stop_delay'] = round(self.stop_delay, 2)
        contact = self.contact or {}
        if self.track_started is not None and 'time' in contact:
            row['track_time'] = round(contact['time'] - self.track_started, 2)
        velocity = contact.get('pad_velocity')
        if velocity is not None:
            row.update(pad_v_east=round(velocity[0], 3), pad_v_north=round(velocity[1], 3))
            if self.pad_velocity_at_commit is not None:
                row['pad_velocity_change'] = round(math.hypot(
                    velocity[0] - self.pad_velocity_at_commit[0], velocity[1] - self.pad_velocity_at_commit[1]), 3)
        estimate = contact.get('estimated_offset')
        if estimate is not None:
            # The landing logic keeps "pad relative to drone"; the results are "drone relative to pad".
            row.update(estimated_east=round(-estimate[0], 3), estimated_north=round(-estimate[1], 3))

        if result is None:
            row['reason'] = row['reason'] or 'no ground truth at touchdown'
            row['outcome'] = 'failed'
        else:
            self.ground_world = (result['drone_x'], result['drone_y'])    # the next take-off is from here
            row.update(error_east=round(result['east'], 4), error_north=round(result['north'], 4),
                       error=round(math.hypot(result['east'], result['north']), 4),
                       drone_z=round(result['drone_z'], 3))
            along, across = travel_frame(result['east'], result['north'], velocity, self.pad_velocity_at_commit)
            if along is not None:
                row.update(along=round(along, 4), across=round(across, 4))
            row['outcome'] = 'landed' if result['on_pad'] else 'missed'
            if not result['on_pad']:
                row['reason'] = row['reason'] or 'touched down off the pad'
        self.write_row()

        if row['outcome'] == 'landed':
            self.errors.append(row['error'])
            where = '' if row['along'] == '' else (
                f', {100 * abs(row["along"]):.1f} cm {"ahead of" if row["along"] > 0 else "behind"} the centre '
                f'along the pad\'s travel')
            self.get_logger().info(f'trial {self.trial}: landed {100 * row["error"]:.1f} cm from the pad centre{where}')
        else:
            self.get_logger().error(f'trial {self.trial}: {row["outcome"]}, {row["reason"]}')
        self.next_or_stop(now, clean=row['outcome'] == 'landed' and not self.used_land_mode)

    def next_or_stop(self, now, clean):
        if self.trial >= self.trials:
            self.wrap_up()
        elif not clean:
            self.get_logger().error(
                'The drone is not standing on the pad with its motors stopped, so this batch ends here')
            self.failed = True
            self.wrap_up()
        else:
            self.begin_trial()
            self.enter('GUIDED', now)

    def wrap_up(self):
        if self.errors:
            ordered = sorted(self.errors)
            self.get_logger().info(
                f'{len(ordered)} of {self.trial} trials landed on the pad: median '
                f'{100 * ordered[len(ordered) // 2]:.1f} cm, worst {100 * ordered[-1]:.1f} cm from its centre. '
                f'Results: {self.output}')
        self.finish()

    # ---- overrides ---------------------------------------------------------------------

    def fail(self, reason, now):
        """A failure in the air lands the drone where it is; either way the batch ends."""
        self.get_logger().error(reason)
        self.row['reason'] = self.row['reason'] or reason
        if self.state in self.AIRBORNE_STATES:
            self.enter('LAND', now)
        else:
            self.record_failure()

    def record_failure(self):
        self.row['outcome'] = 'failed'
        self.row.update(final_attempts=self.final_attempts, detections=self.detections)
        self.write_row()
        self.failed = True
        self.wrap_up()

    def on_touchdown(self, now):
        # Reached through LAND mode: either the autopilot would not stop the motors on
        # the pad, which can still be measured, or the trial failed in the air.
        if self.lander.state == 'CONTACT' and not self.row['reason']:
            self.enter('MEASURE', now)
        else:
            self.record_failure()

    def fly(self, now):
        if self.state == 'POSITION':
            self.go_to_start_point(now)
        elif self.state == 'WAIT':
            self.fly_towards(now, self.start_local)
            if self.time_in_state(now) >= self.wait:
                self.track_started = now.nanoseconds * 1e-9
                self.new_detection = None        # the landing starts from a fresh detection
                self.enter('TRACK', now)
        else:
            super().fly(now)

    # ---- the extra mission states ------------------------------------------------------

    def fly_towards(self, now, target):
        """Fly towards a point in the autopilot's frame; returns the distance left."""
        latest = self.poses.latest()
        if latest is None:
            self.send_map_velocity(now)
            return math.inf
        east, north = target[0] - latest[0][0], target[1] - latest[0][1]
        distance = math.hypot(east, north)
        speed = min(GOTO_SPEED, GOTO_GAIN * distance)
        if distance > 1e-6:
            self.send_map_velocity(now, east=speed * east / distance, north=speed * north / distance)
        else:
            self.send_map_velocity(now)
        return distance

    def go_to_start_point(self, now):
        """POSITION: set the pad moving again if it was stopped, and fly to the start point."""
        if not self.pad_restarted:
            self.pad_restarted = self.set_pad_speed(self.pad_speed)
            if not self.pad_restarted and self.time_in_state(now) > 10.0:
                self.fail('pad_mover is not running. Is the moving pad world running?', now)
                return
        if self.start_local is None:
            # The climb was straight up, so the drone is still above the spot it stood on.
            # That ties the autopilot's frame, whose origin moved when the motors were
            # armed, to the Gazebo world.
            latest = self.poses.latest()
            if latest is None:
                self.send_map_velocity(now)
                return
            shift = (self.ground_world[0] - latest[0][0], self.ground_world[1] - latest[0][1])
            self.start_local = (self.start[0] - shift[0], self.start[1] - shift[1])
            self.get_logger().info(
                f'The autopilot measures from a point {shift[0]:+.2f} m east, {shift[1]:+.2f} m north '
                f'of the world origin on this flight')
        seconds = now.nanoseconds * 1e-9
        if self.fly_towards(now, self.start_local) > IN_POSITION:
            self.in_position_since = None
        elif self.in_position_since is None:
            self.in_position_since = seconds
        elif seconds - self.in_position_since >= IN_POSITION_HOLD and self.pad_restarted:
            self.enter('WAIT', now)


def main():
    run(MovingPadTrials)


if __name__ == '__main__':
    main()
