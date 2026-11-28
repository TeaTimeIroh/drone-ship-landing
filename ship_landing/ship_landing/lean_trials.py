"""Land on the moving pad by leaning, many times, and record how accurately.

Needs the moving pad world running (pad_sim.launch.py world:=moving_pad):

    ros2 run ship_landing lean_trials --ros-args -p use_sim_time:=true -p trials:=10

The trials are the ones moving_pad_trials flies, with the same random start points and
waits for the same seed, and the results are written in the same columns, so the two
ways of landing can be compared directly. Each trial: take off, fly to a point 1.5 to
4 m from the middle of the pad's path, wait up to 12 s, then land as land_by_leaning
does. Flying to the start point is done in GUIDED on the autopilot's own position;
from the switch to ALT_HOLD onwards the camera is the only guide.

A trial that does not end with the drone standing on the pad ends the batch.

On the boat (world:=ship, with -p ship_speed:=2.0 and the pad at 0 0 0) there are no
start points: each trial takes off from the pad with the boat at rest, holds over it,
gets the boat under way, follows it for the trial's random wait and lands.
"""
import csv
import math
import random
from pathlib import Path

from ship_landing.flight_base import run
from ship_landing.land_by_leaning import LandByLeaning
from ship_landing.lean_landing import LandingParams, LeanLander
from ship_landing.moving_trial_stats import COLUMNS, EARLY_CONTACT, travel_frame
from ship_landing.relative_tracking import RelativeTrack

GOTO_GAIN = 0.8             # 1/s
GOTO_SPEED = 1.5            # m/s
IN_POSITION = 0.3           # m, close enough to the start point
IN_POSITION_HOLD = 1.5      # s to stay there


class LeanTrials(LandByLeaning):
    STATE_TIMEOUT = dict(LandByLeaning.STATE_TIMEOUT, CONNECT=240.0, READY=240.0, ARM=120.0, HOLD=420.0)

    def __init__(self):
        super().__init__('lean_trials')
        self.trials = int(self.declare_parameter('trials', 10).value)
        self.seed = int(self.declare_parameter('seed', 0).value)
        self.batch = str(self.declare_parameter('batch', '').value) or f'seed{self.seed}'
        self.results = Path(self.declare_parameter('results', 'lean_trials.csv').value)
        self.pattern = str(self.declare_parameter('pattern', 'line').value)
        self.pad_speed = float(self.declare_parameter('pad_speed', 0.5).value)
        if self.ship_speed > 0.0:
            self.pad_speed = self.ship_speed
        self.radius_min = float(self.declare_parameter('start_radius_min', 1.5).value)
        self.radius_max = float(self.declare_parameter('start_radius_max', 4.0).value)
        self.max_wait = float(self.declare_parameter('max_wait', 12.0).value)
        # Where the drone stands in the Gazebo world before the first take-off.
        self.ground_world = (float(self.declare_parameter('spawn_x', 0.0).value),
                             float(self.declare_parameter('spawn_y', 0.0).value))
        self.rng = random.Random(self.seed)
        self.trial = 0
        self.errors = []
        self.detections = 0
        self.results.parent.mkdir(parents=True, exist_ok=True)
        with open(self.results, 'w', newline='') as handle:
            csv.DictWriter(handle, COLUMNS).writeheader()
        self.get_logger().info(f'{self.trials} trials, seed {self.seed}, results in {self.results}')
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
        # Everything a landing keeps between calls starts afresh.
        self.track = RelativeTrack()
        self.lander = LeanLander(LandingParams(hold=self.hold.params))
        self.lander_state = self.lander.state
        self.command = None
        self.contact = self.pad_velocity_at_commit = self.result = None
        self.contact_since = self.next_disarm = self.stop_delay = self.measure_at = None
        self.used_land_mode = self.pad_stop_sent = False
        self.abort_reason = None
        self.sticks_ok_since = None
        self.phase, self.phase_started = ('acquire' if self.ship_speed > 0.0 else 'position'), None
        self.stage = 'settle' if self.ship_speed > 0.0 else 'land'
        self.stage_since = self.settled_since = None
        self.cruise_time = self.wait
        self.start_local = None
        self.in_position_since = None
        self.pad_restarted = False
        self.landing_started = None
        self.detections = 0
        if self.ship_speed > 0.0:
            self.get_logger().info(
                f'--- trial {self.trial}/{self.trials}: take off from the boat, follow it at '
                f'{self.ship_speed:.1f} m/s for {self.wait:.1f} s, then land')
            return
        self.get_logger().info(
            f'--- trial {self.trial}/{self.trials}: start point {self.start[0]:+.2f} m east, '
            f'{self.start[1]:+.2f} m north, then wait {self.wait:.1f} s')

    def on_pad(self, msg):
        super().on_pad(msg)
        self.detections += 1

    def write_row(self):
        with open(self.results, 'a', newline='') as handle:
            csv.DictWriter(handle, COLUMNS).writerow(self.row)

    # ---- the two extra phases before the landing ----------------------------------------

    def extra_phase(self, seconds, elapsed, now):
        if self.phase == 'position':
            self.go_to_start_point(seconds, elapsed, now)
            return True
        if self.phase == 'wait':
            self.fly_towards(now, self.start_local)
            if elapsed >= self.wait:
                self.landing_started = seconds
                self.next_phase('acquire', seconds)
            return True
        return super().extra_phase(seconds, elapsed, now)

    def fly_towards(self, now, target):
        """Fly towards a point in the autopilot's frame; returns the distance left."""
        east, north = target[0] - self.position[0], target[1] - self.position[1]
        distance = math.hypot(east, north)
        speed = min(GOTO_SPEED, GOTO_GAIN * distance)
        if distance > 1e-6:
            self.send_map_velocity(now, east=speed * east / distance, north=speed * north / distance)
        else:
            self.send_map_velocity(now)
        return distance

    def go_to_start_point(self, seconds, elapsed, now):
        if not self.pad_restarted:
            self.pad_restarted = self.set_pad_speed(self.pad_speed)
            if not self.pad_restarted and elapsed > 10.0:
                self.fail('pad_mover is not running. Is the moving pad world running?', now)
                return
        if self.start_local is None:
            # The climb was straight up, so the drone is still above the spot it stood on.
            # That ties the autopilot's frame, whose origin moved when the motors were
            # armed, to the Gazebo world.
            shift = (self.ground_world[0] - self.position[0], self.ground_world[1] - self.position[1])
            self.start_local = (self.start[0] - shift[0], self.start[1] - shift[1])
        if self.fly_towards(now, self.start_local) > IN_POSITION:
            self.in_position_since = None
        elif self.in_position_since is None:
            self.in_position_since = seconds
        elif seconds - self.in_position_since >= IN_POSITION_HOLD and self.pad_restarted:
            self.next_phase('wait', seconds)
        if elapsed > 90.0:
            self.fail('Did not reach the start point in 90 s', now)

    # ---- results -----------------------------------------------------------------------

    def landing_measured(self, now):
        row, result, contact = self.row, self.result, self.contact or {}
        row.update(final_attempts=self.lander.final_attempts, detections=self.detections,
                   land_mode=int(self.used_land_mode))
        if self.stop_delay is not None:
            row['stop_delay'] = round(self.stop_delay, 2)
        if self.landing_started is not None and 'time' in contact:
            row['track_time'] = round(contact['time'] - self.landing_started, 2)
        velocity = contact.get('pad_velocity')
        if velocity is not None:
            row.update(pad_v_east=round(velocity[0], 3), pad_v_north=round(velocity[1], 3))
            if self.pad_velocity_at_commit is not None:
                row['pad_velocity_change'] = round(math.hypot(
                    velocity[0] - self.pad_velocity_at_commit[0], velocity[1] - self.pad_velocity_at_commit[1]), 3)
        estimate = contact.get('estimated_offset')
        if estimate is not None:
            # The tracker keeps "pad relative to drone"; the results are "drone relative to pad".
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
            elif self.lander.touched_early:
                row['reason'] = EARLY_CONTACT
        self.write_row()
        if row['outcome'] == 'landed':
            self.errors.append(row['error'])
            self.get_logger().info(f'trial {self.trial}: landed {100 * row["error"]:.1f} cm from the pad centre')
        else:
            self.get_logger().error(f'trial {self.trial}: {row["outcome"]}, {row["reason"]}')
        clean = row['outcome'] == 'landed' and not self.used_land_mode
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
                f'Results: {self.results}')
        self.finish()

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
        self.row.update(final_attempts=self.lander.final_attempts, detections=self.detections)
        self.write_row()
        self.failed = True
        self.wrap_up()

    def on_touchdown(self, now):
        self.record_failure()       # only reached through the autopilot's LAND mode after a failure in the air


def main():
    run(LeanTrials)


if __name__ == '__main__':
    main()
