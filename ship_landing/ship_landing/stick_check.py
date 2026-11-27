"""Fly the drone by stick positions sent from ROS, and record what it does.

Needs a simulator running (any world):

    ros2 run ship_landing stick_check --ros-args -p use_sim_time:=true -p output:=results/stick_check
    python3 tools/analyze_sticks.py results/stick_check

Everything so far has told the autopilot how fast to fly, which only works if the
autopilot knows its own horizontal speed. This check tries the other way to steer:
the autopilot's ALT_HOLD mode, which holds height and levels the drone but needs no
position or speed at all, with the stick positions a pilot would give coming from
this node over the `joy` topic.

The drone takes off as usual, hands over to ALT_HOLD with the sticks centred, gets
a short push and counter-push on the roll stick and on the pitch stick, a push on
the throttle stick, goes back to GUIDED and lands. The true motion from Gazebo's /odometry is written to
sticks.csv, and the analysis reports which way each stick leans and moves the drone
and by how much.
"""
import csv
import math
from pathlib import Path

from ardupilot_msgs.srv import ModeSwitch
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from ship_landing.flight_base import GUIDED, FlightNode, run
from ship_landing.stick_base import ALT_HOLD, StickNode

RECORD_PERIOD = 0.05     # s between rows
# (label, seconds, roll stick, pitch stick, throttle stick), each stick from -1 to 1.
# A centred throttle stick means "hold this height" in ALT_HOLD.
PLAN = [
    ('centred', 5.0, 0.0, 0.0, 0.0),
    ('roll', 1.5, 0.15, 0.0, 0.0),
    ('roll back', 1.5, -0.15, 0.0, 0.0),            # the opposite push, to stop again
    ('centred after roll', 3.0, 0.0, 0.0, 0.0),
    ('pitch', 1.5, 0.0, 0.15, 0.0),
    ('pitch back', 1.5, 0.0, -0.15, 0.0),
    ('centred after pitch', 3.0, 0.0, 0.0, 0.0),
    ('throttle', 2.0, 0.0, 0.0, 0.3),
    ('centred after throttle', 3.0, 0.0, 0.0, 0.0),
]


def calibration_plan():
    """Pushes of several sizes on the roll stick, and two on the pitch stick to compare."""
    plan = [('centred', 4.0, 0.0, 0.0, 0.0)]
    for stick in (0.1, 0.2, 0.3, 0.4):
        plan += [(f'roll {stick:+.2f}', 1.5, stick, 0.0, 0.0), (f'roll {-stick:+.2f}', 1.5, -stick, 0.0, 0.0),
                 (f'centred after roll {stick:.2f}', 2.0, 0.0, 0.0, 0.0)]
    for stick in (0.2, 0.4):
        plan += [(f'pitch {stick:+.2f}', 1.5, 0.0, stick, 0.0), (f'pitch {-stick:+.2f}', 1.5, 0.0, -stick, 0.0),
                 (f'centred after pitch {stick:.2f}', 2.0, 0.0, 0.0, 0.0)]
    return plan


class StickCheck(StickNode):
    STATE_TIMEOUT = dict(FlightNode.STATE_TIMEOUT, STICKS=180.0)
    AIRBORNE_STATES = ('CLIMB', 'STICKS')
    FIRST_MISSION_STATE = 'STICKS'

    def __init__(self):
        super().__init__('stick_check')
        self.output = Path(str(self.declare_parameter('output', 'results/stick_check').value))
        truth_topic = str(self.declare_parameter('truth_topic', '/odometry').value)
        # 'check': one push per stick. 'calibrate': several sizes, to map stick position to lean.
        self.plan = calibration_plan() if str(self.declare_parameter('plan', 'check').value) == 'calibrate' else PLAN
        self.phase = 'settle'        # settle, to_alt_hold, plan, to_guided, hold
        self.phase_started = None
        self.index = 0
        self.label = 'before'
        self.rows = 0
        self.last_row = None
        self.start_alt = None
        self.sticks_ok_since = None
        self.abort_reason = None

        self.output.mkdir(parents=True, exist_ok=True)
        self.file = open(self.output / 'sticks.csv', 'w', newline='')
        self.log = csv.writer(self.file)
        self.log.writerow(['time', 'step', 'mode', 'roll_stick', 'pitch_stick', 'throttle_stick',
                           'east', 'north', 'up', 'qx', 'qy', 'qz', 'qw'])
        self.create_subscription(Odometry, truth_topic, self.on_truth, qos_profile_sensor_data)

    def on_truth(self, msg):
        if self.state != 'STICKS' or self.file.closed:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_row is not None and t - self.last_row < RECORD_PERIOD:
            return
        self.last_row = t
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.log.writerow([f'{t:.3f}', self.label, self.mode, *self.sticks]
                          + [f'{v:.4f}' for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)])
        self.rows += 1

    def next_phase(self, phase, seconds):
        self.get_logger().info(f'sticks: {self.phase} -> {phase}')
        self.phase, self.phase_started = phase, seconds
        self.pending = None

    def fly(self, now):
        seconds = now.nanoseconds * 1e-9
        if self.phase_started is None:
            self.phase_started = seconds
            self.start_alt = self.alt
        elapsed = seconds - self.phase_started
        # Losing or gaining height it was not asked to: give the drone straight back to
        # GUIDED, which stops it, and end the experiment from there.
        if (self.phase in ('to_alt_hold', 'plan') and self.alt is not None and self.start_alt is not None
                and self.label != 'throttle' and abs(self.alt - self.start_alt) > 0.8):
            self.abort_reason = (f'Height changed by {self.alt - self.start_alt:+.1f} m with the throttle stick '
                                 f'centred ({self.describe_rc()})')
            self.get_logger().error(self.abort_reason)
            self.next_phase('to_guided', seconds)
            return

        if self.phase == 'settle':
            # Hover on a zero-velocity command, and start sending centred sticks so that
            # they are already in force at the moment ALT_HOLD takes over.
            self.send_map_velocity(now)
            self.send_sticks()
            self.start_alt = self.alt
            # Hand over only once the autopilot itself reports that it is using our sticks.
            if not self.sticks_in_force():
                self.sticks_ok_since = None
            elif self.sticks_ok_since is None:
                self.sticks_ok_since = seconds
            if self.sticks_ok_since is not None and seconds - self.sticks_ok_since >= 1.0 and elapsed >= 3.0:
                self.get_logger().info(f'sticks accepted by the autopilot: {self.describe_rc()}')
                self.next_phase('to_alt_hold', seconds)
            elif elapsed >= 8.0:
                self.fail(f'The autopilot is not using the stick positions sent on {self.prefix}/joy: '
                          f'{self.describe_rc()}. Staying in GUIDED and landing.', now)
        elif self.phase == 'to_alt_hold':
            self.send_sticks()
            if self.mode == ALT_HOLD:
                self.next_phase('plan', seconds)
            else:
                self.call(self.mode_cli, ModeSwitch.Request(mode=ALT_HOLD))
                if elapsed > 10.0:
                    self.fail('The autopilot did not switch to ALT_HOLD', now)
        elif self.phase == 'plan':
            label, duration, roll, pitch, throttle = self.plan[self.index]
            if self.label != label:
                self.label = label
                self.get_logger().info(f'sticks: {label} (roll {roll:+.2f}, pitch {pitch:+.2f}, '
                                       f'throttle {throttle:+.2f}) for {duration:.1f} s')
            self.send_sticks(roll, pitch, throttle)
            if throttle != 0.0:
                self.start_alt = self.alt        # a change of height that was asked for
            if elapsed >= duration:
                self.index += 1
                self.phase_started = seconds
                if self.index >= len(self.plan):
                    self.label = 'after'
                    self.next_phase('to_guided', seconds)
        elif self.phase == 'to_guided':
            self.send_sticks()
            self.send_map_velocity(now)
            if self.mode == GUIDED:
                self.next_phase('hold', seconds)
            else:
                self.call(self.mode_cli, ModeSwitch.Request(mode=GUIDED))
                if elapsed > 10.0:
                    self.fail('The autopilot did not switch back to GUIDED', now)
        else:
            self.send_map_velocity(now)
            if elapsed >= 3.0:
                if self.abort_reason:
                    self.fail('Stick check ended early: ' + self.abort_reason, now)
                else:
                    self.enter('LAND', now)

    def on_touchdown(self, now):
        self.finish()

    def finish(self):
        if not self.file.closed:
            self.file.close()
            self.get_logger().info(f'Recorded {self.rows} rows in {self.output / "sticks.csv"}')
        super().finish()


def main():
    run(StickCheck)


if __name__ == '__main__':
    main()
