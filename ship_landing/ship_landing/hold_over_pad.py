"""Hold the drone over the pad by leaning it, with the camera as the only guide.

Needs a simulator running, static or moving pad:

    ros2 run ship_landing hold_over_pad --ros-args -p use_sim_time:=true -p output:=results/hold
    python3 tools/analyze_hold.py results/hold

The drone takes off as usual. Once the camera tracker has the pad and the autopilot
confirms it is taking our stick positions, the node switches to ALT_HOLD and flies
by lean angle alone (lean_control.py) for `hold_time` seconds: no position and no
speed from the autopilot, only its attitude. It then goes back to GUIDED, returns to
where it took off and lands there. It does not land on the pad yet.

The true positions of drone and pad from Gazebo are written next to the tracker's
view of them, for the analysis.
"""
import csv
import math
from pathlib import Path

from ardupilot_msgs.srv import ModeSwitch
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from ship_landing.flight_base import GUIDED, run
from ship_landing.land_on_pad import CAMERA_DOWN, CAMERA_FORWARD, CAMERA_LEFT
from ship_landing.lean_control import HoldParams, PadHold, StickMap
from ship_landing.levelling import AttitudeBuffer, heading, pad_in_world
from ship_landing.relative_tracking import RelativeTrack, acceleration_from_lean
from ship_landing.stick_base import ALT_HOLD, StickNode

PAD_TOP = 0.12
RECORD_PERIOD = 0.05


def seconds_of(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class HoldOverPad(StickNode):
    STATE_TIMEOUT = dict(StickNode.STATE_TIMEOUT, HOLD=240.0)
    AIRBORNE_STATES = ('CLIMB', 'HOLD')
    FIRST_MISSION_STATE = 'HOLD'

    def __init__(self, name='hold_over_pad'):
        super().__init__(name)
        self.hold_time = float(self.declare_parameter('hold_time', 40.0).value)
        self.output = Path(str(self.declare_parameter('output', 'results/hold').value))
        self.csv_name = str(self.declare_parameter('csv_name', 'hold.csv').value)
        self.pad_origin = (float(self.declare_parameter('pad_x', 4.0).value),
                           float(self.declare_parameter('pad_y', 3.0).value))
        defaults = HoldParams()
        self.hold = PadHold(HoldParams(
            closing_gain=float(self.declare_parameter('closing_gain', defaults.closing_gain).value),
            speed_gain=float(self.declare_parameter('speed_gain', defaults.speed_gain).value),
            max_lean=float(self.declare_parameter('max_lean', defaults.max_lean).value)), StickMap())
        self.camera = (CAMERA_FORWARD, CAMERA_LEFT, CAMERA_DOWN)
        self.track = RelativeTrack()
        self.attitudes = AttitudeBuffer()
        self.attitude = None
        self.position = None             # the autopilot's own position, used only to fly back afterwards
        self.home = None
        self.accel = (0.0, 0.0)
        self.command = None
        self.pad_joints = (0.0, 0.0)
        self.phase = 'acquire'           # acquire, to_alt_hold, hold, to_guided, retreat
        self.phase_started = None
        self.sticks_ok_since = None
        self.start_alt = None
        self.abort_reason = None
        self.rows = 0
        self.last_row = None

        self.output.mkdir(parents=True, exist_ok=True)
        self.file = open(self.output / self.csv_name, 'w', newline='')
        self.log = csv.writer(self.file)
        self.log.writerow(['time', 'phase', 'east', 'north', 'velocity_east', 'velocity_north',
                           'pad_accel_east', 'pad_accel_north', 'age', 'true_east', 'true_north', 'true_height',
                           'roll_stick', 'pitch_stick', 'lean_forward', 'lean_left', 'height', 'throttle_stick',
                           'landing'])
        self.create_subscription(PoseStamped, 'landing_pad/pose', self.on_pad, 10)
        self.create_subscription(JointState, 'moving_pad/joint_states', self.on_pad_joints, 10)
        self.create_subscription(Odometry, '/odometry', self.on_truth, qos_profile_sensor_data)
        self.create_timer(0.05, self.control_tick)

    # ---- inputs ------------------------------------------------------------------------

    def on_pose(self, msg):
        super().on_pose(msg)
        p, o = msg.pose.position, msg.pose.orientation
        self.position = (p.x, p.y, p.z)
        self.attitude = (o.x, o.y, o.z, o.w)
        self.attitudes.add(seconds_of(msg.header.stamp), self.attitude)

    def on_pad(self, msg):
        stamp = seconds_of(msg.header.stamp)
        q = self.attitudes.at(stamp)
        if q is None:
            return
        p = msg.pose.position
        east, north, up = pad_in_world(p.x, p.y, p.z, (0.0, 0.0, 0.0), q, *self.camera)
        self.track.update(stamp, east, north, -up)

    def on_pad_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if 'slide_east' in positions and 'slide_north' in positions:
            self.pad_joints = (positions['slide_east'], positions['slide_north'])

    def on_truth(self, msg):
        t = seconds_of(msg.header.stamp)
        if self.state != 'HOLD' or self.file.closed or self.track.time is None:
            return
        if self.last_row is not None and t - self.last_row < RECORD_PERIOD:
            return
        self.last_row = t
        p = msg.pose.pose.position
        pad = (self.pad_origin[0] + self.pad_joints[0], self.pad_origin[1] + self.pad_joints[1])
        track, c = self.track, self.command
        self.log.writerow([f'{t:.3f}', self.phase] + [f'{v:.4f}' for v in (
            track.position[0], track.position[1], track.velocity[0], track.velocity[1],
            track.pad_accel[0], track.pad_accel[1], min(track.age(t), 99.0),
            pad[0] - p.x, pad[1] - p.y, p.z - PAD_TOP, self.sticks[0], self.sticks[1],
            c.lean_forward if c else 0.0, c.lean_left if c else 0.0,
            track.height if track.height is not None else 0.0, self.sticks[2])] + [self.landing_label()])
        self.rows += 1

    # ---- control -----------------------------------------------------------------------

    def control_tick(self):
        """Twenty times a second: move the tracker on, and while holding, send the sticks."""
        if self.attitude is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.accel = acceleration_from_lean(self.attitude)
        self.track.predict(now, self.accel)
        if self.state != 'HOLD' or self.phase != 'hold':
            return
        self.send_sticks(*self.steer(now))

    def landing_label(self):
        return ''

    def steer(self, now):
        """(roll, pitch, throttle) sticks for this moment. Here: hold over the pad at the present height."""
        if self.track.age(now) > 1.0:
            self.command = None
            return (0.0, 0.0, 0.0)           # pad not seen: level the drone and wait
        self.command = self.hold.command(self.track, heading(self.attitude))
        return (self.command.roll_stick, self.command.pitch_stick, 0.0)

    def next_phase(self, phase, seconds):
        self.get_logger().info(f'hold: {self.phase} -> {phase}')
        self.phase, self.phase_started = phase, seconds
        self.pending = None

    def abort(self, reason, seconds):
        self.abort_reason = reason
        self.get_logger().error(reason + '. Going back to GUIDED.')
        self.next_phase('to_guided', seconds)

    def fly(self, now):
        seconds = now.nanoseconds * 1e-9
        if self.phase_started is None:
            self.phase_started = seconds
            self.home = self.position
        elapsed = seconds - self.phase_started
        track = self.track
        if self.extra_phase(seconds, elapsed, now):
            return

        if self.phase == 'acquire':
            # Hover on a zero-velocity command with centred sticks already being sent,
            # until the tracker has the pad and the autopilot confirms the sticks.
            self.send_map_velocity(now)
            self.send_sticks()
            self.start_alt = self.alt
            ready = self.sticks_in_force() and track.ready and track.age(seconds) < 0.5
            if not ready:
                self.sticks_ok_since = None
            elif self.sticks_ok_since is None:
                self.sticks_ok_since = seconds
            if self.sticks_ok_since is not None and seconds - self.sticks_ok_since >= 1.0 and elapsed >= 3.0:
                self.get_logger().info(
                    f'Pad {math.hypot(*track.position):.2f} m away, {track.height:.2f} m below. '
                    f'Sticks accepted: {self.describe_rc()}')
                self.next_phase('to_alt_hold', seconds)
            elif elapsed >= 20.0:
                why = ('the pad is not being tracked' if not track.ready or track.age(seconds) >= 0.5
                       else f'the autopilot is not using our sticks ({self.describe_rc()})')
                self.fail(f'Cannot start the hold: {why}. Staying in GUIDED and landing.', now)
        elif self.phase == 'to_alt_hold':
            self.send_sticks()
            if self.mode == ALT_HOLD:
                self.next_phase('hold', seconds)
            else:
                self.call(self.mode_cli, ModeSwitch.Request(mode=ALT_HOLD))
                if elapsed > 10.0:
                    self.fail('The autopilot did not switch to ALT_HOLD', now)
        elif self.phase == 'hold':
            self.watch(seconds, elapsed, now)       # the sticks themselves are sent by control_tick
        elif self.phase == 'to_guided':
            self.send_sticks()
            self.send_map_velocity(now)
            if self.mode == GUIDED:
                self.next_phase('retreat', seconds)
            else:
                self.call(self.mode_cli, ModeSwitch.Request(mode=GUIDED))
                if elapsed > 10.0:
                    self.fail('The autopilot did not switch back to GUIDED', now)
        else:
            # Fly back to where the hold began, clear of the pad, and land there.
            east, north = self.home[0] - self.position[0], self.home[1] - self.position[1]
            distance = math.hypot(east, north)
            if elapsed < 2.0:
                self.send_map_velocity(now)
            elif distance > 0.7 and elapsed < 30.0:
                speed = min(1.5, 0.6 * distance)
                self.send_map_velocity(now, east=speed * east / distance, north=speed * north / distance)
            else:
                self.send_map_velocity(now)
                if self.abort_reason:
                    self.fail('Hold ended early: ' + self.abort_reason, now)
                else:
                    self.enter('LAND', now)

    def extra_phase(self, seconds, elapsed, now):
        """For nodes built on this one: handle phases of their own. True if handled."""
        return False

    def watch(self, seconds, elapsed, now):
        """Called ten times a second during the hold: report, end it, or abort it."""
        track = self.track
        gap = math.hypot(*track.position)
        if self.alt is not None and abs(self.alt - self.start_alt) > 1.0:
            self.abort(f'Height changed by {self.alt - self.start_alt:+.1f} m during the hold', seconds)
        elif track.age(seconds) > 2.5:
            self.abort('The pad was not seen for 2.5 s', seconds)
        elif gap > 8.0:
            self.abort(f'The pad is {gap:.1f} m away', seconds)
        elif elapsed >= self.hold_time:
            self.next_phase('to_guided', seconds)
        elif int(elapsed / 2.0) != int((elapsed - 0.1) / 2.0):
            c = self.command
            self.get_logger().info(
                f'hold: pad {track.position[0]:+.2f} m east, {track.position[1]:+.2f} m north, '
                f'relative speed {track.speed():.2f} m/s, lean '
                f'{(c.lean_forward if c else 0.0):+.1f} deg forward, {(c.lean_left if c else 0.0):+.1f} deg left')

    def on_touchdown(self, now):
        self.finish()

    def finish(self):
        if not self.file.closed:
            self.file.close()
            self.get_logger().info(f'Recorded {self.rows} rows in {self.output / self.csv_name}')
        super().finish()


def main():
    run(HoldOverPad)


if __name__ == '__main__':
    main()
