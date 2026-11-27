"""Land on the pad, static or moving, steering by lean angle with the camera as the only guide.

Needs a simulator running:

    ros2 run ship_landing land_by_leaning --ros-args -p use_sim_time:=true -p output:=results/lean_landing

The drone takes off as usual. Once the camera tracker has the pad and the autopilot
confirms it is taking our stick positions, the node switches to ALT_HOLD and from
there to the pad everything is done with three sticks (lean_landing.py): lean left
or right, lean forward or back, and throttle for the way down. The autopilot keeps
the drone level and its height steady and is not asked where it is or how fast it
moves. On the pad the throttle stick goes to the bottom and the motors are stopped.

Afterwards the pad is stopped and the true touchdown position is read from Gazebo.
If the pad is lost for good on the way down, the node goes back to GUIDED, returns
to where it took off and lands there.
"""
import math
from collections import deque

from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

from ship_landing.flight_base import LAND, run
from ship_landing.ground_truth import gz_model_pose
from ship_landing.hold_over_pad import HoldOverPad
from ship_landing.lean_landing import LandingParams, LeanLander
from ship_landing.levelling import heading

DISARM_RETRY = 0.4          # s between requests to stop the motors
LAND_MODE_AFTER = 10.0      # s on the pad after which the autopilot's own LAND mode is asked to finish
PAD_STOP_TIME = 5.0         # s for the pad to come to rest before measuring
PAD_HALF = 0.625            # m, half the pad's side


class LandByLeaning(HoldOverPad):
    STATE_TIMEOUT = dict(HoldOverPad.STATE_TIMEOUT, HOLD=300.0)

    def __init__(self, name='land_by_leaning'):
        super().__init__(name)
        self.lander = LeanLander(LandingParams(hold=self.hold.params))
        self.lander_state = self.lander.state
        self.contact_since = None
        self.next_disarm = None
        self.stop_delay = None
        self.used_land_mode = False
        self.pad_stop_sent = False
        self.measure_at = None
        self.result = None
        self.pad_history = deque(maxlen=40)      # (time, east, north) of the pad's true slide, for the records only
        self.pad_velocity_at_commit = None
        self.contact = None
        self.pad_param_cli = self.create_client(SetParameters, 'pad_mover/set_parameters')

    def on_pad_joints(self, msg):
        super().on_pad_joints(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.pad_history or stamp - self.pad_history[-1][0] >= 0.02:
            self.pad_history.append((stamp,) + tuple(self.pad_joints))

    def true_pad_velocity(self):
        """The pad's true velocity over the last 0.3 s or so. For the records; the landing does not use it."""
        if len(self.pad_history) < 2:
            return None
        last = self.pad_history[-1]
        first = next((h for h in self.pad_history if last[0] - h[0] <= 0.3), self.pad_history[0])
        if last[0] - first[0] < 0.05:
            return (0.0, 0.0)
        return ((last[1] - first[1]) / (last[0] - first[0]), (last[2] - first[2]) / (last[0] - first[0]))

    # ---- steering: called twenty times a second while in the 'hold' phase -----------------

    def steer(self, now):
        lander = self.lander
        sticks = lander.update(now, self.track, heading(self.attitude))
        self.command = self.hold.command(self.track, heading(self.attitude)) if lander.state not in (
            'CONTACT', 'FAILED', 'CLIMB') else None
        if lander.state != self.lander_state:
            why = f' ({lander.reason})' if lander.reason else ''
            self.get_logger().info(f'landing: {self.lander_state} -> {lander.state}{why} | {self.describe()}')
            self.lander_state = lander.state
            if lander.state == 'COMMIT':
                self.pad_velocity_at_commit = self.true_pad_velocity()
            elif lander.state == 'CONTACT':
                self.contact = {'time': now, 'pad_velocity': self.true_pad_velocity(),
                                'estimated_offset': tuple(self.track.position)}
        return (sticks.roll, sticks.pitch, sticks.throttle)

    def landing_label(self):
        return self.lander.state if self.phase == 'hold' else self.phase

    def describe(self):
        track = self.track
        if track.height is None:
            return 'pad not tracked'
        return (f'pad {track.position[0]:+.2f} m east, {track.position[1]:+.2f} m north, '
                f'{track.height:.2f} m below the drone, relative speed {track.speed():.2f} m/s')

    # ---- supervision: called ten times a second -------------------------------------------

    def watch(self, seconds, elapsed, now):
        state = self.lander.state
        if state == 'FAILED':
            self.abort(f'Landing given up: {self.lander.reason}', seconds)
        elif state == 'CONTACT':
            self.contact_since = seconds
            self.next_phase('contact', seconds)
        elif elapsed > 180.0:
            self.abort('No landing after 180 s', seconds)
        elif math.hypot(*self.track.position) > 8.0:
            self.abort('The pad is more than 8 m away', seconds)
        elif int(elapsed / 3.0) != int((elapsed - 0.1) / 3.0):
            self.get_logger().info(f'landing: {state} | {self.describe()}')

    def extra_phase(self, seconds, elapsed, now):
        if self.phase == 'contact':
            self.wait_for_motors(seconds, elapsed, now)
            return True
        if self.phase == 'measure':
            self.measure(seconds, elapsed, now)
            return True
        return False

    def wait_for_motors(self, seconds, elapsed, now):
        """On the pad: sticks level, throttle at the bottom, and ask for the motors to stop until they do."""
        sticks = self.lander.contact_sticks(seconds)
        self.send_sticks(sticks.roll, sticks.pitch, sticks.throttle)
        if self.armed is False:
            self.stop_delay = elapsed
            self.get_logger().info(f'Motors stopped on the pad after {elapsed:.1f} s of contact')
            self.next_phase('measure', seconds)
            return
        if elapsed > LAND_MODE_AFTER and not self.used_land_mode:
            self.get_logger().warn('The autopilot has not accepted the landing; asking its LAND mode to finish')
            self.used_land_mode = True
        if self.used_land_mode:
            if self.mode != LAND:
                self.call(self.mode_cli, ModeSwitch.Request(mode=LAND))
            return
        if self.pending is None and self.next_disarm is not None and seconds < self.next_disarm:
            return
        reply = self.call(self.arm_cli, ArmMotors.Request(arm=False))
        if reply is not None:
            self.next_disarm = seconds + DISARM_RETRY     # refused while it still counts as flying; ask again

    def set_pad_speed(self, speed):
        if not self.pad_param_cli.service_is_ready():
            return False
        request = SetParameters.Request()
        request.parameters = [Parameter('speed', Parameter.Type.DOUBLE, float(speed)).to_parameter_msg()]
        self.pad_param_cli.call_async(request)
        return True

    def measure(self, seconds, elapsed, now):
        """Stop the pad if it moves, then compare the drone's and the pad's true positions."""
        if not self.pad_stop_sent:
            moving = self.set_pad_speed(0.0)
            if not moving and elapsed < 2.0:
                return                                  # give pad_mover a moment to show up
            self.pad_stop_sent = True
            self.measure_at = seconds + (PAD_STOP_TIME if moving else 1.0)
            if moving:
                self.get_logger().info('Stopping the pad to measure where the drone stands on it')
            return
        if seconds < self.measure_at:
            return
        drone = gz_model_pose('iris')
        if drone is None:
            self.get_logger().warn('Could not read the true position from Gazebo')
        else:
            east = drone.x - (self.pad_origin[0] + self.pad_joints[0])
            north = drone.y - (self.pad_origin[1] + self.pad_joints[1])
            on_pad = abs(east) < PAD_HALF and abs(north) < PAD_HALF and drone.z > 0.2
            self.result = {'east': east, 'north': north, 'on_pad': on_pad, 'drone_z': drone.z,
                           'drone_x': drone.x, 'drone_y': drone.y}
            self.get_logger().info(
                f'True touchdown position (Gazebo): {100 * east:+.1f} cm east, {100 * north:+.1f} cm north '
                f'of the pad centre, {100 * math.hypot(east, north):.1f} cm off, '
                f'{"on the pad" if on_pad else "NOT on the pad"} (drone height {drone.z:.2f} m); '
                f'final descents started: {self.lander.final_attempts}')
        self.landing_measured(now)

    def landing_measured(self, now):
        """Called once after the landing; self.result holds the true offset, or None."""
        self.finish()


def main():
    run(LandByLeaning)


if __name__ == '__main__':
    main()
