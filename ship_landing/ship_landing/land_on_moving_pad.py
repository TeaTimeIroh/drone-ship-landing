"""Take off, follow the moving pad and land on it.

Needs the moving pad world running (pad_sim.launch.py world:=moving_pad):

    ros2 run ship_landing land_on_moving_pad --ros-args -p use_sim_time:=true

How it differs from land_on_pad:

- Each detection is corrected for the drone's tilt and placed in the autopilot's local
  frame, where a filter estimates the pad's position, velocity and acceleration.
- The drone is commanded to fly at the pad's velocity plus a correction for the offset.
- It descends all the way itself. The autopilot's LAND mode would land on a fixed spot.
- On the pad, it asks for the velocity the pad is already carrying it at, so that the
  autopilot has nothing to correct, and asks the autopilot to stop the motors. The
  autopilot only agrees once its own landing detector is satisfied, so the motors
  cannot be stopped in the air by mistake.

Afterwards it stops the pad and asks Gazebo where the drone and the pad really are.
The logic is in pad_tracking.py; this node moves messages in and out.
"""
import math
from collections import deque

from ardupilot_msgs.srv import ArmMotors
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from ship_landing.flight_base import FlightNode, run
from ship_landing.ground_truth import gz_model_pose
from ship_landing.land_on_pad import CAMERA_FORWARD, CAMERA_LEFT
from ship_landing.levelling import PoseBuffer, pad_in_world
from ship_landing.pad_tracking import MovingPadLander, MovingPadParams

CAMERA_DOWN = 0.10          # m, the camera sits roughly this far below the drone's centre
LOG_PERIOD = 2.0            # s between progress lines
DISARM_RETRY = 0.4          # s between requests to stop the motors
DISARM_GIVE_UP = 10.0       # s on the pad before falling back to LAND mode
PAD_STOP_TIME = 5.0         # s for the pad to come to rest before measuring
PAD_HALF = 0.625            # m, half the pad's side


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LandOnMovingPad(FlightNode):
    STATE_TIMEOUT = dict(FlightNode.STATE_TIMEOUT, TRACK=240.0, SETTLE=30.0, MEASURE=60.0)
    AIRBORNE_STATES = ('CLIMB', 'TRACK', 'SETTLE')
    FIRST_MISSION_STATE = 'TRACK'

    def __init__(self, name='land_on_moving_pad'):
        super().__init__(name)
        d = MovingPadParams()
        self.pad_params = MovingPadParams(
            gain=float(self.declare_parameter('gain', d.gain).value),
            lead_time=float(self.declare_parameter('lead_time', d.lead_time).value),
            max_closing_speed=float(self.declare_parameter('max_closing_speed', d.max_closing_speed).value),
            commit_height=float(self.declare_parameter('commit_height', d.commit_height).value),
            final_descent=float(self.declare_parameter('final_descent', d.final_descent).value),
        )
        self.camera = (float(self.declare_parameter('camera_forward', CAMERA_FORWARD).value),
                       float(self.declare_parameter('camera_left', CAMERA_LEFT).value),
                       float(self.declare_parameter('camera_down', CAMERA_DOWN).value))
        self.measure = bool(self.declare_parameter('measure', True).value)
        # Where the pad's start point is in the Gazebo world, for the measurement only.
        self.pad_origin = (float(self.declare_parameter('pad_x', 4.0).value),
                           float(self.declare_parameter('pad_y', 3.0).value))

        self.lander = MovingPadLander(self.pad_params, camera_down=self.camera[2])
        self.lander_state = None
        self.poses = PoseBuffer()
        self.new_detection = None
        self.detections = 0
        self.dropped = 0
        self.last_log = None
        self.last_command = (0.0, 0.0, 0.0)
        self.next_disarm = None
        self.pad_joints = None           # true (east, north) of the pad from its start point
        self.pad_history = deque()       # (time, east, north) of the last second, for its true velocity
        self.pad_stop_sent = False
        self.measure_at = None
        self.final_attempts = 0
        self.pad_velocity_at_commit = None
        self.contact = None              # what was known at the moment of contact
        self.stop_delay = None           # s from contact until the motors stopped
        self.used_land_mode = False

        self.create_subscription(PoseStamped, 'landing_pad/pose', self.on_pad, 10)
        self.create_subscription(JointState, 'moving_pad/joint_states', self.on_pad_joints, 10)
        self.pad_param_cli = self.create_client(SetParameters, 'pad_mover/set_parameters')

    # ---- inputs ----------------------------------------------------------------------

    def on_pose(self, msg):
        super().on_pose(msg)
        p, o = msg.pose.position, msg.pose.orientation
        self.poses.add(stamp_seconds(msg.header.stamp), (p.x, p.y, p.z), (o.x, o.y, o.z, o.w))

    def on_pad(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        pose = self.poses.at(stamp)
        if pose is None:
            self.dropped += 1            # no pose for the moment this image was taken
            return
        p = msg.pose.position
        east, north, up = pad_in_world(p.x, p.y, p.z, pose[0], pose[1], *self.camera)
        self.new_detection = (stamp, east, north, up)
        self.detections += 1

    def on_pad_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if 'slide_east' in positions and 'slide_north' in positions:
            self.pad_joints = (positions['slide_east'], positions['slide_north'])
            stamp = stamp_seconds(msg.header.stamp) or self.get_clock().now().nanoseconds * 1e-9
            if not self.pad_history or stamp > self.pad_history[-1][0]:
                self.pad_history.append((stamp, *self.pad_joints))
                while self.pad_history[0][0] < stamp - 1.0:
                    self.pad_history.popleft()

    def true_pad_velocity(self):
        """The pad's true (east, north) velocity over the last 0.3 s, from Gazebo's joint positions."""
        if len(self.pad_history) < 2:
            return None
        newest = self.pad_history[-1]
        oldest = next(row for row in self.pad_history if row[0] >= newest[0] - 0.3)
        dt = newest[0] - oldest[0]
        if dt < 0.1:
            return None
        return ((newest[1] - oldest[1]) / dt, (newest[2] - oldest[2]) / dt)

    def set_pad_speed(self, speed):
        """Ask pad_mover for a new speed. Returns False if pad_mover is not there."""
        if not self.pad_param_cli.service_is_ready():
            return False
        request = SetParameters.Request()
        request.parameters = [Parameter('speed', Parameter.Type.DOUBLE, float(speed)).to_parameter_msg()]
        self.pad_param_cli.call_async(request)
        return True

    # ---- flying ----------------------------------------------------------------------

    def describe(self):
        lander = self.lander
        if lander.offset is None:
            return f'pad not followed yet, {self.detections} detections'
        if lander.state == 'CONTACT':
            east, north = lander.own_velocity()
            return f'carried at {math.hypot(east, north):.2f} m/s, waiting for the autopilot to accept the landing'
        speed = math.hypot(*lander.track.velocity)
        return (f'pad {lander.offset[0]:+.2f} m east, {lander.offset[1]:+.2f} m north of the drone, '
                f'{lander.height:.2f} m below the camera, moving at {speed:.2f} m/s')

    def follow(self, now):
        """Run the landing logic once and send its command. Returns the logic's state."""
        seconds = now.nanoseconds * 1e-9
        latest = self.poses.latest()
        if latest is None:
            self.send_map_velocity(now)
            return self.lander.state
        detection, self.new_detection = self.new_detection, None
        command = self.lander.update(seconds, latest[0], detection)
        self.last_command = (command.east, command.north, command.up)

        if self.lander.state != self.lander_state:
            note = f' ({self.lander.reason})' if self.lander.reason else ''
            self.get_logger().info(
                f'moving pad: {self.lander_state or "start"} -> {self.lander.state}{note} | {self.describe()}')
            self.lander_state = self.lander.state
            self.last_log = seconds
            if self.lander.state == 'FINAL':
                self.final_attempts += 1
                self.pad_velocity_at_commit = self.true_pad_velocity()
            elif self.lander.state == 'CONTACT':
                self.contact = {
                    'time': seconds,
                    'pad_velocity': self.true_pad_velocity(),
                    'estimated_offset': self.lander.offset,
                }
        elif self.last_log is None or seconds - self.last_log >= LOG_PERIOD:
            self.get_logger().info(f'moving pad: {self.lander.state} | {self.describe()}')
            self.last_log = seconds
        self.send_map_velocity(now, east=command.east, north=command.north, up=command.up)
        return self.lander.state

    def fly(self, now):
        if self.state == 'TRACK':
            state = self.follow(now)
            if state == 'CONTACT':
                self.enter('SETTLE', now)
            elif state == 'FAILED':
                self.fail(f'Moving pad landing failed: {self.lander.reason}', now)
        elif self.state == 'SETTLE':
            self.settle(now)
        elif self.state == 'MEASURE':
            self.measure_touchdown(now)
        else:
            raise NotImplementedError(f'no handler for state {self.state}')

    def settle(self, now):
        """On the pad: keep pace with it and ask the autopilot to stop the motors until it agrees."""
        self.follow(now)
        seconds = now.nanoseconds * 1e-9
        if self.armed is False:
            self.motors_stopped(now)
            return
        if self.time_in_state(now) > DISARM_GIVE_UP:
            self.get_logger().warn('The autopilot would not stop the motors; switching to LAND mode')
            self.used_land_mode = True
            self.enter('LAND', now)
            return
        if self.pending is None and self.next_disarm is not None and seconds < self.next_disarm:
            return
        reply = self.call(self.arm_cli, ArmMotors.Request(arm=False))
        if reply is None:
            return
        if reply.result:
            self.motors_stopped(now)
        else:
            self.next_disarm = seconds + DISARM_RETRY     # still counted as flying; ask again shortly

    def on_touchdown(self, now):
        # Reached through LAND mode, after the autopilot would not stop the motors.
        if self.measure and self.lander.state == 'CONTACT':
            self.enter('MEASURE', now)
        else:
            self.finish()

    def motors_stopped(self, now):
        self.stop_delay = self.time_in_state(now)
        self.get_logger().info(f'Motors stopped on the pad after {self.stop_delay:.1f} s of contact')
        if self.measure:
            self.enter('MEASURE', now)
        else:
            self.finish()

    def measure_touchdown(self, now):
        """Stop the pad, then compare the drone's and the pad's true positions."""
        seconds = now.nanoseconds * 1e-9
        if not self.pad_stop_sent:
            if not self.set_pad_speed(0.0):
                if self.time_in_state(now) > 5.0:
                    self.get_logger().warn('pad_mover is not running, so the touchdown cannot be measured')
                    self.touchdown_measured(now, None)
                return
            self.pad_stop_sent = True
            self.measure_at = seconds + PAD_STOP_TIME
            self.get_logger().info('Stopping the pad to measure where the drone stands on it')
            return
        if seconds < self.measure_at:
            return
        drone = gz_model_pose('iris')
        if drone is None or self.pad_joints is None:
            self.get_logger().warn('Could not read the true positions from Gazebo')
            self.touchdown_measured(now, None)
            return
        pad_east = self.pad_origin[0] + self.pad_joints[0]
        pad_north = self.pad_origin[1] + self.pad_joints[1]
        east, north = drone.x - pad_east, drone.y - pad_north
        on_pad = abs(east) < PAD_HALF and abs(north) < PAD_HALF and drone.z > 0.2
        self.get_logger().info(
            f'True touchdown position (Gazebo): {100 * east:+.1f} cm east, {100 * north:+.1f} cm north '
            f'of the pad centre, {100 * math.hypot(east, north):.1f} cm off, '
            f'{"on the pad" if on_pad else "NOT on the pad"} (drone height {drone.z:.2f} m)')
        self.touchdown_measured(now, {'east': east, 'north': north, 'on_pad': on_pad, 'drone_z': drone.z,
                                      'drone_x': drone.x, 'drone_y': drone.y})

    def touchdown_measured(self, now, result):
        """Called once after the landing with the true offset from the pad centre, or None."""
        self.get_logger().info(f'{self.detections} detections used, {self.dropped} dropped for lack of a pose')
        self.finish()


def main():
    run(LandOnMovingPad)


if __name__ == '__main__':
    main()
