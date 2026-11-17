"""Shared base for nodes that fly the drone through ArduPilot's ROS 2 (DDS) interface.

Handles the parts every mission has in common: finding the autopilot, switching to
GUIDED, arming, taking off, and landing, each with a timeout. A mission subclass adds
its own states between the climb and the landing by overriding `fly`.

State order:
    CONNECT -> READY -> GUIDED -> ARM -> TAKEOFF -> CLIMB -> (mission states) -> LAND -> TOUCHDOWN

READY waits for the autopilot's own pre-arm checks to pass. Without it, a node started
while the simulator is still booting can have its mode change overwritten by the
autopilot's start-up, arm in the wrong mode, and then have every takeoff refused.
"""
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from std_srvs.srv import Trigger

from ship_landing.ap_names import find_ap_prefix

# ArduCopter flight mode numbers
GUIDED, LAND = 4, 9

RETRY_DELAY = 2.0       # s, wait after the autopilot refuses a request
REPLY_TIMEOUT = 3.0     # s, resend a request that got no reply
LANDED_ALT = 0.3        # m above the take-off point
MAX_PREFLIGHT_RESTARTS = 8   # times the GUIDED/ARM/TAKEOFF sequence may start over


class FlightNode(Node):
    # Seconds of simulation time a state may take. Subclasses add their own states.
    STATE_TIMEOUT = {
        'CONNECT': 30.0,
        'READY': 180.0,
        'GUIDED': 10.0,
        'ARM': 60.0,
        'TAKEOFF': 20.0,
        'CLIMB': 30.0,
        'LAND': 10.0,
        'TOUCHDOWN': 60.0,
    }
    # States in which a timeout means "bring it down" rather than "give up on the ground".
    AIRBORNE_STATES = ('CLIMB',)
    # The state entered once the climb is complete.
    FIRST_MISSION_STATE = 'LAND'

    def __init__(self, name):
        super().__init__(name)
        # Empty means: find the autopilot's prefix ('/ap' or '/ap/v1') at start-up.
        self.prefix = self.declare_parameter('prefix', '').value
        self.target_alt = self.declare_parameter('target_alt', 5.0).value

        self.alt = None
        self.armed = None       # from the autopilot's status topic, None until heard
        self.mode = None        # flight mode number from the same topic
        self.preflight_restarts = 0
        self.state = 'CONNECT'
        self.state_entered = None
        self.ticks = 0
        self.pending = None
        self.pending_since = None
        self.wait_until = None
        self.failed = False
        self.vel_pub = None

        self.create_timer(0.1, self.step)
        # This one runs on wall-clock time, to notice a simulator that is not running.
        self.create_timer(5.0, self.check_clock, clock=Clock(clock_type=ClockType.STEADY_TIME))

    # ---- to be overridden by a mission -------------------------------------------------

    def fly(self, now):
        """Handle the mission's own states. Called every 0.1 s while in one of them."""
        raise NotImplementedError(f'no handler for state {self.state}')

    # ---- plumbing ----------------------------------------------------------------------

    def check_clock(self):
        if self.ticks == 0:
            self.get_logger().warn('No simulation time yet. Is the simulator running?')

    def connect(self, prefix):
        """Create the publishers, subscribers and service clients under the autopilot's prefix."""
        self.get_logger().info(f'Autopilot interface found at {prefix}')
        self.create_subscription(PoseStamped, f'{prefix}/pose/filtered',
                                 self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Status, f'{prefix}/status', self.on_status, qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(TwistStamped, f'{prefix}/cmd_vel', 10)
        self.mode_cli = self.create_client(ModeSwitch, f'{prefix}/mode_switch')
        self.arm_cli = self.create_client(ArmMotors, f'{prefix}/arm_motors')
        self.takeoff_cli = self.create_client(Takeoff, f'{prefix}/experimental/takeoff')
        self.prearm_cli = self.create_client(Trigger, f'{prefix}/prearm_check')

    def on_pose(self, msg):
        self.alt = msg.pose.position.z

    def on_status(self, msg):
        self.armed = bool(msg.armed)
        self.mode = int(msg.mode)

    def restart_preflight(self, why, now):
        """Go back to the mode switch and redo the arming sequence."""
        self.preflight_restarts += 1
        if self.preflight_restarts > MAX_PREFLIGHT_RESTARTS:
            self.fail(f'Could not get airborne: {why}', now)
            return
        self.get_logger().warn(f'{why}; starting the arming sequence again')
        self.enter('GUIDED', now)

    def on_touchdown(self, now):
        """Called once the drone is back on the ground. The default ends the flight."""
        self.finish()

    def call(self, client, request):
        """Send one request and return its result on a later tick, else None."""
        if self.pending is None:
            if client.service_is_ready():
                self.pending = client.call_async(request)
                self.pending_since = self.get_clock().now()
            return None
        if not self.pending.done():
            return None
        result, self.pending = self.pending.result(), None
        return result

    def enter(self, next_state, now):
        self.get_logger().info(f'{self.state} -> {next_state}')
        self.state = next_state
        self.state_entered = now
        self.pending = None
        self.pending_since = None
        self.wait_until = None

    def advance(self, ok, next_state, now):
        """Move on if the autopilot accepted the request, otherwise retry shortly."""
        if ok:
            self.enter(next_state, now)
        else:
            self.get_logger().warn(f'{self.state} refused, retrying in {RETRY_DELAY:.0f} s')
            self.wait_until = now + Duration(seconds=RETRY_DELAY)

    def time_in_state(self, now):
        return (now - self.state_entered).nanoseconds * 1e-9

    def fail(self, reason, now):
        """Abandon the mission: land if airborne, otherwise stop."""
        self.get_logger().error(reason)
        self.failed = True
        if self.state in self.AIRBORNE_STATES:
            self.enter('LAND', now)   # in the air: bring it down
        else:
            self.finish()             # on the ground, or landing itself failed

    def finish(self):
        if self.failed:
            self.get_logger().error('Flight ended after a failure, see messages above')
        else:
            self.get_logger().info('Flight complete')
        raise SystemExit(1 if self.failed else 0)

    def send_body_velocity(self, now, forward=0.0, left=0.0, up=0.0):
        """Command a velocity relative to the drone, in m/s.

        The autopilot drops the command after a few seconds, so keep sending it.

        ArduPilot's 'base_link' velocity takes +y as RIGHT, not left as ROS convention
        (REP-103) and the comment in its own source say. Found in flight: a command to
        go right sent the drone left. ArduCopter 4.8.0-dev, AP_DDS_ExternalControl.cpp.
        If a later ArduPilot corrects this, the sign below has to flip with it.
        """
        self._send_velocity(now, 'base_link', forward, -left, up)

    def send_map_velocity(self, now, east=0.0, north=0.0, up=0.0):
        """Command a velocity in the world frame, in m/s. Keep sending it."""
        self._send_velocity(now, 'map', east, north, up)

    def _send_velocity(self, now, frame, x, y, z):
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = frame
        cmd.twist.linear.x = float(x)
        cmd.twist.linear.y = float(y)
        cmd.twist.linear.z = float(z)
        self.vel_pub.publish(cmd)

    # ---- the state machine -------------------------------------------------------------

    def step(self):
        now = self.get_clock().now()
        self.ticks += 1
        if self.state_entered is None:
            self.state_entered = now

        limit = self.STATE_TIMEOUT.get(self.state)
        if limit is not None and self.time_in_state(now) > limit:
            if self.state == 'CONNECT':
                self.fail('No autopilot found: no mode_switch service appeared. '
                          'Is the simulator fully started?', now)
            else:
                self.fail(f'{self.state} timed out', now)
            return
        if self.wait_until is not None and now < self.wait_until:
            return
        if self.pending is not None and now - self.pending_since > Duration(seconds=REPLY_TIMEOUT):
            self.get_logger().warn(f'{self.state}: no reply, sending again')
            self.pending = None

        if self.state == 'CONNECT':
            prefix = self.prefix or find_ap_prefix(self)
            if prefix:
                self.connect(prefix)
                self.enter('READY', now)
        elif self.state == 'READY':
            r = self.call(self.prearm_cli, Trigger.Request())
            if r is not None:
                if r.success:
                    self.enter('GUIDED', now)
                else:
                    self.get_logger().info('Waiting for the autopilot to be ready to arm',
                                           throttle_duration_sec=10.0)
                    self.wait_until = now + Duration(seconds=RETRY_DELAY)
        elif self.state == 'GUIDED':
            r = self.call(self.mode_cli, ModeSwitch.Request(mode=GUIDED))
            if r is not None:
                self.advance(r.status, 'ARM', now)
        elif self.state == 'ARM':
            if self.pending is None and self.mode is not None and self.mode != GUIDED:
                self.restart_preflight(f'the autopilot is in mode {self.mode}, not GUIDED', now)
                return
            r = self.call(self.arm_cli, ArmMotors.Request(arm=True))
            if r is not None:
                self.advance(r.result, 'TAKEOFF', now)
        elif self.state == 'TAKEOFF':
            r = self.call(self.takeoff_cli, Takeoff.Request(alt=float(self.target_alt)))
            if r is not None:
                if r.status:
                    self.enter('CLIMB', now)
                elif self.mode is not None and self.mode != GUIDED:
                    self.restart_preflight(f'takeoff refused, the autopilot is in mode {self.mode}, not GUIDED', now)
                elif self.armed is False:
                    self.restart_preflight('takeoff refused, the motors are disarmed', now)
                else:
                    self.advance(False, 'CLIMB', now)
        elif self.state == 'CLIMB':
            self.preflight_restarts = 0
            if self.alt is not None and self.alt > self.target_alt - 0.5:
                self.enter(self.FIRST_MISSION_STATE, now)
        elif self.state == 'LAND':
            r = self.call(self.mode_cli, ModeSwitch.Request(mode=LAND))
            if r is not None:
                self.advance(r.status, 'TOUCHDOWN', now)
        elif self.state == 'TOUCHDOWN':
            if self.alt is not None and self.alt < LANDED_ALT:
                self.get_logger().info(f'Touched down, altitude {self.alt:.2f} m')
                self.on_touchdown(now)
        else:
            self.fly(now)


def run(node_class):
    """Standard main(): spin the node until it finishes or is interrupted."""
    rclpy.init()
    node = node_class()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
