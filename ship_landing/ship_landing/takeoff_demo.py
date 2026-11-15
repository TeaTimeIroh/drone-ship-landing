"""Autonomous takeoff, short velocity cruise and landing through ArduPilot's ROS 2 (DDS) interface.

Run with the simulator already up and the vehicle disarmed on the ground:

    ros2 run ship_landing takeoff_demo --ros-args -p use_sim_time:=true
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

# ArduCopter flight mode numbers
GUIDED, LAND = 4, 9

# Seconds of simulation time each state may take before the node gives up.
STATE_TIMEOUT = {
    'GUIDED': 10.0,
    'ARM': 30.0,
    'TAKEOFF': 10.0,
    'CLIMB': 30.0,
    'LAND': 10.0,
    'DESCEND': 60.0,
}
AIRBORNE_STATES = ('CLIMB', 'CRUISE')
RETRY_DELAY = 2.0
LANDED_ALT = 0.3


class TakeoffDemo(Node):
    def __init__(self):
        super().__init__('takeoff_demo')
        prefix = self.declare_parameter('prefix', '/ap/v1').value
        self.target_alt = self.declare_parameter('target_alt', 5.0).value
        self.cruise_speed = self.declare_parameter('cruise_speed', 1.0).value
        self.cruise_time = self.declare_parameter('cruise_time', 5.0).value

        self.alt = None
        self.state = 'GUIDED'
        self.state_entered = None
        self.pending = None
        self.wait_until = None
        self.failed = False

        self.create_subscription(PoseStamped, f'{prefix}/pose/filtered',
                                 self.on_pose, qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(TwistStamped, f'{prefix}/cmd_vel', 10)
        self.mode_cli = self.create_client(ModeSwitch, f'{prefix}/mode_switch')
        self.arm_cli = self.create_client(ArmMotors, f'{prefix}/arm_motors')
        self.takeoff_cli = self.create_client(Takeoff, f'{prefix}/experimental/takeoff')
        self.create_timer(0.1, self.step)

    def on_pose(self, msg):
        self.alt = msg.pose.position.z

    def call(self, client, request):
        """Send one request and return its result on a later tick, else None."""
        if self.pending is None:
            if client.service_is_ready():
                self.pending = client.call_async(request)
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
        self.wait_until = None

    def advance(self, ok, next_state, now):
        """Move on if the autopilot accepted the request, otherwise retry shortly."""
        if ok:
            self.enter(next_state, now)
        else:
            self.get_logger().warn(f'{self.state} refused, retrying in {RETRY_DELAY:.0f} s')
            self.wait_until = now + Duration(seconds=RETRY_DELAY)

    def finish(self):
        if self.failed:
            self.get_logger().error('Flight ended after a failure, see messages above')
        else:
            self.get_logger().info('Flight complete')
        raise SystemExit(1 if self.failed else 0)

    def handle_timeout(self, now):
        self.get_logger().error(f'{self.state} timed out')
        self.failed = True
        if self.state in AIRBORNE_STATES:
            self.enter('LAND', now)   # in the air: bring it down
        else:
            self.finish()             # on the ground, or landing itself failed

    def step(self):
        now = self.get_clock().now()
        if self.state_entered is None:
            self.state_entered = now

        limit = STATE_TIMEOUT.get(self.state)
        if limit is not None and now - self.state_entered > Duration(seconds=limit):
            self.handle_timeout(now)
            return
        if self.wait_until is not None and now < self.wait_until:
            return

        if self.state == 'GUIDED':
            r = self.call(self.mode_cli, ModeSwitch.Request(mode=GUIDED))
            if r is not None:
                self.advance(r.status, 'ARM', now)
        elif self.state == 'ARM':
            r = self.call(self.arm_cli, ArmMotors.Request(arm=True))
            if r is not None:
                self.advance(r.result, 'TAKEOFF', now)
        elif self.state == 'TAKEOFF':
            r = self.call(self.takeoff_cli, Takeoff.Request(alt=float(self.target_alt)))
            if r is not None:
                self.advance(r.status, 'CLIMB', now)
        elif self.state == 'CLIMB':
            if self.alt is not None and self.alt > self.target_alt - 0.5:
                self.enter('CRUISE', now)
        elif self.state == 'CRUISE':
            cmd = TwistStamped()
            cmd.header.stamp = now.to_msg()
            cmd.header.frame_id = 'map'
            cmd.twist.linear.x = float(self.cruise_speed)
            self.vel_pub.publish(cmd)
            if now - self.state_entered > Duration(seconds=float(self.cruise_time)):
                self.enter('LAND', now)
        elif self.state == 'LAND':
            r = self.call(self.mode_cli, ModeSwitch.Request(mode=LAND))
            if r is not None:
                self.advance(r.status, 'DESCEND', now)
        elif self.state == 'DESCEND':
            if self.alt is not None and self.alt < LANDED_ALT:
                self.get_logger().info(f'Touched down, altitude {self.alt:.2f} m')
                self.finish()


def main():
    rclpy.init()
    node = TakeoffDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
