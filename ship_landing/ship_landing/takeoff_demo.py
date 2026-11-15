import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

PREFIX = '/ap/v1'          
GUIDED, LAND = 4, 9
TARGET_ALT = 5.0


class TakeoffDemo(Node):
    def __init__(self):
        super().__init__('takeoff_demo')
        self.alt = 0.0
        self.state = 'GUIDED'
        self.pending = None
        self.wait_until = self.get_clock().now()
        self.cruise_end = None

        self.create_subscription(PoseStamped, f'{PREFIX}/pose/filtered',
                                 self.on_pose, qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(TwistStamped, f'{PREFIX}/cmd_vel', 10)
        self.mode_cli = self.create_client(ModeSwitch, f'{PREFIX}/mode_switch')
        self.arm_cli = self.create_client(ArmMotors, f'{PREFIX}/arm_motors')
        self.takeoff_cli = self.create_client(Takeoff, f'{PREFIX}/experimental/takeoff')
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

    def advance(self, ok, next_state):
        if ok:
            self.get_logger().info(f'{self.state} ok -> {next_state}')
            self.state = next_state
        else:
            self.get_logger().warn(f'{self.state} refused, retrying in 2 s')
            self.wait_until = self.get_clock().now() + Duration(seconds=2)

    def step(self):
        now = self.get_clock().now()
        if now < self.wait_until:
            return

        if self.state == 'GUIDED':
            r = self.call(self.mode_cli, ModeSwitch.Request(mode=GUIDED))
            if r is not None:
                self.advance(r.status, 'ARM')
        elif self.state == 'ARM':
            r = self.call(self.arm_cli, ArmMotors.Request(arm=True))
            if r is not None:
                self.advance(r.result, 'TAKEOFF')
        elif self.state == 'TAKEOFF':
            r = self.call(self.takeoff_cli, Takeoff.Request(alt=TARGET_ALT))
            if r is not None:
                self.advance(r.status, 'CLIMB')
        elif self.state == 'CLIMB':
            if self.alt > TARGET_ALT - 0.5:
                self.cruise_end = now + Duration(seconds=5)
                self.advance(True, 'CRUISE')
        elif self.state == 'CRUISE':
            cmd = TwistStamped()
            cmd.header.stamp = now.to_msg()
            cmd.header.frame_id = 'map'
            cmd.twist.linear.x = 1.0
            self.vel_pub.publish(cmd)
            if now > self.cruise_end:
                self.advance(True, 'LAND')
        elif self.state == 'LAND':
            r = self.call(self.mode_cli, ModeSwitch.Request(mode=LAND))
            if r is not None:
                self.advance(r.status, 'DONE')


def main():
    rclpy.init()
    rclpy.spin(TakeoffDemo())


if __name__ == '__main__':
    main()