"""Drive the moving landing pad along a line or a circle.

Started by pad_sim.launch.py when the world is moving_pad. By hand:

    ros2 run ship_landing pad_mover --ros-args -p use_sim_time:=true -p pattern:=circle

Change the speed while it runs (0 stops the pad):

    ros2 param set /pad_mover speed 1.0

The node sends an east and a north speed to the pad's two sliding joints and reads the
true joint positions back from Gazebo, which it uses only to keep the pad on its path.
The path itself is in pad_motion.py.
"""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from ship_landing.pad_motion import MotionParams, PadMotion

PERIOD = 0.05            # s between commands
MAX_STEP = 0.2           # s, longest time step the path is advanced by at once
STATE_MAX_AGE = 1.0      # s, older joint positions than this are not used
REPORT_EVERY = 5.0       # s between progress lines
MAX_SPEED = 6.0          # m/s, a typing mistake should not launch the pad


class PadMover(Node):
    def __init__(self):
        super().__init__('pad_mover')
        params = MotionParams(
            pattern=str(self.declare_parameter('pattern', 'line').value),
            speed=float(self.declare_parameter('speed', 0.5).value),
            travel=float(self.declare_parameter('travel', 2.5).value),
            radius=float(self.declare_parameter('radius', 1.5).value),
            accel=float(self.declare_parameter('accel', 0.25).value),
            heading_deg=float(self.declare_parameter('heading', 0.0).value),
        )
        params.speed = min(abs(params.speed), MAX_SPEED)
        self.motion = PadMotion(params)
        self.add_on_set_parameters_callback(self.on_parameters)

        self.east_pub = self.create_publisher(Float64, 'moving_pad/cmd_vel_east', 10)
        self.north_pub = self.create_publisher(Float64, 'moving_pad/cmd_vel_north', 10)
        self.create_subscription(JointState, 'moving_pad/joint_states', self.on_joint_state, 10)

        self.measured = None         # (east, north) from Gazebo
        self.measured_at = None
        self.last_step = None
        self.last_report = None
        self.warned_no_state = False
        self.create_timer(PERIOD, self.tick)
        self.get_logger().info(
            f'Pad pattern: {params.pattern}, speed {params.speed:.2f} m/s. Waiting for simulation time.')

    def on_parameters(self, changes):
        for change in changes:
            if change.name == 'speed':
                speed = float(change.value)
                if not 0.0 <= speed <= MAX_SPEED:
                    return SetParametersResult(
                        successful=False, reason=f'speed must be between 0 and {MAX_SPEED} m/s')
                self.motion.params.speed = speed
                self.get_logger().info(f'Pad speed set to {speed:.2f} m/s')
            elif change.name != 'use_sim_time':
                return SetParametersResult(
                    successful=False,
                    reason='only speed can be changed while running; restart the node for the rest')
        return SetParametersResult(successful=True)

    def on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if 'slide_east' in positions and 'slide_north' in positions:
            self.measured = (positions['slide_east'], positions['slide_north'])
            self.measured_at = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return                   # simulation time has not started
        if self.last_step is None:
            self.last_step = self.last_report = now
            return
        dt = (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        if dt <= 0.0:
            return
        dt = min(dt, MAX_STEP)

        measured = None
        if self.measured_at is not None and \
                (now - self.measured_at).nanoseconds * 1e-9 < STATE_MAX_AGE:
            measured = self.measured
        east, north = self.motion.step(dt, measured)
        self.publish(east, north)

        if (now - self.last_report).nanoseconds * 1e-9 >= REPORT_EVERY:
            self.last_report = now
            self.report(measured, east, north)

    def publish(self, east, north):
        self.east_pub.publish(Float64(data=float(east)))
        self.north_pub.publish(Float64(data=float(north)))

    def report(self, measured, east, north):
        speed = (east ** 2 + north ** 2) ** 0.5
        if measured is not None:
            self.get_logger().info(
                f'pad is {measured[0]:+.2f} m east, {measured[1]:+.2f} m north of its start point '
                f'(from Gazebo), commanded speed {speed:.2f} m/s')
            return
        if not self.warned_no_state:
            self.warned_no_state = True
            self.get_logger().warn(
                'No joint positions from Gazebo on moving_pad/joint_states. '
                'The pad is driven without feedback. Is the moving_pad world running?')
        expected = self.motion.reference
        self.get_logger().info(
            f'pad should be {expected[0]:+.2f} m east, {expected[1]:+.2f} m north of its start point '
            f'(not confirmed by Gazebo), commanded speed {speed:.2f} m/s')


def main():
    # Handle Ctrl+C here, so the pad can be told to stop before the node goes away.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):     # older rclpy: stopping the pad on exit may not get through
        rclpy.init()
    node = PadMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(3):
                node.publish(0.0, 0.0)
            node.get_logger().info('Pad stopped.')
        except Exception:        # the context may already be gone; nothing more to do
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
