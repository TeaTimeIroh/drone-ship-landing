"""Shared by the nodes that steer the drone with stick positions instead of velocities.

The autopilot's ALT_HOLD mode holds height and levels the drone, and takes what a
pilot's sticks would give: lean left or right, lean forward or back, climb or sink.
It needs no position or speed. ArduPilot's ROS interface accepts stick positions on
the `joy` topic and reports on `rc` what it currently reads and whether that is our
doing. A node hands over to ALT_HOLD only once `rc` shows our sticks in force, with
the throttle stick near the middle: left at the bottom, ALT_HOLD sinks at full rate.
"""
from ardupilot_msgs.msg import Rc
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

from ship_landing.flight_base import FlightNode

ALT_HOLD = 2


class StickNode(FlightNode):
    def __init__(self, name):
        super().__init__(name)
        self.sticks = (0.0, 0.0, 0.0)    # roll, pitch, throttle as last sent
        self.joy_pub = None
        self.rc_sub = None
        self.rc = None                   # latest stick channels as the autopilot sees them

    def on_rc(self, msg):
        self.rc = (list(msg.channels), list(msg.active_overrides))

    def sticks_in_force(self):
        """True if the autopilot reports our four sticks as overridden, throttle near the middle."""
        if self.rc is None:
            return False
        channels, overridden = self.rc
        return (len(channels) >= 4 and len(overridden) >= 4 and all(overridden[:4])
                and 1400 <= channels[2] <= 1600)

    def describe_rc(self):
        if self.rc is None:
            return 'no message on the rc topic'
        channels, overridden = self.rc
        return f'channels 1 to 4 read {channels[:4]}, overridden: {overridden[:4]}'

    def send_sticks(self, roll=0.0, pitch=0.0, throttle=0.0):
        """Stick positions from -1 to 1 on channels 1 to 4: roll, pitch, throttle, yaw.

        Roll positive lowers the right side (the drone goes right), pitch positive
        raises the nose (it goes backwards), throttle positive climbs. All measured in
        flight, see docs/stick_check.
        """
        if self.joy_pub is None:
            self.joy_pub = self.create_publisher(Joy, f'{self.prefix}/joy', 10)
            self.rc_sub = self.create_subscription(Rc, f'{self.prefix}/rc', self.on_rc, qos_profile_sensor_data)
        self.sticks = (roll, pitch, throttle)
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(roll), float(pitch), float(throttle), 0.0]
        self.joy_pub.publish(msg)
