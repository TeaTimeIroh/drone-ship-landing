"""Take off, find the landing pad with the camera, fly over it and land on it.

Needs the simulator and the marker detector running:

    ros2 run ship_landing marker_detector --ros-args -p use_sim_time:=true
    ros2 run ship_landing land_on_pad --ros-args -p use_sim_time:=true

Listens to landing_pad/pose from the detector and commands velocities in the drone's
own frame. Below about 1 m, where the marker no longer fits in the image, it hands
over to the autopilot's LAND mode for the last stretch.
"""
from geometry_msgs.msg import PoseStamped

from ship_landing.flight_base import FlightNode, run
from ship_landing.landing_control import LandingParams, PadLander, offset_from_camera

LOG_PERIOD = 2.0    # s of simulation time between progress lines


class LandOnPad(FlightNode):
    STATE_TIMEOUT = dict(FlightNode.STATE_TIMEOUT, APPROACH=240.0)
    AIRBORNE_STATES = ('CLIMB', 'APPROACH')
    FIRST_MISSION_STATE = 'APPROACH'

    def __init__(self, name='land_on_pad'):
        super().__init__(name)
        defaults = LandingParams()
        self.landing_params = LandingParams(
            gain=float(self.declare_parameter('gain', defaults.gain).value),
            max_speed=float(self.declare_parameter('max_speed', defaults.max_speed).value),
            descent_fast=float(self.declare_parameter('descent_fast', defaults.descent_fast).value),
            descent_slow=float(self.declare_parameter('descent_slow', defaults.descent_slow).value),
            handover_height=float(
                self.declare_parameter('handover_height', defaults.handover_height).value),
        )
        self.lander = None
        self.lander_state = None
        self.new_offset = None      # a detection not yet given to the lander
        self.last_offset = None     # the most recent detection, for log lines
        self.last_log = None
        self.create_subscription(PoseStamped, 'landing_pad/pose', self.on_pad, 10)

    def on_pad(self, msg):
        p = msg.pose.position
        self.new_offset = self.last_offset = offset_from_camera(p.x, p.y, p.z)

    def describe(self):
        if self.last_offset is None:
            return 'pad not seen yet'
        o = self.last_offset
        return (f'pad {o.forward:+.2f} m ahead, {o.left:+.2f} m left, '
                f'{o.height:.2f} m below, {o.horizontal:.2f} m off centre')

    def fly(self, now):
        self.approach(now)

    def approach(self, now):
        """The APPROACH state: everything between the climb and the autopilot's final landing."""
        seconds = now.nanoseconds * 1e-9
        if self.lander is None:
            self.lander = PadLander(self.landing_params)

        # Give each detection to the lander once; between camera frames it coasts.
        offset, self.new_offset = self.new_offset, None
        command = self.lander.update(seconds, offset)

        if self.lander.state != self.lander_state:
            self.get_logger().info(
                f'pad landing: {self.lander_state or "start"} -> {self.lander.state} | {self.describe()}')
            self.lander_state = self.lander.state
            self.last_log = seconds
        elif seconds - self.last_log >= LOG_PERIOD:
            self.get_logger().info(f'pad landing: {self.lander.state} | {self.describe()}')
            self.last_log = seconds

        if self.lander.state == 'HANDOVER':
            note = f' ({self.lander.reason})' if self.lander.reason else ''
            self.get_logger().info(f'Handing over to LAND mode{note}')
            self.enter('LAND', now)
        elif self.lander.state == 'FAILED':
            self.fail(f'Pad landing failed: {self.lander.reason}', now)
        else:
            self.send_body_velocity(now, forward=command.forward, left=command.left, up=command.up)


def main():
    run(LandOnPad)


if __name__ == '__main__':
    main()
