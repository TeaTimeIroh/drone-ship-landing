"""Record a flight that tilts the drone in view of the static pad, to check the tilt correction.

Needs the static pad world and the marker detector running (pad_sim.launch.py):

    ros2 run ship_landing tilt_check --ros-args -p use_sim_time:=true -p output:=results/tilt_check
    python3 tools/analyze_tilt.py results/tilt_check

The drone takes off, flies to a point 1.5 m west and 1 m south of the pad, and then
swings hard east-west and north-south, which makes it roll and pitch. Every marker
detection and every pose from the autopilot goes into a CSV file. The pad does not
move and its position is known, so the analysis can tell how wrong the camera's view
is while the drone is tilted, and whether rotating it by the autopilot's attitude
puts it right. See tilt_analysis.py.
"""
import csv
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped

from ship_landing.flight_base import FlightNode, run
from ship_landing.land_on_pad import CAMERA_FORWARD, CAMERA_LEFT
from ship_landing.landing_control import offset_from_camera
from ship_landing.levelling import AttitudeBuffer, levelled_offset, tilt

CAMERA_DOWN = 0.10       # m, the camera sits roughly this far below the drone's centre
GOTO_GAIN = 0.8          # 1/s
GOTO_SPEED = 1.0         # m/s
IN_POSITION = 0.25       # m
IN_POSITION_HOLD = 2.0   # s
LOG_PERIOD = 2.0         # s


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TiltCheck(FlightNode):
    STATE_TIMEOUT = dict(FlightNode.STATE_TIMEOUT, GOTO=90.0, SWING=120.0)
    AIRBORNE_STATES = ('CLIMB', 'GOTO', 'SWING')
    FIRST_MISSION_STATE = 'GOTO'

    def __init__(self):
        super().__init__('tilt_check')
        self.output = Path(str(self.declare_parameter('output', 'tilt_check').value))
        pad_x = float(self.declare_parameter('pad_x', 3.0).value)
        pad_y = float(self.declare_parameter('pad_y', 2.0).value)
        self.centre = (pad_x + float(self.declare_parameter('offset_east', -1.5).value),
                       pad_y + float(self.declare_parameter('offset_north', -1.0).value))
        speed = float(self.declare_parameter('swing_speed', 1.5).value)

        # (seconds, east speed, north speed); None means hold position at the centre.
        swing = [1.5, 3.0, 3.0, 3.0, 1.5]
        signs = [1, -1, 1, -1, 1]
        self.plan = [(4.0, None, None)]
        self.plan += [(t, s * speed, 0.0) for t, s in zip(swing, signs)]
        self.plan += [(4.0, None, None)]
        self.plan += [(t, 0.0, s * speed) for t, s in zip(swing, signs)]
        self.plan += [(4.0, None, None)]
        self.step_index = 0
        self.step_started = None

        self.position = None
        self.attitudes = AttitudeBuffer()
        self.in_position_since = None
        self.last_log = None
        self.last_line = 'pad not seen yet'
        self.counts = {'detections': 0, 'poses': 0}

        self.output.mkdir(parents=True, exist_ok=True)
        self.detection_file = open(self.output / 'detections.csv', 'w', newline='')
        self.pose_file = open(self.output / 'poses.csv', 'w', newline='')
        self.detection_log = csv.writer(self.detection_file)
        self.pose_log = csv.writer(self.pose_file)
        self.detection_log.writerow(['stamp', 'received', 'x', 'y', 'z'])
        self.pose_log.writerow(['stamp', 'received', 'east', 'north', 'up', 'qx', 'qy', 'qz', 'qw'])
        self.create_subscription(PoseStamped, 'landing_pad/pose', self.on_pad, 10)

    # ---- recording -------------------------------------------------------------------

    def recording(self):
        return self.state in ('GOTO', 'SWING')

    def on_pose(self, msg):
        super().on_pose(msg)
        p, o = msg.pose.position, msg.pose.orientation
        self.position = (p.x, p.y, p.z)
        stamp = stamp_seconds(msg.header.stamp)
        q = (o.x, o.y, o.z, o.w)
        self.attitudes.add(stamp, q)
        if self.recording():
            received = self.get_clock().now().nanoseconds * 1e-9
            self.pose_log.writerow([f'{stamp:.4f}', f'{received:.4f}'] + [f'{v:.5f}' for v in (*self.position, *q)])
            self.counts['poses'] += 1

    def on_pad(self, msg):
        p = msg.pose.position
        stamp = stamp_seconds(msg.header.stamp)
        if self.recording():
            received = self.get_clock().now().nanoseconds * 1e-9
            self.detection_log.writerow([f'{stamp:.4f}', f'{received:.4f}'] + [f'{v:.5f}' for v in (p.x, p.y, p.z)])
            self.counts['detections'] += 1
        raw = offset_from_camera(p.x, p.y, p.z, CAMERA_FORWARD, CAMERA_LEFT)
        q = self.attitudes.at(stamp)
        if q is None:
            self.last_line = (f'raw {raw.forward:+.2f} m ahead, {raw.left:+.2f} m left | '
                              'no attitude for this image')
            return
        fixed = levelled_offset(p.x, p.y, p.z, q, CAMERA_FORWARD, CAMERA_LEFT, CAMERA_DOWN)
        self.last_line = (f'raw {raw.forward:+.2f} m ahead, {raw.left:+.2f} m left | '
                          f'levelled {fixed.forward:+.2f} m ahead, {fixed.left:+.2f} m left | '
                          f'tilt {math.degrees(tilt(q)):.0f} deg')

    def close_logs(self):
        for f in (self.detection_file, self.pose_file):
            if not f.closed:
                f.close()

    # ---- flying ----------------------------------------------------------------------

    def step(self):
        # The check assumes the pad stands still. Refuse to take off in the moving pad world.
        # It may take a moment for that world's topics to be seen, so keep looking until the
        # recording starts.
        if self.state not in ('SWING', 'LAND', 'TOUCHDOWN'):
            topics = [name for name, _ in self.get_topic_names_and_types()]
            if '/moving_pad/joint_states' in topics:
                self.fail('This check needs the static pad world, and the moving pad world is running.',
                          self.get_clock().now())
                return
        super().step()

    def hold_centre(self, now):
        """Fly towards the centre point; returns the distance left."""
        if self.position is None:
            self.send_map_velocity(now)
            return float('inf')
        east = self.centre[0] - self.position[0]
        north = self.centre[1] - self.position[1]
        distance = math.hypot(east, north)
        speed = min(GOTO_SPEED, GOTO_GAIN * distance)
        if distance > 1e-6:
            self.send_map_velocity(now, east=speed * east / distance, north=speed * north / distance)
        else:
            self.send_map_velocity(now)
        return distance

    def fly(self, now):
        seconds = now.nanoseconds * 1e-9
        if self.last_log is None or seconds - self.last_log >= LOG_PERIOD:
            self.last_log = seconds
            self.get_logger().info(f'{self.state}: {self.last_line}')

        if self.state == 'GOTO':
            if self.hold_centre(now) < IN_POSITION:
                if self.in_position_since is None:
                    self.in_position_since = seconds
                elif seconds - self.in_position_since >= IN_POSITION_HOLD:
                    self.enter('SWING', now)
            else:
                self.in_position_since = None
            return

        # SWING: work through the plan.
        if self.step_started is None:
            self.step_started = seconds
        duration, east, north = self.plan[self.step_index]
        if seconds - self.step_started >= duration:
            self.step_index += 1
            self.step_started = seconds
            if self.step_index >= len(self.plan):
                self.enter('LAND', now)
                return
            duration, east, north = self.plan[self.step_index]
        if east is None:
            self.hold_centre(now)
        else:
            self.send_map_velocity(now, east=east, north=north)

    def on_touchdown(self, now):
        self.close_logs()
        self.get_logger().info(
            f'Recorded {self.counts["detections"]} detections and {self.counts["poses"]} poses in {self.output}')
        if self.counts['detections'] < 100:
            self.get_logger().warn('Few detections recorded. Was the marker in view?')
        self.finish()

    def finish(self):
        self.close_logs()
        super().finish()


def main():
    run(TiltCheck)


if __name__ == '__main__':
    main()
