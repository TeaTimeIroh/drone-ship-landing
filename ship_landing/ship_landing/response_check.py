"""Record how the drone answers velocity commands, in truth and as the autopilot sees it.

Needs a simulator running (any world):

    ros2 run ship_landing response_check --ros-args -p use_sim_time:=true -p output:=results/response_gps
    python3 tools/analyze_response.py results/response_gps

The drone takes off, then is told to fly east at a set speed, stop, fly north, stop,
and land. All the while the node writes the command, the true position that Gazebo
publishes on /odometry, and the autopilot's own position to response.csv. The
analysis turns that into how quickly the drone picks up speed and stops, and whether
the autopilot's picture of its own motion keeps up with the truth.

Run it once with GPS and once without to see what changes.
"""
import csv
from pathlib import Path

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from ship_landing.flight_base import FlightNode, run

RECORD_PERIOD = 0.05     # s between rows


class ResponseCheck(FlightNode):
    STATE_TIMEOUT = dict(FlightNode.STATE_TIMEOUT, RESPOND=120.0)
    AIRBORNE_STATES = ('CLIMB', 'RESPOND')
    FIRST_MISSION_STATE = 'RESPOND'

    def __init__(self):
        super().__init__('response_check')
        speed = float(self.declare_parameter('speed', 1.0).value)
        leg = float(self.declare_parameter('leg_time', 6.0).value)
        pause = float(self.declare_parameter('pause_time', 6.0).value)
        self.output = Path(str(self.declare_parameter('output', 'results/response').value))
        truth_topic = str(self.declare_parameter('truth_topic', '/odometry').value)

        # (seconds, east speed, north speed)
        self.plan = [(pause, 0.0, 0.0), (leg, speed, 0.0), (pause, 0.0, 0.0), (leg, 0.0, speed), (pause, 0.0, 0.0)]
        self.index = 0
        self.step_started = None
        self.command = (0.0, 0.0)
        self.position = None
        self.rows = 0
        self.last_row = None

        self.output.mkdir(parents=True, exist_ok=True)
        self.file = open(self.output / 'response.csv', 'w', newline='')
        self.log = csv.writer(self.file)
        self.log.writerow(['time', 'command_east', 'command_north', 'true_east', 'true_north', 'true_up',
                           'autopilot_east', 'autopilot_north', 'autopilot_up'])
        self.create_subscription(Odometry, truth_topic, self.on_truth, qos_profile_sensor_data)

    def on_pose(self, msg):
        super().on_pose(msg)
        p = msg.pose.position
        self.position = (p.x, p.y, p.z)

    def on_truth(self, msg):
        if self.state != 'RESPOND' or self.position is None or self.file.closed:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_row is not None and t - self.last_row < RECORD_PERIOD:
            return
        self.last_row = t
        p = msg.pose.pose.position
        self.log.writerow([f'{t:.3f}', *self.command] + [f'{v:.4f}' for v in (p.x, p.y, p.z, *self.position)])
        self.rows += 1

    def fly(self, now):
        seconds = now.nanoseconds * 1e-9
        if self.step_started is None:
            self.step_started = seconds
            duration, east, north = self.plan[self.index]
            self.get_logger().info(f'Commanding {east:+.1f} m/s east, {north:+.1f} m/s north for {duration:.0f} s')
        duration, east, north = self.plan[self.index]
        if seconds - self.step_started >= duration:
            self.index += 1
            self.step_started = None
            if self.index >= len(self.plan):
                self.command = (0.0, 0.0)
                self.send_map_velocity(now)
                self.enter('LAND', now)
            return
        self.command = (east, north)
        self.send_map_velocity(now, east=east, north=north)

    def on_touchdown(self, now):
        self.file.close()
        self.get_logger().info(f'Recorded {self.rows} rows in {self.output / "response.csv"}')
        if self.rows < 100:
            self.get_logger().warn('Few rows recorded. Is the true position being published on /odometry?')
        self.finish()

    def finish(self):
        if not self.file.closed:
            self.file.close()
        super().finish()


def main():
    run(ResponseCheck)


if __name__ == '__main__':
    main()
