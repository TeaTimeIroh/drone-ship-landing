"""Compare where the autopilot thinks the drone is with where Gazebo says it is.

Needs a simulator running (any world):

    ros2 run ship_landing nav_check --ros-args -p use_sim_time:=true

The drone takes off, flies four legs at a commanded speed (north, east, south, west)
with a pause after each, hovers for a while on a zero-velocity command, and lands.
At each pause the node reads the true position from Gazebo and the autopilot's own
position, and at the end it prints, for every leg and for the hover:

- how far the drone really moved and how far the autopilot believes it moved,
- its true height above where it started and the height the autopilot reports.

With GPS the two should agree. Without GPS this shows how good the replacement is:
a velocity that reads low makes the drone fly further than commanded, and a
position that drifts shows up in the hover.
"""
import csv
import math
from pathlib import Path

from ship_landing.flight_base import FlightNode, run
from ship_landing.ground_truth import gz_model_pose

SETTLE = 6.0     # s of hover on a zero-velocity command before each reading


class NavCheck(FlightNode):
    STATE_TIMEOUT = dict(FlightNode.STATE_TIMEOUT, CHECK=400.0)
    AIRBORNE_STATES = ('CLIMB', 'CHECK')
    FIRST_MISSION_STATE = 'CHECK'

    def __init__(self):
        super().__init__('nav_check')
        speed = float(self.declare_parameter('speed', 1.0).value)
        leg_time = float(self.declare_parameter('leg_time', 6.0).value)
        hover_time = float(self.declare_parameter('hover_time', 60.0).value)
        output = str(self.declare_parameter('output', '').value)
        self.output = Path(output) if output else None
        self.model = str(self.declare_parameter('model', 'iris').value)

        # (label, seconds, east speed, north speed). After every step the drone is
        # given SETTLE seconds on a zero-velocity command and then a reading is taken.
        self.plan = [
            ('start', 0.0, 0.0, 0.0),
            ('north', leg_time, 0.0, speed),
            ('east', leg_time, speed, 0.0),
            ('south', leg_time, 0.0, -speed),
            ('west', leg_time, -speed, 0.0),
            (f'hover {hover_time:.0f} s', hover_time, 0.0, 0.0),
        ]
        self.index = 0
        self.step_started = None
        self.position = None
        self.readings = []       # (label, true x, y, z, autopilot east, north, up)

        self.ground = gz_model_pose(self.model)
        if self.ground is None:
            self.get_logger().error('Could not read the drone position from Gazebo. Is the simulator running?')

    def on_pose(self, msg):
        super().on_pose(msg)
        p = msg.pose.position
        self.position = (p.x, p.y, p.z)

    def fly(self, now):
        seconds = now.nanoseconds * 1e-9
        if self.ground is None or self.position is None:
            self.fail('No position to compare.', now)
            return
        if self.step_started is None:
            self.step_started = seconds
        label, duration, east, north = self.plan[self.index]
        elapsed = seconds - self.step_started
        if elapsed < duration:
            self.send_map_velocity(now, east=east, north=north)
            return
        self.send_map_velocity(now)
        if elapsed < duration + SETTLE:
            return

        truth = gz_model_pose(self.model)       # takes a moment; the drone is hovering
        if truth is None:
            self.fail('Could not read the true position from Gazebo.', now)
            return
        self.readings.append((label, truth.x, truth.y, truth.z, *self.position))
        self.get_logger().info(
            f'{label}: true ({truth.x:+.2f}, {truth.y:+.2f}, {truth.z:.2f}) m, '
            f'autopilot ({self.position[0]:+.2f}, {self.position[1]:+.2f}, {self.position[2]:.2f}) m')
        self.index += 1
        self.step_started = None
        if self.index >= len(self.plan):
            self.report()
            self.enter('LAND', now)

    def report(self):
        log = self.get_logger().info
        rows = []
        log('Step        | truly moved (east, north) | autopilot believes        | ratio | true height | autopilot height')
        for before, after in zip(self.readings, self.readings[1:]):
            label = after[0]
            true = (after[1] - before[1], after[2] - before[2])
            believed = (after[4] - before[4], after[5] - before[5])
            true_distance, believed_distance = math.hypot(*true), math.hypot(*believed)
            ratio = believed_distance / true_distance if true_distance > 0.3 else float('nan')
            true_height = after[3] - self.ground.z
            rows.append([label, *true, *believed, ratio, true_height, after[6]])
            log(f'{label:<11} | {true[0]:+6.2f}, {true[1]:+6.2f} m          | {believed[0]:+6.2f}, {believed[1]:+6.2f} m          '
                f'| {ratio:5.2f} | {true_height:6.2f} m    | {after[6]:6.2f} m')
        first, last = self.readings[0], self.readings[-1]
        log(f'Whole flight: truly ended {math.hypot(last[1] - first[1], last[2] - first[2]):.2f} m from the first reading, '
            f'the autopilot believes {math.hypot(last[4] - first[4], last[5] - first[5]):.2f} m.')
        log('Ratio is believed distance over true distance. Below 1 means the autopilot under-reads its movement.')
        if self.output is not None:
            self.output.mkdir(parents=True, exist_ok=True)
            with open(self.output / 'nav_check.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['step', 'true_east', 'true_north', 'believed_east', 'believed_north',
                                 'ratio', 'true_height', 'autopilot_height'])
                writer.writerows([[r[0]] + [f'{v:.3f}' for v in r[1:]] for r in rows])
            log(f'Saved to {self.output / "nav_check.csv"}')


def main():
    run(NavCheck)


if __name__ == '__main__':
    main()
