"""Run the camera-only pad tracker alongside a flight and record it against the truth.

This node does not fly. Start it, then fly anything over the pad, and stop it with
Ctrl+C when the flight is over:

    ros2 run ship_landing relative_check --ros-args -p use_sim_time:=true -p output:=results/relative_check
    python3 tools/analyze_relative.py results/relative_check

It feeds relative_tracking.RelativeTrack with the detector's output and with the
drone's attitude, and nothing else from the autopilot: no position, no speed. Next
to the tracker's answer it writes where the pad truly is relative to the drone,
from Gazebo's /odometry and the pad's joint positions.
"""
import csv
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from ship_landing.ap_names import find_ap_prefix
from ship_landing.land_on_pad import CAMERA_DOWN, CAMERA_FORWARD, CAMERA_LEFT
from ship_landing.levelling import AttitudeBuffer, pad_in_world
from ship_landing.relative_tracking import RelativeTrack, acceleration_from_lean

PAD_TOP = 0.12          # m, height of the pad's surface above the ground in both worlds
RECORD_PERIOD = 0.05    # s between rows


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RelativeCheck(Node):
    def __init__(self):
        super().__init__('relative_check')
        self.output = Path(str(self.declare_parameter('output', 'results/relative_check').value))
        # Where the pad stands when its joints are at zero. A pad that never reports
        # joints (the static one) is taken to stand there throughout.
        self.pad_origin = (float(self.declare_parameter('pad_x', 4.0).value),
                           float(self.declare_parameter('pad_y', 3.0).value))
        self.camera = (CAMERA_FORWARD, CAMERA_LEFT, CAMERA_DOWN)
        self.track = RelativeTrack()
        self.attitudes = AttitudeBuffer()
        self.attitude = None
        self.accel = (0.0, 0.0)      # the drone's own horizontal acceleration, from its lean
        self.pad_joints = (0.0, 0.0)
        self.drone = None            # true (east, north, up, time)
        self.pose_sub = None
        self.detections = 0
        self.unmatched = 0
        self.rows = 0
        self.last_row = None

        self.output.mkdir(parents=True, exist_ok=True)
        self.file = open(self.output / 'relative.csv', 'w', newline='')
        self.log = csv.writer(self.file)
        self.log.writerow(['time', 'east', 'north', 'velocity_east', 'velocity_north', 'height', 'age',
                           'true_east', 'true_north', 'true_height', 'drone_east', 'drone_north',
                           'pad_east', 'pad_north', 'accel_east', 'accel_north'])
        self.create_subscription(PoseStamped, 'landing_pad/pose', self.on_pad, 10)
        self.create_subscription(JointState, 'moving_pad/joint_states', self.on_pad_joints, 10)
        self.create_subscription(Odometry, '/odometry', self.on_truth, qos_profile_sensor_data)
        self.create_timer(0.5, self.find_autopilot)
        self.create_timer(0.02, self.step)
        self.create_timer(5.0, self.report)
        self.get_logger().info('Recording. Fly over the pad now, and press Ctrl+C here when the flight is over.')

    def find_autopilot(self):
        if self.pose_sub is None:
            prefix = find_ap_prefix(self)
            if prefix:
                self.pose_sub = self.create_subscription(PoseStamped, f'{prefix}/pose/filtered',
                                                         self.on_pose, qos_profile_sensor_data)
                self.get_logger().info(f'Reading the attitude from {prefix}/pose/filtered')

    def on_pose(self, msg):
        o = msg.pose.orientation
        self.attitude = (o.x, o.y, o.z, o.w)             # the position in this message is not used
        self.attitudes.add(seconds(msg.header.stamp), self.attitude)

    def on_pad_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if 'slide_east' in positions and 'slide_north' in positions:
            self.pad_joints = (positions['slide_east'], positions['slide_north'])

    def on_pad(self, msg):
        stamp = seconds(msg.header.stamp)
        q = self.attitudes.at(stamp)
        if q is None:
            self.unmatched += 1
            return
        p = msg.pose.position
        east, north, up = pad_in_world(p.x, p.y, p.z, (0.0, 0.0, 0.0), q, *self.camera)
        self.track.update(stamp, east, north, -up)
        self.detections += 1

    def step(self):
        if self.attitude is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.accel = acceleration_from_lean(self.attitude)
        self.track.predict(now, self.accel)

    def on_truth(self, msg):
        t = seconds(msg.header.stamp)
        if self.track.time is None or (self.last_row is not None and t - self.last_row < RECORD_PERIOD):
            return
        self.last_row = t
        p = msg.pose.pose.position
        pad = (self.pad_origin[0] + self.pad_joints[0], self.pad_origin[1] + self.pad_joints[1])
        track = self.track
        accel = self.accel
        self.log.writerow([f'{t:.3f}'] + [f'{v:.4f}' for v in (
            track.position[0], track.position[1], track.velocity[0], track.velocity[1],
            track.height if track.height is not None else float('nan'), min(track.age(t), 99.0),
            pad[0] - p.x, pad[1] - p.y, p.z - PAD_TOP, p.x, p.y, pad[0], pad[1], accel[0], accel[1])])
        self.rows += 1

    def report(self):
        self.get_logger().info(f'{self.rows} rows, {self.detections} detections used, '
                               f'{self.unmatched} without a matching attitude')

    def close(self):
        if not self.file.closed:
            self.file.close()
            print(f'Recorded {self.rows} rows in {self.output / "relative.csv"}')


def main():
    rclpy.init()
    node = RelativeCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
