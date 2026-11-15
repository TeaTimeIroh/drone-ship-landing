"""Detect the landing pad in the drone's camera image and publish where it is.

    ros2 run ship_landing marker_detector --ros-args -p use_sim_time:=true

Publishes:
    landing_pad/pose         geometry_msgs/PoseStamped  marker centre in the camera optical frame
                                                        (x right in the image, y down, z out of the lens)
    landing_pad/debug_image  sensor_msgs/Image          camera image with the detection drawn on it
"""
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ship_landing.pad_detection import PadDetector

# Bytes per pixel and conversion to grey / BGR for the encodings Gazebo can produce.
ENCODINGS = {
    'rgb8': (3, cv2.COLOR_RGB2GRAY, cv2.COLOR_RGB2BGR),
    'bgr8': (3, cv2.COLOR_BGR2GRAY, None),
    'rgba8': (4, cv2.COLOR_RGBA2GRAY, cv2.COLOR_RGBA2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2GRAY, cv2.COLOR_BGRA2BGR),
    'mono8': (1, None, cv2.COLOR_GRAY2BGR),
}
LOG_EVERY_FRAMES = 20


def decode_image(msg):
    """Convert a sensor_msgs/Image to (grey, bgr) NumPy arrays without cv_bridge."""
    if msg.encoding not in ENCODINGS:
        raise ValueError(f'unsupported image encoding {msg.encoding!r}')
    channels, to_gray, to_bgr = ENCODINGS[msg.encoding]
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        pixels = pixels[:, :, 0]
    gray = pixels if to_gray is None else cv2.cvtColor(pixels, to_gray)
    bgr = pixels.copy() if to_bgr is None else cv2.cvtColor(pixels, to_bgr)
    return gray, bgr


def rvec_to_quaternion(rvec):
    """Rodrigues rotation vector to quaternion (x, y, z, w)."""
    angle = float(np.linalg.norm(rvec))
    if angle < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    axis = np.asarray(rvec, dtype=float) / angle
    s = np.sin(angle / 2.0)
    return float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), float(np.cos(angle / 2.0))


class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        image_topic = self.declare_parameter('image_topic', '/camera/image').value
        info_topic = self.declare_parameter('camera_info_topic', '/camera/camera_info').value
        marker_size = self.declare_parameter('marker_size', 1.0).value
        marker_id = self.declare_parameter('marker_id', 0).value
        self.publish_debug = self.declare_parameter('publish_debug_image', True).value

        self.detector = PadDetector(marker_size=float(marker_size), marker_id=int(marker_id))
        self.camera_matrix = None
        self.dist_coeffs = None
        self.frames = 0
        self.hits = 0
        self.last = None
        self.warned_mirrored = False

        self.pose_pub = self.create_publisher(PoseStamped, 'landing_pad/pose', 10)
        self.debug_pub = self.create_publisher(Image, 'landing_pad/debug_image', 1)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f'Looking for marker {marker_id} ({marker_size} m) in {image_topic}, OpenCV {cv2.__version__}')

    def on_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
            self.get_logger().info(
                f'Camera calibration received: {msg.width}x{msg.height}, '
                f'focal length {self.camera_matrix[0, 0]:.1f} px')

    def on_image(self, msg):
        if self.camera_matrix is None:
            return                      # cannot turn pixels into metres yet
        try:
            gray, bgr = decode_image(msg)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return

        detection = self.detector.detect(gray, self.camera_matrix, self.dist_coeffs)
        self.frames += 1
        if detection is not None:
            self.hits += 1
            self.last = detection
            self.publish_pose(msg, detection)
        elif not self.warned_mirrored and self.detector.is_mirrored(gray):
            self.warned_mirrored = True
            self.get_logger().error(
                'The marker is only readable when the image is flipped: its texture is '
                'mirrored in the simulator and needs flipping in tools/make_marker.py')

        if self.publish_debug:
            self.publish_debug_image(msg, bgr, detection)
        if self.frames % LOG_EVERY_FRAMES == 0:
            self.report()

    def publish_pose(self, image_msg, detection):
        pose = PoseStamped()
        pose.header.stamp = image_msg.header.stamp
        pose.header.frame_id = 'camera_optical'
        x, y, z = (float(v) for v in detection.position)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        qx, qy, qz, qw = rvec_to_quaternion(detection.rvec)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        self.pose_pub.publish(pose)

    def publish_debug_image(self, image_msg, bgr, detection):
        if detection is not None:
            outline = detection.corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(bgr, [outline], True, (0, 255, 0), 2)
            centre = tuple(int(v) for v in detection.corners.mean(axis=0))
            cv2.drawMarker(bgr, centre, (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
            x, y, z = detection.position
            cv2.putText(bgr, f'x {x:+.2f}  y {y:+.2f}  z {z:.2f} m', (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(bgr, 'no marker', (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        out = Image()
        out.header = image_msg.header
        out.height, out.width = bgr.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = bgr.tobytes()
        self.debug_pub.publish(out)

    def report(self):
        rate = 100.0 * self.hits / self.frames
        if self.last is None:
            self.get_logger().info(f'{self.frames} frames, marker not seen yet')
            return
        x, y, z = self.last.position
        self.get_logger().info(
            f'seen in {rate:.0f}% of {self.frames} frames | last: right {x:+.2f} m, '
            f'down-image {y:+.2f} m, range along lens {z:.2f} m, sideways offset {np.hypot(x, y):.2f} m')


def main():
    rclpy.init()
    node = MarkerDetector()
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
