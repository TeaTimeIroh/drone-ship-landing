"""Check PadDetector against synthetic camera views with a known marker position."""
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from ship_landing.pad_detection import PadDetector

TEXTURE = Path(__file__).resolve().parents[1] / 'models/aruco_pad/materials/textures/aruco_pad.png'
PAD_SIZE = 1.25          # metres, the whole textured pad
WIDTH, HEIGHT = 640, 480
FOCAL = (WIDTH / 2) / math.tan(1.0)     # horizontal field of view 2.0 rad, as in the simulator
K = np.array([[FOCAL, 0, WIDTH / 2], [0, FOCAL, HEIGHT / 2], [0, 0, 1.0]])


def render(position, yaw_deg=0.0, tilt_deg=0.0, noise=3.0, seed=0):
    """Draw the pad as a camera would see it with the pad centre at `position` (camera frame)."""
    texture = cv2.imread(str(TEXTURE), cv2.IMREAD_GRAYSCALE)
    size = texture.shape[0]
    yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
    rot_z = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    rot_x = np.array([[1, 0, 0], [0, math.cos(tilt), -math.sin(tilt)], [0, math.sin(tilt), math.cos(tilt)]])
    # Pad frame with x right and y down in the texture, so an untilted pad faces the camera.
    rotation = rot_x @ rot_z
    corners_pad = np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]], dtype=float) * PAD_SIZE / 2
    corners_cam = corners_pad @ rotation.T + np.asarray(position, dtype=float)
    pixels = corners_cam @ K.T
    pixels = (pixels[:, :2] / pixels[:, 2:]).astype(np.float32)
    source = np.array([[0, 0], [size, 0], [size, size], [0, size]], dtype=np.float32)
    image = np.full((HEIGHT, WIDTH), 110, np.uint8)
    cv2.warpPerspective(texture, cv2.getPerspectiveTransform(source, pixels), (WIDTH, HEIGHT),
                        dst=image, borderMode=cv2.BORDER_TRANSPARENT, flags=cv2.INTER_AREA)
    rng = np.random.default_rng(seed)
    image = cv2.GaussianBlur(image, (3, 3), 0).astype(float) + rng.normal(0, noise, image.shape)
    return np.clip(image, 0, 255).astype(np.uint8)


@pytest.mark.parametrize('position', [
    (0.0, 0.0, 5.0), (3.0, 2.0, 5.0), (-2.0, 1.5, 3.0), (0.3, -0.2, 1.5), (0.0, 0.0, 1.0),
])
def test_position_is_recovered_at_landing_range(position):
    """Within 5 m the estimate should be good to a few centimetres."""
    detection = PadDetector(marker_size=1.0).detect(render(position, yaw_deg=25), K)
    assert detection is not None
    error = np.linalg.norm(detection.position - np.array(position))
    assert error < 0.06, f'{error:.3f} m error at {position}'


@pytest.mark.parametrize('position', [(0.0, 0.0, 8.0), (4.0, -3.0, 10.0), (1.0, 1.0, 12.0)])
def test_direction_is_recovered_from_far_away(position):
    """From far away the marker is ~20 px wide: range is biased, but the direction holds."""
    detection = PadDetector(marker_size=1.0).detect(render(position, yaw_deg=25), K)
    assert detection is not None
    truth = np.array(position)
    range_error = abs(detection.distance - np.linalg.norm(truth)) / np.linalg.norm(truth)
    cosine = detection.position @ truth / (detection.distance * np.linalg.norm(truth))
    assert range_error < 0.10, f'range off by {range_error:.1%} at {position}'
    assert math.degrees(math.acos(min(1.0, cosine))) < 1.0


def test_tilted_marker_is_recovered():
    detection = PadDetector(marker_size=1.0).detect(render((0.5, 0.5, 4.0), tilt_deg=15), K)
    assert detection is not None
    assert np.linalg.norm(detection.position - np.array([0.5, 0.5, 4.0])) < 0.2


def test_no_marker_gives_none():
    blank = np.full((HEIGHT, WIDTH), 110, np.uint8)
    assert PadDetector().detect(blank, K) is None


def test_other_marker_id_is_ignored():
    assert PadDetector(marker_id=7).detect(render((0.0, 0.0, 5.0)), K) is None


def test_mirrored_marker_is_flagged():
    detector = PadDetector()
    mirrored = cv2.flip(render((0.0, 0.0, 5.0)), 1)
    assert detector.detect(mirrored, K) is None
    assert detector.is_mirrored(mirrored)


@pytest.mark.parametrize('height', [0.5, 0.4, 0.3, 0.2])
def test_small_marker_takes_over_close_up(height):
    """Too close for the large marker to fit in the image, the small one gives the pad centre."""
    detector = PadDetector()
    position = (0.03, -0.02, height)
    detection = detector.detect(render(position, yaw_deg=25), K)
    assert detection is not None
    assert detection.marker_id != detector.marker_id and detection.marker_size == 0.16
    assert np.linalg.norm(detection.position - np.array(position)) < 0.01


def test_large_marker_alone_is_lost_close_up():
    """Without the small marker the pad is not seen from 0.4 m: the gap the small marker closes."""
    assert PadDetector(small_marker_id=None).detect(render((0.0, 0.0, 0.4), yaw_deg=25), K) is None


def test_pad_is_seen_all_the_way_down():
    """No height between 4 m and 0.2 m at which neither marker can be read, or read badly."""
    detector = PadDetector()
    for height in np.arange(4.0, 0.19, -0.1):
        position = np.array([0.1 * height, -0.05 * height, height])
        detection = detector.detect(render(position, yaw_deg=25, tilt_deg=8), K)
        assert detection is not None, f'pad not seen from {height:.1f} m'
        error = np.linalg.norm(detection.position - position)
        assert error < 0.01 + 0.02 * height, f'{error:.3f} m error from {height:.1f} m'
