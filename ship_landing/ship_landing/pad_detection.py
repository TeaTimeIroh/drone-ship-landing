"""Find the landing pad's ArUco marker in a camera image and estimate where it is.

Pure OpenCV and NumPy, no ROS, so it can be tested on its own.

The result is the position of the marker centre in the camera's optical
frame: x to the right in the image, y down in the image, z out of the lens.
All distances are in metres.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    position: np.ndarray      # marker centre in the camera optical frame, metres, shape (3,)
    rvec: np.ndarray          # marker orientation as a Rodrigues vector, shape (3,)
    corners: np.ndarray       # the four marker corners in pixels, shape (4, 2)

    @property
    def distance(self):
        return float(np.linalg.norm(self.position))


class PadDetector:
    def __init__(self, marker_size=1.0, marker_id=0, dictionary_id=cv2.aruco.DICT_4X4_50):
        self.marker_id = marker_id
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, 'ArucoDetector'):                 # OpenCV 4.7 and newer
            parameters = cv2.aruco.DetectorParameters()
        else:                                                   # OpenCV 4.6 and older
            parameters = cv2.aruco.DetectorParameters_create()
        # Sub-pixel corners matter here: the marker is only a few dozen pixels wide
        # from flying height, and without refinement the range reads several percent long.
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._find = cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers
        else:
            self._find = lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
        # Marker corners in the marker's own frame, in the order ArUco reports them:
        # top-left, top-right, bottom-right, bottom-left.
        h = marker_size / 2.0
        self._object_points = np.array(
            [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)

    def find_corners(self, gray):
        """Return the pixel corners of our marker, or None."""
        corners, ids, _ = self._find(gray)
        if ids is None:
            return None
        for marker_corners, marker_id in zip(corners, ids.ravel()):
            if int(marker_id) == self.marker_id:
                return np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
        return None

    def detect(self, gray, camera_matrix, dist_coeffs=None):
        """Return a Detection for our marker, or None if it is not in the image."""
        corners = self.find_corners(gray)
        if corners is None:
            return None
        if dist_coeffs is None:
            dist_coeffs = np.zeros(5)
        camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
        # IPPE_SQUARE is the solver meant for a single square marker. It can return
        # NaN when the marker faces the camera exactly, so fall back to the iterative solver.
        for solver in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):
            ok, rvec, tvec = cv2.solvePnP(self._object_points, corners, camera_matrix,
                                          dist_coeffs, flags=solver)
            if ok and np.all(np.isfinite(tvec)) and np.all(np.isfinite(rvec)):
                break
        else:
            return None
        return Detection(np.asarray(tvec).reshape(3), np.asarray(rvec).reshape(3), corners)

    def is_mirrored(self, gray):
        """True if the marker is only found in the left-right flipped image.

        A mirrored marker means the texture is applied back to front in the simulator.
        """
        return self.find_corners(gray) is None and self.find_corners(cv2.flip(gray, 1)) is not None
