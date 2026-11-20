"""Find the landing pad's ArUco markers in a camera image and estimate where the pad is.

Pure OpenCV and NumPy, no ROS, so it can be tested on its own.

The pad carries two markers with the same centre: a large one for flying
height and a small one inside it, which still fits in the image when the
camera is too close to see the large one whole.

The result is the position of that shared centre in the camera's optical
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
    marker_id: int = 0        # which of the pad's markers this came from
    marker_size: float = 1.0  # its side in metres

    @property
    def distance(self):
        return float(np.linalg.norm(self.position))


class PadDetector:
    def __init__(self, marker_size=1.0, marker_id=16, small_marker_size=0.16, small_marker_id=3,
                 dictionary_id=cv2.aruco.DICT_4X4_50):
        """Pass small_marker_id=None for a pad with one marker only."""
        self.marker_id = marker_id
        # Side length of each marker we accept, in order of preference. The large marker
        # comes first: when both are readable the small one is only a few pixels wide.
        self.sizes = {marker_id: float(marker_size)}
        if small_marker_id is not None:
            self.sizes[small_marker_id] = float(small_marker_size)
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

    @staticmethod
    def object_points(marker_size):
        """Marker corners in the marker's own frame, in the order ArUco reports them.

        Top-left, top-right, bottom-right, bottom-left.
        """
        h = marker_size / 2.0
        return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)

    def read_markers(self, gray):
        """Return {marker id: pixel corners} for every marker of ours that OpenCV reports."""
        corners, ids, _ = self._find(gray)
        if ids is None:
            return {}
        return {int(marker_id): np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
                for marker_corners, marker_id in zip(corners, ids.ravel()) if int(marker_id) in self.sizes}

    def find_marker(self, gray):
        """Return (marker id, pixel corners) of the best of our markers in view, or None."""
        found = self.read_markers(gray)
        if self.marker_id not in found and found:
            # Only the small marker was reported. Older OpenCV reports both markers of a
            # nested pair, newer OpenCV only the inner one once it can be read, and from
            # a few metres up that one is about ten pixels wide. If the large marker
            # should still fit in the image, paint over the small one and look again.
            small_id, corners = next(iter(found.items()))
            side = np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1).mean()
            if side * self.sizes[self.marker_id] / self.sizes[small_id] < 0.9 * min(gray.shape[:2]):
                centre = corners.mean(axis=0)
                cover = np.round(centre + 1.4 * (corners - centre)).astype(np.int32)
                x, y, w, h = cv2.boundingRect(cover)
                white = int(gray[max(y, 0):y + h, max(x, 0):x + w].max())
                found.update(self.read_markers(cv2.fillConvexPoly(gray.copy(), cover, white)))
        for marker_id in self.sizes:
            if marker_id in found:
                return marker_id, found[marker_id]
        return None

    def find_corners(self, gray):
        """Return the pixel corners of the marker in use, or None."""
        found = self.find_marker(gray)
        return None if found is None else found[1]

    def detect(self, gray, camera_matrix, dist_coeffs=None):
        """Return a Detection of the pad, or None if neither marker is readable."""
        found = self.find_marker(gray)
        if found is None:
            return None
        marker_id, corners = found
        size = self.sizes[marker_id]
        if dist_coeffs is None:
            dist_coeffs = np.zeros(5)
        camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
        # IPPE_SQUARE is the solver meant for a single square marker. It can return
        # NaN when the marker faces the camera exactly, so fall back to the iterative solver.
        for solver in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):
            ok, rvec, tvec = cv2.solvePnP(self.object_points(size), corners, camera_matrix,
                                          dist_coeffs, flags=solver)
            if ok and np.all(np.isfinite(tvec)) and np.all(np.isfinite(rvec)):
                break
        else:
            return None
        return Detection(np.asarray(tvec).reshape(3), np.asarray(rvec).reshape(3), corners, marker_id, size)

    def is_mirrored(self, gray):
        """True if the marker is only found in the left-right flipped image.

        A mirrored marker means the texture is applied back to front in the simulator.
        """
        return self.find_corners(gray) is None and self.find_corners(cv2.flip(gray, 1)) is not None
