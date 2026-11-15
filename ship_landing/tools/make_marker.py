"""Generate the ArUco texture used by the aruco_pad Gazebo model.

    python3 tools/make_marker.py

Writes models/aruco_pad/materials/textures/aruco_pad.png: marker ID 0 from
DICT_4X4_50 with a white quiet zone, so that the black marker covers
MARKER_FRACTION of the image side. With the 1.25 m pad in model.sdf the
marker itself is 1.0 m across.
"""
from pathlib import Path

import cv2
import numpy as np

MARKER_ID = 0
IMAGE_PX = 1000
MARKER_FRACTION = 0.8

out = Path(__file__).resolve().parents[1] / 'models/aruco_pad/materials/textures/aruco_pad.png'
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_px = int(IMAGE_PX * MARKER_FRACTION)
if hasattr(cv2.aruco, 'generateImageMarker'):      # OpenCV 4.7 and newer
    marker = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, marker_px)
else:                                              # OpenCV 4.6 and older
    marker = cv2.aruco.drawMarker(dictionary, MARKER_ID, marker_px)

image = np.full((IMAGE_PX, IMAGE_PX), 255, dtype=np.uint8)
pad = (IMAGE_PX - marker_px) // 2
image[pad:pad + marker_px, pad:pad + marker_px] = marker
out.parent.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(out), cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))   # 3-channel, for the renderer
print(f'wrote {out}')
