"""Generate the ArUco texture used by the aruco_pad and moving_pad Gazebo models.

    python3 tools/make_marker.py

Writes models/<model>/materials/textures/aruco_pad.png: two markers from
DICT_4X4_50 on a white pad, one inside the other.

The large marker (1.0 m on the 1.25 m pad) is the one seen from flying height.
Below about 0.6 m it no longer fits in the camera image, so a small marker
(0.16 m) sits in its centre and stays in view almost down to touchdown.

The large marker's four middle cells form a 0.33 m square. The ID is chosen so
that these four are white, which leaves the small marker a white margin of its
own, and the large one still reads correctly because each middle cell stays
mostly white.
"""
from pathlib import Path

import cv2
import numpy as np

MARKER_ID = 16            # one of the two IDs in DICT_4X4_50 whose middle four cells are white (16 and 45)
SMALL_MARKER_ID = 3
PAD_SIZE = 1.25           # m, the textured top of the pad in model.sdf
MARKER_SIZE = 1.0         # m
SMALL_MARKER_SIZE = 0.16  # m
PX_PER_M = 1200           # gives whole pixels per cell: 200 for the large marker, 32 for the small one

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# The function was renamed in OpenCV 4.7.
draw = cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker

middle = draw(dictionary, MARKER_ID, 6)[2:4, 2:4]      # at one pixel per cell, border included
assert (middle == 255).all(), f'marker {MARKER_ID} has no white middle to hold the small marker'

image_px = round(PAD_SIZE * PX_PER_M)
image = np.full((image_px, image_px), 255, dtype=np.uint8)
for marker_id, size in ((MARKER_ID, MARKER_SIZE), (SMALL_MARKER_ID, SMALL_MARKER_SIZE)):
    marker_px = round(size * PX_PER_M)
    start = (image_px - marker_px) // 2                # both markers share the pad's centre
    image[start:start + marker_px, start:start + marker_px] = draw(dictionary, marker_id, marker_px)
# The moving pad carries the same markers and keeps its own copy of the texture.
for model in ('aruco_pad', 'moving_pad'):
    out = Path(__file__).resolve().parents[1] / f'models/{model}/materials/textures/aruco_pad.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))   # 3-channel, for the renderer
    print(f'wrote {out}')
