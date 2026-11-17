"""Ask Gazebo where a model really is.

This is the simulator's own answer, independent of the autopilot's estimate and of the
camera, so it is what landing accuracy gets measured against. It uses the `gz` command
line tool, which takes about a second; do not call it from a tight control loop.
"""
import re
import subprocess
from typing import NamedTuple, Optional


class Pose(NamedTuple):
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


_TRIPLE = re.compile(r'\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]')


def parse_gz_pose(text: str) -> Optional[Pose]:
    """Read the pose out of the text printed by `gz model -m <name> -p`.

    The output has a 'Pose [ XYZ (m) ] [ RPY (rad) ]:' line followed by two bracketed
    triples, position then orientation. Warnings may come before it.
    """
    lines = text.splitlines()
    for index, line in enumerate(lines):
        if 'Pose' in line and 'XYZ' in line:
            triples = []
            for following in lines[index + 1:index + 6]:
                match = _TRIPLE.search(following)
                if match:
                    triples.append([float(v) for v in match.groups()])
                if len(triples) == 2:
                    return Pose(*triples[0], *triples[1])
    return None


def gz_model_pose(model: str = 'iris', timeout: float = 15.0) -> Optional[Pose]:
    """Return the model's pose in the Gazebo world frame, or None if it cannot be read."""
    try:
        result = subprocess.run(['gz', 'model', '-m', model, '-p'],
                                capture_output=True, text=True, timeout=timeout)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    return parse_gz_pose(result.stdout + '\n' + result.stderr)
