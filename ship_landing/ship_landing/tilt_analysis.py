"""Check the tilt correction against a recorded flight. No ROS in here.

Input is two logs from tilt_check: every marker detection (camera frame, with the
image's timestamp) and every pose the autopilot published (position and attitude).
The pad is static and its position is known, so for each detection the true offset
from the drone to the pad follows from the autopilot's position. The detection is
then converted three ways and compared with that:

    raw        as land_on_pad uses it today, ignoring tilt
    levelled   rotated by the autopilot's attitude, as published
    variants   the same with roll, pitch or both reversed

If the attitude convention is understood correctly, "levelled" has the smallest error.
A reversed sign shows up as a variant winning, and as an error larger than raw.

The comparison trusts the autopilot's position estimate, which uses GPS here. Its own
error, typically several centimetres, is a floor under every number in the result.
"""
import bisect
import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

from ship_landing.levelling import blend, heading, levelled_offset, normalise, tilt

VARIANTS = ('as published', 'roll reversed', 'pitch reversed', 'both reversed')
SHIFTS = [round(-0.30 + 0.02 * i, 2) for i in range(31)]      # s, clock offsets to try
TILTED = math.radians(4.0)       # more tilt than this counts as "tilted"
LEVEL = math.radians(1.5)        # less than this counts as "level"


@dataclass
class Detection:
    stamp: float
    x: float
    y: float
    z: float


@dataclass
class Pose:
    stamp: float
    position: tuple
    quat: tuple


@dataclass
class Geometry:
    pad_east: float = 3.0
    pad_north: float = 2.0
    camera_forward: float = 0.020
    camera_left: float = -0.010
    camera_down: float = 0.10


@dataclass
class Result:
    samples: int = 0
    tilted_samples: int = 0
    level_samples: int = 0
    max_tilt_deg: float = 0.0
    rms_raw: float = float('nan')
    rms_raw_tilted: float = float('nan')
    rms_raw_level: float = float('nan')
    rms: dict = field(default_factory=dict)            # variant -> RMS error, all samples
    rms_tilted: dict = field(default_factory=dict)     # variant -> RMS error, tilted samples
    rms_level: dict = field(default_factory=dict)
    best_variant: str = ''
    best_shift: float = 0.0
    rms_at_best_shift: float = float('nan')
    heading_autopilot_deg: float = float('nan')
    heading_camera_deg: float = float('nan')
    series: list = field(default_factory=list)         # (time, tilt deg, raw error, levelled error)


def load_log(folder):
    """Read detections.csv and poses.csv from a tilt_check output folder.

    Returns (detections, poses, note). Timestamps are the ones in the messages. If the
    two streams turn out to be stamped by unrelated clocks, the times at which the
    recording node received them are used instead, and the note says so.
    """
    folder = Path(folder)
    detections = load_detections(folder / 'detections.csv')
    poses = load_poses(folder / 'poses.csv')
    if not detections or not poses:
        return detections, poses, ''
    first, last = detections[0].stamp, detections[-1].stamp
    inside = sum(1 for p in poses if first <= p.stamp <= last)
    if inside >= 0.5 * len(poses) * min(1.0, (last - first) / max(1e-9, poses[-1].stamp - poses[0].stamp)) \
            and inside > 10:
        return detections, poses, ''
    detections = load_detections(folder / 'detections.csv', 'received')
    poses = load_poses(folder / 'poses.csv', 'received')
    return detections, poses, ('The image and pose timestamps come from unrelated clocks, so the times '
                               'at which the messages arrived were used instead. These include delivery delays.')


def load_detections(path, time_column='stamp'):
    with open(path, newline='') as f:
        return [Detection(float(r[time_column]), float(r['x']), float(r['y']), float(r['z']))
                for r in csv.DictReader(f)]


def load_poses(path, time_column='stamp'):
    with open(path, newline='') as f:
        return [Pose(float(r[time_column]),
                     (float(r['east']), float(r['north']), float(r['up'])),
                     normalise((float(r['qx']), float(r['qy']), float(r['qz']), float(r['qw']))))
                for r in csv.DictReader(f)]


def clean_poses(poses):
    """Sort by timestamp and drop repeats, which the autopilot produces between clock updates."""
    out = []
    for pose in sorted(poses, key=lambda p: p.stamp):
        if not out or pose.stamp > out[-1].stamp:
            out.append(pose)
    return out


def pose_at(poses, times, t):
    """Interpolated pose at time t, or None outside the log or across a gap."""
    i = bisect.bisect_left(times, t)
    if i == 0 or i == len(times):
        return None
    a, b = poses[i - 1], poses[i]
    if b.stamp - a.stamp > 0.5:
        return None
    f = (t - a.stamp) / (b.stamp - a.stamp)
    position = tuple(pa + f * (pb - pa) for pa, pb in zip(a.position, b.position))
    return Pose(t, position, blend(a.quat, b.quat, f))


def euler(q):
    """Roll, pitch, yaw of a body(x forward, y left, z up) to world(z up) rotation, ZYX order."""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def variant_quat(q, variant):
    roll, pitch, yaw = euler(q)
    if variant in ('roll reversed', 'both reversed'):
        roll = -roll
    if variant in ('pitch reversed', 'both reversed'):
        pitch = -pitch
    return from_euler(roll, pitch, yaw)


def true_offset(pose, geometry):
    """Where the pad really is relative to the drone's centre: (forward, left) in the level frame."""
    east = geometry.pad_east - pose.position[0]
    north = geometry.pad_north - pose.position[1]
    yaw = heading(pose.quat)
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * east + s * north, -s * east + c * north)


def raw_offset(d, geometry):
    return (-d.y + geometry.camera_forward, -d.x + geometry.camera_left)


def _rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float('nan')


def errors_for(detections, poses, geometry, variant, shift):
    """Per detection: (time, tilt, raw error, levelled error), for one variant and clock offset."""
    times = [p.stamp for p in poses]
    rows = []
    for d in detections:
        pose = pose_at(poses, times, d.stamp + shift)
        if pose is None:
            continue
        truth = true_offset(pose, geometry)
        raw = raw_offset(d, geometry)
        q = pose.quat if variant == 'as published' else variant_quat(pose.quat, variant)
        lev = levelled_offset(d.x, d.y, d.z, q, geometry.camera_forward,
                              geometry.camera_left, geometry.camera_down)
        rows.append((d.stamp, tilt(pose.quat),
                     math.hypot(raw[0] - truth[0], raw[1] - truth[1]),
                     math.hypot(lev.forward - truth[0], lev.left - truth[1])))
    return rows


def camera_heading(detections, poses, geometry):
    """The heading at which the camera's view of the pad best matches the world, in radians.

    Uses near-level samples only. Should equal the autopilot's heading; a difference of
    90 or 180 degrees would mean the camera's axes are not the ones assumed.
    """
    times = [p.stamp for p in poses]
    sin_sum = cos_sum = 0.0
    for d in detections:
        pose = pose_at(poses, times, d.stamp)
        if pose is None or tilt(pose.quat) > LEVEL:
            continue
        east = geometry.pad_east - pose.position[0]
        north = geometry.pad_north - pose.position[1]
        forward, left = raw_offset(d, geometry)
        # Find yaw with (east, north) = Rz(yaw) (forward, left).
        sin_sum += north * forward - east * left
        cos_sum += east * forward + north * left
    if sin_sum == 0.0 and cos_sum == 0.0:
        return float('nan')
    return math.atan2(sin_sum, cos_sum)


def analyse(detections, poses, geometry=None):
    geometry = geometry or Geometry()
    poses = clean_poses(poses)
    result = Result()
    base = errors_for(detections, poses, geometry, 'as published', 0.0)
    if not base:
        return result
    result.samples = len(base)
    tilted = [r for r in base if r[1] > TILTED]
    flat = [r for r in base if r[1] < LEVEL]
    result.tilted_samples, result.level_samples = len(tilted), len(flat)
    result.max_tilt_deg = math.degrees(max(r[1] for r in base))
    result.rms_raw = _rms([r[2] for r in base])
    result.rms_raw_tilted = _rms([r[2] for r in tilted])
    result.rms_raw_level = _rms([r[2] for r in flat])
    result.series = [(r[0], math.degrees(r[1]), r[2], r[3]) for r in base]

    for variant in VARIANTS:
        rows = base if variant == 'as published' else errors_for(detections, poses, geometry, variant, 0.0)
        result.rms[variant] = _rms([r[3] for r in rows])
        result.rms_tilted[variant] = _rms([r[3] for r in rows if r[1] > TILTED])
        result.rms_level[variant] = _rms([r[3] for r in rows if r[1] < LEVEL])
    key = result.rms_tilted if tilted else result.rms
    result.best_variant = min(VARIANTS, key=lambda v: key[v])

    best = (float('inf'), 0.0)
    for shift in SHIFTS:
        rows = errors_for(detections, poses, geometry, 'as published', shift)
        if len(rows) < 0.8 * len(base):
            continue
        best = min(best, (_rms([r[3] for r in rows]), shift))
    result.rms_at_best_shift, result.best_shift = best

    times = [p.stamp for p in poses]
    headings = [heading(p.quat) for p in (pose_at(poses, times, d.stamp) for d in detections) if p]
    if headings:
        result.heading_autopilot_deg = math.degrees(math.atan2(
            sum(math.sin(h) for h in headings), sum(math.cos(h) for h in headings)))
    result.heading_camera_deg = math.degrees(camera_heading(detections, poses, geometry))
    return result


def verdict(result):
    """One paragraph saying what the numbers mean."""
    if result.samples == 0:
        return 'No detection could be matched with a pose. Nothing to conclude.'
    if result.tilted_samples < 20:
        return (f'Only {result.tilted_samples} detections were taken while the drone was tilted by more '
                'than 4 degrees. That is too few to judge the correction; fly the check again.')
    lev = result.rms_tilted['as published']
    raw = result.rms_raw_tilted
    if result.best_variant != 'as published':
        return (f'The correction as written is NOT the best variant: "{result.best_variant}" has the smallest '
                f'error while tilted ({result.rms_tilted[result.best_variant]:.2f} m against {lev:.2f} m). '
                'The attitude convention is not what the code assumes and must be fixed before use.')
    if lev >= raw:
        return (f'The correction does not help: {lev:.2f} m while tilted against {raw:.2f} m without it. '
                'Do not use it until this is understood.')
    text = (f'The correction as written is the best variant. While tilted it cuts the error from '
            f'{raw:.2f} m to {lev:.2f} m.')
    if abs(result.best_shift) >= 0.04 and result.rms_at_best_shift < 0.9 * result.rms['as published']:
        text += (f' Reading the pose {abs(result.best_shift):.2f} s '
                 f'{"later" if result.best_shift > 0 else "earlier"} than the image timestamp would lower '
                 f'the overall error from {result.rms["as published"]:.2f} m to '
                 f'{result.rms_at_best_shift:.2f} m, so the two clocks are not perfectly aligned.')
    return text


def to_markdown(result, note=''):
    def m(value):
        return 'n/a' if value != value else f'{value:.3f} m'

    lines = ['# Tilt check', '']
    if result.samples == 0:
        return '\n'.join(lines + [verdict(result), ''])
    lines += [
        verdict(result), '',
        f'- Detections used: {result.samples}, of which {result.tilted_samples} with more than 4 degrees '
        f'of tilt and {result.level_samples} with less than 1.5 degrees. Largest tilt: {result.max_tilt_deg:.1f} degrees.',
        f'- Heading: {result.heading_autopilot_deg:.1f} degrees from east according to the autopilot, '
        f'{result.heading_camera_deg:.1f} degrees according to the camera (these should agree).',
        f'- Best time offset when looking up the pose for an image: {result.best_shift:+.2f} s '
        f'(error {m(result.rms_at_best_shift)} there, {m(result.rms["as published"])} at zero).',
        '',
        'Horizontal error against the position the autopilot reports (RMS):', '',
        '| Conversion | All detections | Tilted | Level |',
        '|---|---|---|---|',
        f'| raw, tilt ignored | {m(result.rms_raw)} | {m(result.rms_raw_tilted)} | {m(result.rms_raw_level)} |',
    ]
    for variant in VARIANTS:
        name = 'levelled, as published' if variant == 'as published' else f'levelled, {variant}'
        lines.append(f'| {name} | {m(result.rms[variant])} | {m(result.rms_tilted[variant])} '
                     f'| {m(result.rms_level[variant])} |')
    lines += ['', 'The autopilot\'s own position error is included in every number above.', '']
    if note:
        lines += [note, '']
    return '\n'.join(lines)

