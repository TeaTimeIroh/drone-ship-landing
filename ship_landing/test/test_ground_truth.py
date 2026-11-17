"""Check that Gazebo's pose printout is read correctly."""
from ship_landing.ground_truth import parse_gz_pose

# Exactly what `gz model -m iris -p` printed during development, warnings included.
SAMPLE = '''
Requesting state for world [landing_pad]...

Warning [Utils.cc:132] [/sdf/model[@name="iris_with_gimbal"]/link[@name="imu_link"]/sensor[@name="imu_sensor"]/gz_frame_id:<data-string>:L282]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
Model: [16]
  - Name: iris
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [2.977610 1.973500 0.215000]
    [0.000001 0.000000 1.566050]
'''


def test_reads_position_and_orientation():
    pose = parse_gz_pose(SAMPLE)
    assert (pose.x, pose.y, pose.z) == (2.977610, 1.973500, 0.215000)
    assert pose.yaw == 1.566050


def test_reads_negative_and_scientific_numbers():
    pose = parse_gz_pose('  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [-5.647480 1.5e+00 0.195]\n    [-1e-06 1e-06 1.569380]\n')
    assert (pose.x, pose.y, pose.z) == (-5.647480, 1.5, 0.195)
    assert pose.roll == -1e-06


def test_returns_none_when_there_is_no_pose():
    assert parse_gz_pose('Requesting state for world [landing_pad]...\nModel not found\n') is None
    assert parse_gz_pose('') is None
