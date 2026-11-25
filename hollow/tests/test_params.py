import hashlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import params

VOLUME_CM3 = {
    "Under_arm_SO101": 93.0,
    "Upper_arm_SO101": 117.4,
    "Rotation_Pitch_SO101": 68.8,
    "Wrist_Roll_Pitch_SO101": 32.3,
    "Motor_holder_SO101_Wrist": 13.9,
    "Motor_holder_SO101_Base": 13.3,
    "Base_motor_holder_SO101": 23.7,
}
# Joint name and the model body whose pose in its parent the joint frame is.
BODY = {
    "shoulder_yaw": "shoulder_yaw_linkage",
    "shoulder_pitch": "shoulder_pitch_linkage",
    "forearm_pitch": "forearm_pitch_linkage",
    "forearm_roll": "forearm_roll_linkage",
    "wrist_flex": "wrist_flex_linkage",
    "wrist_roll": "gripper",
    "gripper": "moving_jaw_so101_v1",
}


def test_donor_checksums(root):
    folder = root / "hollow/cad/donors"
    lines = (folder / "SHA256SUMS").read_text().split("\n")
    listed = dict(reversed(line.split()) for line in lines if line)
    assert sorted(listed) == sorted(name + ".step" for name in VOLUME_CM3)
    for name, digest in listed.items():
        assert hashlib.sha256((folder / name).read_bytes()).hexdigest() == digest, name


@pytest.mark.parametrize("name", VOLUME_CM3)
def test_donor_is_one_valid_solid(name):
    solid = params.donor(name)
    assert solid.isValid() and len(solid.Solids()) == 1
    assert abs(solid.Volume() / 1000 - VOLUME_CM3[name]) < 0.1


def test_frames_match_model(root):
    model = ET.parse(root / "hollow/robot.xml")
    frames = params.joint_frames(0, 0, 0)
    assert list(frames) == list(BODY)
    for joint, body in BODY.items():
        element = model.find(f".//body[@name='{body}']")
        position = np.fromstring(element.get("pos"), sep=" ")
        quaternion = np.fromstring(element.get("quat"), sep=" ")
        assert np.allclose(frames[joint][0], position, atol=1e-9), joint
        assert np.allclose(frames[joint][1], quaternion, atol=1e-9), joint


def test_shortening_moves_two_joints():
    base, short = params.joint_frames(0, 0, 0), params.joint_frames(0.040, 0.040, 0.012)
    assert np.allclose(short["forearm_roll"][0] - base["forearm_roll"][0], [-0.040, 0, 0])
    assert np.allclose(short["wrist_flex"][0] - base["wrist_flex"][0], [0.040, 0, -0.012])
    assert all(np.allclose(short[j][0], base[j][0]) for j in base if j not in ("forearm_roll", "wrist_flex"))
    assert all(np.allclose(short[j][1], base[j][1]) for j in base)


def test_inventory_skips_scratch_directories(root):
    sys_path_script = (root / "scripts/study_inventory.py").read_text()
    assert ".pytest_cache" in sys_path_script and ".superpowers" in sys_path_script
