import xml.etree.ElementTree as ET

import cadquery as cq
import numpy as np

import assemble
import build_parts
import checks
import params
from replay import pair_hits

import model


def test_committed_parts_are_current():
    assert build_parts.check()


def test_part_masses_are_solid_volume(robot):
    for geom, (_, body, _) in build_parts.CUSTOM.items():
        stored = cq.importers.importStep(str(build_parts.OUT / f"{assemble.CUSTOM[geom]}.step")).val()
        part = robot.model.body(body).mass[0] - params.SERVO_KG
        assert abs(part - stored.Volume() * 1e-9 * params.DENSITY) < 1e-6, geom


def test_zero_pose_has_no_intersections(robot):
    frames, _ = robot.placements(np.zeros((1, 7)))
    assert not any(pair_hits(robot, pair, frames).any() for pair in model.tested_pairs(robot))


def test_neighbours_across_every_joint_are_tested(robot):
    tested = {frozenset(pair[:2]) for pair in model.tested_pairs(robot)}
    assert frozenset(("forearm_roll_link", "forearm_pitch_link")) in tested
    assert frozenset(("wrist_flex_link", "forearm_roll_link")) in tested
    assert frozenset(("shoulder_mount", "shoulder_pitch_link")) in tested
    assert frozenset(("shoulder_mount", "trunk_assembly")) in tested
    assert not any(frozenset(seat) in tested for seat in model.SEATS)


def test_model_ranges_come_from_sweeps(robot):
    for joint, (low, high) in params.RANGES_DEG.items():
        measured = checks.joint_range(robot, joint)
        assert measured[0] <= low < 0 < high <= measured[1], (joint, measured)
        assert np.allclose(np.rad2deg(robot.model.joint(joint).range), (low, high), atol=1e-6)


def test_the_neck_servo_is_not_in_the_model(root):
    trunk = ET.parse(root / "hollow" / "robot.xml").find(".//body[@name='trunk_assembly']")
    kept = [geom for geom in trunk.findall("geom") if geom.get("mesh") in assemble.NECK_SERVO]
    assert len(kept) == 10  # the two hip-yaw servos use the same five meshes
    assert all(float(geom.get("pos").split()[0]) < -0.005 for geom in kept)
