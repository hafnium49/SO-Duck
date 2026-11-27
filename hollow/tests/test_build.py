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
    assert (*model.STOCK_WRIST_JOINT, "crossing") in model.tested_pairs(robot)


def test_a_servo_and_the_link_on_its_horn_are_tested_outside_the_seat(robot):
    pairs = model.tested_pairs(robot)
    for servo, link in model.SEATS:
        assert (servo + model.SEAT_MARK, link + model.SEAT_MARK, "crossing") in pairs
    yaw = ("shoulder_yaw_case" + model.SEAT_MARK, "shoulder_pitch_link" + model.SEAT_MARK, "crossing")
    angles = np.array([-107.0, -100.0, 0.0, 100.0, 107.0])
    sweep = np.zeros((len(angles), 7))
    sweep[:, 0] = angles
    hit = pair_hits(robot, yaw, robot.placements(sweep)[0])
    assert hit.tolist() == [True, False, False, False, True]  # the stage meets the servo's plug strip near the range ends
    whole = pair_hits(robot, ("shoulder_yaw_case", "shoulder_pitch_link", "crossing"), robot.placements(sweep)[0])
    assert whole.all()  # with the seat left in, the two always touch


def test_the_stock_wrist_joint_crosses_in_one_narrow_band(robot):
    low, high = params.RANGES_DEG["wrist_roll"]
    angles = np.arange(-180.0, 180.25, 0.5)
    sweep = np.zeros((len(angles), 7))
    sweep[:, 5] = angles
    hit = pair_hits(robot, (*model.STOCK_WRIST_JOINT, "crossing"), robot.placements(sweep)[0])
    inside = angles[hit & (angles >= low) & (angles <= high)]
    assert len(inside) and inside.min() >= -118.0 and inside.max() <= -114.5, inside
    assert hit[angles <= -165].all() and hit[angles >= 172].all()  # the joint's own stops, outside the range


def test_model_ranges_come_from_sweeps(robot):
    for joint, (low, high) in params.RANGES_DEG.items():
        if joint != "wrist_roll":  # the stock joint keeps its stock range; see the test of its band
            measured = checks.joint_range(robot, joint)
            assert measured[0] <= low < 0 < high <= measured[1], (joint, measured)
        assert np.allclose(np.rad2deg(robot.model.joint(joint).range), (low, high), atol=1e-6)


def test_the_trunk_weighs_one_servo_less(root, robot):
    import mujoco

    stock_trunk = ET.parse(root / "coverage" / "robot.xml").find(".//body[@name='trunk_assembly']/inertial")
    before, centre = float(stock_trunk.get("mass")), np.fromstring(stock_trunk.get("pos"), sep=" ")
    trunk = robot.model.body("trunk_assembly")
    assert abs(trunk.mass[0] - (before - params.SERVO_KG)) < 1e-6
    # putting the servo back at its place gives the stock trunk's centre of mass
    restored = (trunk.mass[0] * trunk.ipos + params.SERVO_KG * np.array(assemble.NECK_SERVO_CENTRE)) / before
    assert np.allclose(restored, centre, atol=1e-6)
    # and that place is the middle of the servo's case meshes in the six-part model
    six = mujoco.MjModel.from_xml_path(str(root / "coverage" / "robot.xml"))
    points = []
    for geom in range(six.ngeom):
        if six.geom_bodyid[geom] != six.body("trunk_assembly").id or six.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = six.geom_dataid[geom]
        if six.mesh(mesh).name in assemble.NECK_SERVO and six.geom_pos[geom][0] > -0.005:
            turn = np.zeros(9)
            mujoco.mju_quat2Mat(turn, six.geom_quat[geom])
            vertices = six.mesh_vert[six.mesh_vertadr[mesh] : six.mesh_vertadr[mesh] + six.mesh_vertnum[mesh]]
            points.append(vertices @ turn.reshape(3, 3).T + six.geom_pos[geom])
    points = np.concatenate(points)
    assert np.abs((points.min(0) + points.max(0)) / 2 - assemble.NECK_SERVO_CENTRE).max() < 0.0005


def test_the_neck_servo_is_not_in_the_model(root):
    trunk = ET.parse(root / "hollow" / "robot.xml").find(".//body[@name='trunk_assembly']")
    kept = [geom for geom in trunk.findall("geom") if geom.get("mesh") in assemble.NECK_SERVO]
    assert len(kept) == 10  # the two hip-yaw servos use the same five meshes
    assert all(float(geom.get("pos").split()[0]) < -0.005 for geom in kept)
