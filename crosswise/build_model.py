"""Build the current 17-axis SO-Duck MJCF from the two archived donor models."""
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import cadquery as cq
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "search"))
import arm_model as arm
from motion import HIGH, NEUTRAL, PARK, PAD_CENTER_MM, PAD_HEIGHT_MM, PAD_RADIUS_MM, ROOF_MM, pickup

INPUTS = ROOT / "collision/inputs"
FRAMES = arm.frames(90, 0, 0, False)
PITCH_LIMITS = (-40., 96.)  # one degree inside first clear negative sample; within donor elbow travel
ARM_JOINTS = ("neck_pitch", "head_pitch", "head_yaw", "forearm_pitch", "wrist_flex", "wrist_roll")
COLORS = {"carrier": ".12 .55 .52 1", "case": ".21 .26 .29 1", "arm": ".83 .55 .2 1"}


def numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def placement(matrix):
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return dict(pos=numbers(matrix[:3, 3]), quat=numbers(np.roll(quat, 1)))


def make_model():
    # Include the housings that the visualization XML comments out.
    source = (INPUTS / "robot_motors.xml").read_text()
    source = re.sub(r'<!--\s*(<geom\b[^>]*?mesh="wj-[^"]+"\s*/>)\s*-->', r'\1', source)
    duck = ET.fromstring(source)
    duck.set("model", "SO-Duck crosswise")
    yaw = duck.find(".//body[@name='neck_yaw_assembly']")
    for node in list(yaw):
        if node.tag != "joint":
            yaw.remove(node)
    # Drop old head exclusions and actuator bindings together with the head subtree.
    contact = duck.find("contact")
    for exclude in list(contact):
        if "head_assembly" in exclude.attrib.values():
            contact.remove(exclude)
    actuators = duck.find("actuator")
    for actuator in list(actuators):
        if actuator.get("joint") == "head_roll":
            actuator.attrib.update(name="forearm_pitch", joint="forearm_pitch")
        elif actuator.get("joint") in ("left_antenna", "right_antenna"):
            actuators.remove(actuator)

    assets = duck.find("asset")
    for mesh in list(assets):
        path = INPUTS / "duck_meshes" / mesh.get("file")
        if path.is_file():
            mesh.set("file", "../collision/inputs/duck_meshes/" + path.name)
        else:
            assets.remove(mesh)
    available = {e.get("name") for e in assets}
    for body in duck.iter("body"):
        for geom in body.findall("geom"):
            if geom.get("mesh") and geom.get("mesh") not in available:
                body.remove(geom)  # unbundled electronics retain their upstream lumped body inertia
    ET.SubElement(assets, "mesh", name="crosswise_carrier", file="carrier.stl", scale=".001 .001 .001")
    yaw_from_print = np.linalg.inv(arm.NECK @ arm.PITCH @ arm.YAW) @ arm.TRUNK_FROM_PRINT
    cross = arm.rotate_about(arm.turn("z", 90), arm.MOUNT_CENTER)
    servo = arm.T(np.diag([-1, 1, -1]), [.0125, 0, .0187])
    yaw_servo = arm.T(np.diag([-1, -1, 1]), [.0325, 0, .12951])
    for name, mesh, transform, color in [
        ("carrier", "crosswise_carrier", yaw_from_print, "carrier"),
        ("yaw_case", "sts3215_03a_v1", yaw_from_print @ yaw_servo, "case"),
        ("pitch_case", "sts3215_03a_v1", yaw_from_print @ cross @ arm.ROOT @ servo, "case"),
    ]:
        ET.SubElement(yaw, "geom", name=name, mesh=mesh, type="mesh",
                      rgba=COLORS[color], **placement(transform))

    # Full tensor from the placed CAD solids and the parallel-axis theorem.
    # Both servo cases belong to yaw; only the fork rotates at forearm pitch.
    assembly = cq.importers.importStep(str(HERE / "connector_assembly.step")).val().Solids()
    carrier = cq.importers.importStep(str(HERE / "carrier.step")).val()
    yaw_case, mounted_carrier, pitch_case, fork = assembly
    if abs(mounted_carrier.Volume() - carrier.Volume()) > .01 or fork.Volume() < 90000:
        raise ValueError("Unexpected STEP assembly members or units")
    parts = [yaw_case, mounted_carrier, pitch_case]
    masses = np.array([.055, carrier.Volume() * 1.24e-6, .055])
    centers = np.array([p.Center().toTuple() for p in parts]) * .001
    center = np.average(centers, weights=masses, axis=0)
    inertia = np.zeros((3, 3))
    for part, m, pos in zip(parts, masses, centers):
        offset = pos - center
        inertia += np.array(cq.Shape.matrixOfInertia(part)) * (m / part.Volume() * 1e-6)
        inertia += m * (np.eye(3) * (offset @ offset) - np.outer(offset, offset))
    rotation = yaw_from_print[:3, :3]
    inertia = rotation @ inertia @ rotation.T
    ET.SubElement(yaw, "inertial", mass=numbers([sum(masses)]),
                  pos=numbers(rotation @ center + yaw_from_print[:3, 3]),
                  fullinertia=numbers(inertia[[0,1,2,0,0,1], [0,1,2,1,2,2]]))

    so = ET.parse(INPUTS / "so101.xml")
    lower = so.find(".//body[@name='lower_arm']")
    lower.attrib.update(name="forearm", **placement(FRAMES[3]))
    lower.find("joint").attrib.update(name="forearm_pitch", range=numbers(np.deg2rad(PITCH_LIMITS)))
    for body in lower.iter("body"):
        body.find("joint").attrib.pop("class")
        for geom in body.findall("geom"):
            if geom.get("class") == "collision":
                body.remove(geom)
                continue
            mesh = geom.get("mesh")
            if mesh not in available:
                ET.SubElement(assets, "mesh", name=mesh, file=f"../collision/inputs/arm_meshes/{mesh}.stl")
                available.add(mesh)
            geom.attrib.pop("class")
            geom.attrib.pop("material")
            geom.set("name", body.get("name") + "_" + mesh)
            geom.set("rgba", COLORS["case" if mesh.startswith("sts") else "arm"])
        name = body.find("joint").get("name")
        if name != "forearm_pitch":
            ET.SubElement(actuators, "motor", name=name, joint=name)
    yaw.append(lower)
    ET.SubElement(duck.find("worldbody"), "geom", name="floor", type="plane",
                  size="1 1 .01", rgba=".78 .81 .83 1")
    # Soft rear landing for the fixed finger, bonded to the unmodified stock roof.
    ET.SubElement(assets, "mesh", name="park_pad", file="park_pad.stl", scale=".001 .001 .001")
    trunk = duck.find(".//body[@name='trunk_assembly']")
    pad = ET.SubElement(trunk, "body", name="park_pad",
                        pos=numbers(np.array([*PAD_CENTER_MM, ROOF_MM]) * .001))
    ET.SubElement(pad, "geom", type="mesh", mesh="park_pad", density="1200", rgba=".64 .17 .23 1")
    sys.path.insert(0, str(ROOT / "study"))
    import pose_test
    leg_angles = {f"{side}_{name}": value for side, row in zip(pose_test.legs.SIDES, pose_test.REF)
                  for name, value in zip(pose_test.legs.NAMES, row)}
    joint_order = [j.get("name") for j in duck.find("worldbody").iter("joint")]
    keys = ET.SubElement(duck, "keyframe")
    path, grasp, contact = pickup()
    for name, row in [("ready", NEUTRAL), ("folded", PARK), ("pickup_open", path[grasp]),
                      ("pickup_contact", path[contact]), ("lifted", path[-1]), ("high_reach", HIGH)]:
        angles = {**leg_angles, **dict(zip((*ARM_JOINTS, "gripper"), np.deg2rad(row)))}
        base = [0, 0, pose_test.W0[2, 3], 1, 0, 0, 0]
        ET.SubElement(keys, "key", name=name, qpos=numbers(base + [angles[j] for j in joint_order]))
    # The recorded stance and paths are in geometric joint coordinates (radians).
    ET.indent(duck)
    return ET.tostring(duck, encoding="unicode") + "\n"


if __name__ == "__main__":
    pad = cq.Solid.makeCylinder(PAD_RADIUS_MM, PAD_HEIGHT_MM)
    cq.exporters.export(pad, str(HERE / "park_pad.stl"), tolerance=.02, angularTolerance=.08)
    (HERE / "robot.xml").write_text(make_model())
