"""Write the hollow-arm model (MJCF) from joint frames, masses and mesh files.

The trunk, legs, parking pad and gripper come from coverage/robot.xml, with the
original roof. The arm between them is written from the arguments.
"""

import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import numpy as np

import params

HERE = Path(__file__).resolve().parent
HOLLOW = HERE.parent
# Model bodies from the trunk outward, and the joint that moves each (none for the mount).
CHAIN = (
    ("upper_mount", None),
    ("shoulder_yaw_linkage", "shoulder_yaw"),
    ("shoulder_pitch_linkage", "shoulder_pitch"),
    ("forearm_pitch_linkage", "forearm_pitch"),
    ("forearm_roll_linkage", "forearm_roll"),
    ("wrist_flex_linkage", "wrist_flex"),
)
# Meshes each body carries: the three review meshes and the viewer's display meshes.
DISPLAY = {
    "upper_mount": ("shoulder_yaw_case", "shoulder_mount"),
    "shoulder_yaw_linkage": ("shoulder_pitch_case", "shoulder_collar", "shoulder_pitch_link"),
    "shoulder_pitch_linkage": ("forearm_pitch_case", "forearm_pitch_link"),
    "forearm_pitch_linkage": ("forearm_roll_case", "forearm_roll_link"),
    "forearm_roll_linkage": ("wrist_flex_case", "wrist_flex_link"),
    "wrist_flex_linkage": ("wrist_roll_case", "wrist_roll_link"),
}
REVIEW = {
    "shoulder_mount": "review-meshes/neck-socket-mount.preview.stl",
    "forearm_roll_link": "review-meshes/elbow-to-forearm-roll.preview.stl",
    "wrist_flex_link": "review-meshes/forearm-roll-to-wrist.preview.stl",
}
# Mass estimates at the present lengths, per body, as (kg, centre in the body frame, m).
# Stock links carry the SO-101 link inertials. The first entry of a body with a custom
# part is that part, at the enclosed volume of its review mesh and 1.24 g/cm3; the second
# is its servo at 55 g.
PRESENT_MASSES = {
    "upper_mount": [(0.037374, (-0.020068, 0.000100, 0.114512)), (0.055000, (-0.011774, 0.000104, 0.158443))],
    "shoulder_yaw_linkage": [(0.100006, (0.030760, -0.000017, 0.025271))],
    "shoulder_pitch_linkage": [(0.103000, (-0.049205, -0.075641, 0.018409))],
    "forearm_pitch_linkage": [(0.051275, (0.065218, -0.028113, 0.018200)), (0.055000, (0.121443, -0.038774, 0.018202))],
    "forearm_roll_linkage": [(0.031863, (-0.033291, 0.000001, 0.033248)), (0.055000, (-0.067215, -0.000129, 0.061998))],
    "wrist_flex_linkage": [(0.079000, (-0.000103, -0.038614, 0.028116))],
}


CUSTOM = {"shoulder_mount": "neck_socket_mount", "forearm_roll_link": "elbow_to_forearm_roll", "wrist_flex_link": "forearm_roll_to_wrist"}
# The Duck's neck servo and its horns, which the neck socket mount replaces.
NECK_SERVO = {"wj-wk00-0123middlecase_56", "wj-wk00-0124bottomcase_45", "wj-wk00-0122topcabinetcase_95", "drive_palonier", "passive_palonier"}


def built_meshes():
    """Body name to (geom name, mesh file relative to hollow/) for the parts cad/build_parts.py writes."""
    return {body: [(part, f"parts/{CUSTOM[part]}.stl" if part in CUSTOM else f"parts/stock/{part}.stl") for part in parts] for body, parts in DISPLAY.items()}


def display_meshes():
    """Body name to (geom name, mesh file relative to hollow/) for the present display meshes."""
    return {body: [(part, REVIEW.get(part, f"meshes/{part}.stl")) for part in parts] for body, parts in DISPLAY.items()}


def stored_ranges():
    """Arm joint ranges of the present model in degrees."""
    model = ET.parse(HOLLOW / "robot.xml")
    names = [joint for _, joint in CHAIN[1:]] + ["wrist_roll"]
    return {
        name: tuple(np.rad2deg(np.fromstring(model.find(f".//joint[@name='{name}']").get("range"), sep=" ")))
        for name in names
    }


def estimated_masses(frames):
    """Body name to (kg, centre) before the parts are drawn.

    A shortened link keeps its present mass, which overstates it. Its servo moves with the
    joint and the link's own centre moves half as far.
    """
    base = params.joint_frames(0, 0, 0)
    moved = {
        "forearm_pitch_linkage": frames["forearm_roll"][0] - base["forearm_roll"][0],
        "forearm_roll_linkage": frames["wrist_flex"][0] - base["wrist_flex"][0],
    }
    masses = {}
    for body, entries in PRESENT_MASSES.items():
        entries = [(mass, np.array(centre)) for mass, centre in entries]
        if body in moved:
            (link, link_centre), (servo, servo_centre) = entries
            entries = [(link, link_centre + moved[body] / 2), (servo, servo_centre + moved[body])]
        total = sum(mass for mass, _ in entries)
        masses[body] = (total, sum(mass * centre for mass, centre in entries) / total)
    return masses


def _numbers(values):
    return " ".join(f"{v:.10g}" for v in np.asarray(values, dtype=float).ravel())


def write_model(path, frames, masses, meshes, ranges, neck_servo=True):
    """Write the model next to hollow/robot.xml, so that its mesh paths resolve.

    frames: joint name to (position m, quaternion wxyz), as params.joint_frames gives.
    masses: body name to (kg, centre m). meshes: body name to (geom name, mesh file in
    millimetres, relative to the model). ranges: joint name to (low, high) in degrees.
    neck_servo: whether the trunk keeps the Duck's neck servo; the built mount replaces it.
    """
    path = Path(path)
    if path.parent.resolve() != HOLLOW.resolve():
        raise ValueError("the model must be written in hollow/, where its mesh paths resolve")
    root = ET.parse(HOLLOW.parent / "coverage" / "robot.xml").getroot()
    root.set("model", "SO-Duck hollow arm")
    root.remove(root.find("keyframe"))
    trunk = root.find(".//body[@name='trunk_assembly']")
    old = trunk.find("body[@name='upper_mount']")
    gripper = deepcopy(old.find(".//body[@name='gripper']"))
    trunk.remove(old)
    if not neck_servo:
        for geom in trunk.findall("geom"):
            # the hip-yaw servos use the same meshes, further back
            if geom.get("mesh") in NECK_SERVO and float(geom.get("pos").split()[0]) > -0.005:
                trunk.remove(geom)
    assets = root.find("asset")
    roof = assets.find("mesh[@name='body_middle_top']")
    roof.attrib.pop("scale", None)
    roof.set("file", "../collision/inputs/duck_meshes/body_middle_top.stl")
    for mesh in list(assets):
        if mesh.get("name") == "shoulder_mount" or mesh.get("name").endswith("_link"):
            assets.remove(mesh)

    parent = trunk
    for body_name, joint in CHAIN:
        body = ET.SubElement(parent, "body", name=body_name)
        if joint:
            position, quaternion = frames[joint]
            body.set("pos", _numbers(position))
            body.set("quat", _numbers(quaternion))
            ET.SubElement(body, "joint", name=joint, type="hinge", axis="0 0 1", range=_numbers(np.deg2rad(ranges[joint])))
        mass, centre = masses[body_name]
        # Only mass and centre of mass matter for the static checks.
        ET.SubElement(body, "inertial", pos=_numbers(centre), mass=f"{mass:.9g}", diaginertia="1e-5 1e-5 1e-5")
        for geom, file in meshes[body_name]:
            ET.SubElement(assets, "mesh", name=geom, file=file, scale=".001 .001 .001", inertia="shell")
            colour = ".24 .26 .29 1" if geom.endswith("_case") else ".12 .57 .55 1" if geom in CUSTOM else ".62 .68 .71 1"
            ET.SubElement(body, "geom", type="mesh", mesh=geom, name=geom, rgba=colour)
        parent = body
    position, quaternion = frames["wrist_roll"]
    gripper.set("pos", _numbers(position))
    gripper.set("quat", _numbers(quaternion))
    gripper.find("joint").set("range", _numbers(np.deg2rad(ranges["wrist_roll"])))
    parent.append(gripper)
    ET.indent(root)
    path.write_text(ET.tostring(root, encoding="unicode") + "\n")
