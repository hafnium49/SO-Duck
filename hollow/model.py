"""The hollow design's model: its tested pairs, and the earlier reconstruction.

robot.xml is written by cad/build_parts.py from the three CAD parts. robot_model() and
tested_pairs() serve that model.

build() is the earlier reconstruction, kept for reference. It rebuilt a model from what
the review viewer stored, before the parts were drawn, and writes robot.xml and meshes/:
run it only to reproduce the earlier results, then run cad/build_parts.py again.

The author's model of the review design is not in the repository. The reconstruction
takes from the viewer:

- Joint frames come from the stored body transforms. Each arm joint is a hinge whose
  child origin stays fixed in the parent. The fit reproduces every stored frame.
- Joint ranges are the largest and smallest angle stored for each joint, widened by the
  2 degree reserve of the replay. The real ranges are unknown.
- Meshes on the two sides of one arm joint touch on the display grid. See NEIGHBOURS.
- Arm meshes are the viewer's display meshes on a 0.1 mm grid. The gripper and jaw use
  the full SO-101 meshes of the six-part model, whose frames they share.
- Masses are estimates. See arm_masses in inspect_viewer.py.
- The trunk, legs and parking pad come from coverage/robot.xml, with the original roof.

    python hollow/model.py path/to/viewer.html

robot.xml in this directory was built from the viewer of commit 9f2be21, the last one
with only the author's poses. Building from a later viewer changes the joint ranges.
"""

import sys
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "coverage"), str(HERE.parent / "crosswise")]

from inspect_viewer import REVIEW_MESHES, arm_masses, decode_clip, decode_mesh, load_viewer  # noqa: E402
from replay import FINGER, JOINTS, collision_pairs  # noqa: E402
from geometry import Robot  # noqa: E402

UPPER = ("upper_mount", *[j + "_linkage" for j in JOINTS[:5]])
# Model body, the viewer body that carries its frame, and the viewer meshes it holds.
BODIES = [
    ("upper_mount", "trunk_assembly", ("shoulder_yaw_case", "shoulder_mount")),
    ("shoulder_yaw_linkage", "shoulder_pitch_link", ("shoulder_pitch_case", "shoulder_collar", "shoulder_pitch_link")),
    ("shoulder_pitch_linkage", "forearm_pitch_link", ("forearm_pitch_case", "forearm_pitch_link")),
    ("forearm_pitch_linkage", "forearm_roll_link", ("forearm_roll_case", "forearm_roll_link")),
    ("forearm_roll_linkage", "wrist_flex_link", ("wrist_flex_case", "wrist_flex_link")),
    ("wrist_flex_linkage", "wrist_roll_link", ("wrist_roll_case", "wrist_roll_link")),
    ("gripper", "gripper", ()),
    (FINGER, FINGER, ()),
]
VIEWER_BODY = {model: viewer for model, viewer, _ in BODIES}
# The display meshes sit on a 0.1 mm grid and touch where a link meets its neighbour
# across a joint. The triangle test of the replay counts touching faces as a hit, so
# meshes on the two sides of one joint get the test that ignores touching faces.
NEIGHBOURS = [(a, b) for (_, _, near), (_, _, far) in zip(BODIES[:5], BODIES[1:6]) for a in near for b in far]
NEIGHBOURS += [("wrist_roll_link", "gripper")]
# Five of them are a link seated on the horn of the servo before it. They touch by design.
# The part tests in tests/ sweep each of them with only the 13 mm around the horn left out.
SEATS = [
    ("shoulder_yaw_case", "shoulder_pitch_link"),
    ("shoulder_pitch_case", "forearm_pitch_link"),
    ("forearm_pitch_case", "forearm_roll_link"),
    ("forearm_roll_case", "wrist_flex_link"),
    ("wrist_flex_case", "wrist_roll_link"),
]
# Stored angles are rounded to 0.01 degree.
RESERVE_DEG = 2.01


def robot_model():
    return Robot(HERE / "robot.xml", JOINTS, UPPER)


def tested_pairs(robot):
    """Pairs of mesh groups that must not intersect, for the model that cad/build_parts.py writes.

    Every pair across a joint gets the triangle test, which counts touching faces as a
    hit. The seats are left out, and so is the stock gripper on its stock wrist link.
    The mount is tested against the trunk: the neck servo it replaces is not in the model.
    """
    seats = SEATS + [("wrist_roll_link", "gripper")]
    return list(collision_pairs(robot, seats=seats, more_parts=("shoulder_collar",))) + [("shoulder_mount", "trunk_assembly")]


def all_frames(data):
    """Every stored frame as 4x4 transforms, with the stored joint angles in degrees."""
    worlds, angles = [], []
    for clip in data["poses"]:
        world = decode_clip(data, clip["name"])[1]
        pose = np.tile(np.eye(4), (*world.shape[:2], 1, 1))
        pose[:, :, :3] = world
        worlds.append(pose)
        angles.append(np.array(clip["q"]))
    return np.concatenate(worlds), np.concatenate(angles)


def fit_joints(data):
    """Position, rotation at angle zero and stored range of the seven arm joints."""
    bodies = data["bodies"]
    world, angles = all_frames(data)
    joints = []
    for k in range(7):
        parent, child = VIEWER_BODY[BODIES[k][0]], VIEWER_BODY[BODIES[k + 1][0]]
        relative = np.linalg.inv(world[:, bodies.index(parent)]) @ world[:, bodies.index(child)]
        position = relative[:, :3, 3].mean(0)
        assert np.abs(relative[:, :3, 3] - position).max() < 1e-6, f"{child} origin moves in {parent}"
        q = np.deg2rad(angles[:, k])
        turn = Rotation.from_rotvec(np.outer(q, [0, 0, 1])).as_matrix()
        left, _, right = np.linalg.svd((relative[:, :3, :3] @ turn.transpose(0, 2, 1)).mean(0))
        zero = left @ right
        fitted = zero @ turn
        error = Rotation.from_matrix(fitted.transpose(0, 2, 1) @ relative[:, :3, :3]).magnitude()
        assert np.rad2deg(error.max()) < 0.03, f"{child} is not a hinge about its z axis"
        joints.append((position, zero, angles[:, k].min(), angles[:, k].max()))
    return joints


def write_stl(path, vertices, faces):
    """Binary STL in millimetres from vertices on the viewer's 0.1 mm grid."""
    triangles = (vertices[faces] * 0.1).astype("<f4")
    record = np.zeros(len(triangles), dtype=[("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    record["v"] = triangles
    header = f"SO-Duck {path.stem} | viewer display mesh, body-local mm".encode().ljust(80, b" ")
    path.write_bytes(header + len(triangles).to_bytes(4, "little") + record.tobytes())


def fmt(values):
    return " ".join(f"{v:.10g}" for v in np.asarray(values, dtype=float).ravel())


def build(viewer):
    data = load_viewer(viewer)
    bodies = data["bodies"]
    joints = fit_joints(data)
    masses = arm_masses(data, decode_clip(data, "targets")[1])

    root = ET.parse(HERE.parent / "coverage" / "robot.xml").getroot()
    root.set("model", "SO-Duck hollow design, reconstructed")
    root.remove(root.find("keyframe"))
    trunk = root.find(".//body[@name='trunk_assembly']")
    old = trunk.find("body[@name='upper_mount']")
    gripper = deepcopy(old.find(".//body[@name='gripper']"))
    trunk.remove(old)
    assets = root.find("asset")
    roof = assets.find("mesh[@name='body_middle_top']")
    roof.attrib.pop("scale")
    roof.set("file", "../collision/inputs/duck_meshes/body_middle_top.stl")
    for mesh in list(assets):
        if mesh.get("name") == "shoulder_mount" or mesh.get("name").endswith("_link"):
            assets.remove(mesh)

    (HERE / "meshes").mkdir(exist_ok=True)
    review = {"shoulder_mount": "mount", "forearm_roll_link": "forearm", "wrist_flex_link": "wrist"}
    parent = trunk
    for k, (name, viewer_body, parts) in enumerate(BODIES[:6]):
        body = ET.SubElement(parent, "body", name=name)
        if k:
            position, zero, low, high = joints[k - 1]
            body.set("pos", fmt(position))
            body.set("quat", fmt(np.roll(Rotation.from_matrix(zero).as_quat(), 1)))
            limits = np.deg2rad([low - RESERVE_DEG, high + RESERVE_DEG])
            ET.SubElement(body, "joint", name=JOINTS[k - 1], type="hinge", axis="0 0 1", range=fmt(limits))
        entries = masses[viewer_body]
        total = sum(mass for mass, _ in entries)
        centre = sum(mass * np.asarray(c) for mass, c in entries) / total
        # Only mass and centre of mass matter for the static checks.
        ET.SubElement(body, "inertial", pos=fmt(centre), mass=f"{total:.9g}", diaginertia="1e-5 1e-5 1e-5")
        for part in parts:
            if part in review:
                file = "review-meshes/" + REVIEW_MESHES[review[part]]
            else:
                groups = [decode_mesh(g) for g in data["meshes"] if bodies[g["b"]] == part and not g.get("sectionOf")]
                offsets = np.cumsum([0] + [len(v) for v, _ in groups[:-1]])
                write_stl(
                    HERE / "meshes" / (part + ".stl"),
                    np.concatenate([v for v, _ in groups]),
                    np.concatenate([f + o for (_, f), o in zip(groups, offsets)]),
                )
                file = f"meshes/{part}.stl"
            ET.SubElement(assets, "mesh", name=part, file=file, scale=".001 .001 .001", inertia="shell")
            colour = ".24 .26 .29 1" if part.endswith("_case") else ".62 .68 .71 1" if part not in review else ".12 .57 .55 1"
            ET.SubElement(body, "geom", type="mesh", mesh=part, name=part, rgba=colour)
        parent = body
    position, zero, low, high = joints[5]
    gripper.set("pos", fmt(position))
    gripper.set("quat", fmt(np.roll(Rotation.from_matrix(zero).as_quat(), 1)))
    gripper.find("joint").set("range", fmt(np.deg2rad([low - RESERVE_DEG, high + RESERVE_DEG])))
    parent.append(gripper)
    ET.indent(root)
    (HERE / "robot.xml").write_text(ET.tostring(root, encoding="unicode") + "\n")
    ranges = ", ".join(f"{JOINTS[k]} {joints[k][2]:.1f} to {joints[k][3]:.1f}" for k in range(6))
    print("Wrote robot.xml. Stored joint angles (deg): " + ranges)


if __name__ == "__main__":
    build(Path(sys.argv[1]))
