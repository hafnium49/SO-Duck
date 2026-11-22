"""Build the 17-actuator spatial arm packaging model. Dimensions in metres."""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from deck import drilled_roof
from links import link, transform
import cadquery as cq
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
DONOR = HERE.parent / "crosswise"
JOINTS = (
    "shoulder_yaw",
    "shoulder_pitch",
    "forearm_pitch",
    "forearm_roll",
    "wrist_flex",
    "wrist_roll",
)
# Pivot positions in the neutral torso frame. Roll/bend centers are deliberately separated.
CENTERS = np.array(
    [
        [0.020, 0, 0.157],
        [0.035, 0, 0.216],
        [0.235, 0, 0.216],
        [0.395, 0, 0.216],
        [0.448, 0.041, 0.216],
        [0.524, 0.012, 0.216],
    ]
)
AXES = np.array([[0, 0, 1], [0, -1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [-1, 0, 0]])
LIMITS = [(-150, 150), (-105, 125), (-147, 147), (-155, 155), (-157, 157), (-155, 160)]


def fmt(x):
    return " ".join(f"{v:.10g}" for v in np.asarray(x))


def quat(rotation):
    return fmt(np.roll(rotation.as_quat(), 1))


def add_keyframes(root):
    import sys

    sys.path.insert(0, str(DONOR))
    from geometry import pose_test, screen
    from motion import JAW_OPEN, JAW_CONTACT

    keys = ET.SubElement(root, "keyframe")
    order = [e.get("name") for e in root.find("worldbody").iter("joint")]
    names = (
        [f"{side}_{j}" for side in screen.legs.SIDES for j in screen.legs.NAMES]
        + list(JOINTS)
        + ["gripper"]
    )

    def add(name, leg, arm, jaw):
        world = pose_test.ANCHOR[0] @ np.linalg.inv(
            screen.legs.leg_fk("left", leg[:5])[0]
        )
        q = dict(zip(names, [*leg, *arm, np.deg2rad(jaw)], strict=True))
        base = np.r_[
            world[:3, 3], np.roll(Rotation.from_matrix(world[:3, :3]).as_quat(), 1)
        ]
        ET.SubElement(keys, "key", name=name, qpos=fmt([*base, *[q[j] for j in order]]))

    first = np.loadtxt(HERE / "targets.csv", delimiter=",", skiprows=1, max_rows=1)[1:]
    add("target_00", first[:10], first[10:], 0.0)
    floor = np.loadtxt(HERE / "floor.csv", delimiter=",", skiprows=1)
    nominal = pose_test.REF.ravel()
    add("floor_open", nominal, floor[0], JAW_OPEN)
    add("floor_contact", nominal, floor[0], JAW_CONTACT)
    add("lifted", nominal, floor[-1], JAW_CONTACT)
    for name in ("park", "high"):
        arm = np.loadtxt(HERE / (name + ".csv"), delimiter=",", skiprows=1)
        add(name, nominal, arm, 0.0)


def build():
    root = ET.parse(DONOR / "robot.xml").getroot()
    root.set("model", "SO-Duck coverage prototype")
    trunk = root.find(".//body[@name='trunk_assembly']")
    old_neck = trunk.find("body[@name='neck_pitch_assembly']")
    gripper = deepcopy(old_neck.find(".//body[@name='gripper']"))
    trunk.remove(old_neck)

    root.remove(root.find("keyframe"))
    for e in list(root.find("contact")):
        if any(
            n in ("neck_yaw_assembly", "head_pitch_to_yaw") for n in e.attrib.values()
        ):
            root.find("contact").remove(e)
    assets = root.find("asset")
    for mesh in list(assets):
        if mesh.get("name") == "park_pad":
            mesh.set("file", "../crosswise/park_pad.stl")
        if mesh.get("name") == "crosswise_carrier":
            assets.remove(mesh)
    roof = drilled_roof(HERE)
    cq.exporters.export(
        roof, str(HERE / "roof.stl"), tolerance=0.03, angularTolerance=0.1
    )
    assets.find("mesh[@name='body_middle_top']").attrib.update(
        file="roof.stl", scale=".001 .001 .001"
    )
    actuators = root.find("actuator")
    for e in list(actuators):
        if e.get("joint") in (
            "neck_pitch",
            "head_pitch",
            "head_yaw",
            *JOINTS,
            "gripper",
        ):
            actuators.remove(e)
    parent = ET.SubElement(trunk, "body", name="upper_mount")
    parent_center = np.zeros(3)
    parent_rot = Rotation.identity()
    parent_housing = Rotation.identity()
    assembly = cq.Assembly(name="coverage_arm_links")
    for i, (name, center, axis, limits) in enumerate(
        zip(JOINTS, CENTERS, AXES, LIMITS)
    ):
        # Drive horn at the joint origin. The motor lies on the parent side.
        if i == 5:
            rot = Rotation.from_euler("y", -90, degrees=True)
        else:
            x = np.array([0, 0, -1]) if abs(axis[0]) else np.array([1, 0, 0])
            rot = Rotation.from_matrix(np.column_stack((x, np.cross(axis, x), axis)))
        local = parent_rot.inv().apply(center - parent_center)
        housing = rot * Rotation.from_euler("x", 180, degrees=True) if i == 0 else rot
        if i == 5:
            housing = rot * Rotation.from_euler("z", 92.7898, degrees=True)
        else:
            if i == 3:
                housing = rot * Rotation.from_euler("y", 180, degrees=True)
            incoming = housing.inv().apply(parent_center - center)
            heading = np.arctan2(incoming[1], incoming[0])
            housing = housing * Rotation.from_euler(
                "z", heading + (1.15 if i == 1 else 0.0)
            )
        case_rot = housing * Rotation.from_euler("y", 180, degrees=True)
        # Place each drive case on the far side of its driven horn plane.
        case = center + housing.apply([0.0125, 0, 0.0187])
        ET.SubElement(
            parent,
            "geom",
            name=name + "_case",
            type="mesh",
            mesh="sts3215_03a_no_horn_v1" if i == 5 else "sts3215_03a_v1",
            pos=fmt(parent_rot.inv().apply(case - parent_center)),
            quat=quat(parent_rot.inv() * case_rot),
            mass=".055",
            rgba=".24 .26 .29 1",
        )
        part = "shoulder_mount" if i == 0 else name + "_link"
        foot = np.array([-0.092, 0, 0.1078909]) if i == 0 else parent_center
        shape = link(foot * 1000, parent_housing, center * 1000, housing, base=(i == 0))
        export_shape = shape
        if name == "forearm_pitch":
            # STEP otherwise loses the outer boundary of a periodic face in
            # the long shoulder-to-elbow tube. Preserve its analytic surface.
            from OCP.ShapeUpgrade import ShapeUpgrade_ShapeDivideClosed

            divide = ShapeUpgrade_ShapeDivideClosed(shape.wrapped)
            divide.SetNbSplitPoints(4)
            divide.Perform()
            export_shape = cq.Shape.cast(divide.Result())
        assembly.add(export_shape, name=part)
        local_shape = transform(
            shape, parent_rot.inv(), -1000 * parent_rot.inv().apply(parent_center)
        )
        cq.exporters.export(
            local_shape,
            str(HERE / (part + ".stl")),
            tolerance=0.04,
            angularTolerance=0.12,
        )
        ET.SubElement(
            assets, "mesh", name=part, file=part + ".stl", scale=".001 .001 .001"
        )
        ET.SubElement(
            parent,
            "geom",
            type="mesh",
            mesh=part,
            name=part,
            mass=f"{shape.Volume()*1.24e-6:.9g}",
            rgba=".12 .57 .55 1",
        )
        if i == 5:
            body = gripper
            parent.append(body)
        else:
            body = ET.SubElement(parent, "body", name=name + "_linkage")
            ET.SubElement(body, "joint", name=name, type="hinge", axis="0 0 1")
        body.set("pos", fmt(local))
        body.set("quat", quat(parent_rot.inv() * rot))
        body.find("joint").set("range", fmt(np.deg2rad(limits)))
        ET.SubElement(actuators, "motor", name=name, joint=name)
        parent_housing = housing
        parent, parent_center, parent_rot = body, center, rot
    ET.SubElement(actuators, "motor", name="gripper", joint="gripper")
    assembly.export(str(HERE / "mounts.step"))
    add_keyframes(root)
    ET.indent(root)
    (HERE / "robot.xml").write_text(ET.tostring(root, encoding="unicode") + "\n")


if __name__ == "__main__":
    build()
