"""Build one adapter (mm) and assemble the two unmodified donor interfaces."""

from pathlib import Path
from copy import deepcopy
import re
import sys
import xml.etree.ElementTree as ET

import cadquery as cq
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
INPUT = ROOT / "collision/inputs"
sys.path.insert(0, str(ROOT / "legs"))
import legs

SO_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
JOINTS = ("neck_pitch", *SO_JOINTS)
# CAD coordinates are level at zero neck pitch, centered between the sheets.
TOP = 53.0
BASE_X = -34.0
CLOCK_DEG = 0.0
CLOCK = Rotation.from_euler("z", CLOCK_DEG, degrees=True)
# Re-clock about the SO shoulder pan center, keeping its pivot over the Duck neck.
PAN_X = 38.8353
ARM_ORIGIN = np.array([BASE_X + PAN_X, 0, TOP + 2.4]) + CLOCK.apply([-PAN_X, 0, 0])
BASE_HOLES = [
    tuple((ARM_ORIGIN + CLOCK.apply([x, side * y, 0]))[:2])
    for x, y in [(-3.39803, 36.27649), (40.41990, 28.46)]
    for side in (-1, 1)
]
SHEET_HOLES = [
    (x, side * (16 - 0.15), 37.0 if side == 1 else 33.25)
    for x in (-10.25, 10.25)
    for side in (-1, 1)
]
CONTROL = dict(connector=".1 .55 .52 1", case=".22 .28 .31 1", arm=".91 .62 .23 1")


def post(rad, height, point, direction=(0, 0, 1)):
    return cq.Solid.makeCylinder(rad, height, cq.Vector(*point), cq.Vector(*direction))


def bracket():
    # Flat bearing deck, two independent inside cheeks, four existing sheet bores.
    from scipy.spatial import ConvexHull

    sites = np.array(BASE_HOLES + [(15.5, 16), (15.5, -16), (-15.5, 16), (-15.5, -16)])
    outline = sites[ConvexHull(sites).vertices]
    deck = (
        cq.Workplane("XY")
        .polyline(outline.tolist())
        .close()
        .offset2D(6)
        .extrude(5)
        .val()
        .translate((0, 0, TOP - 5))
    )
    for side, z in [(1, 37.0), (-1, 33.25)]:
        y = side * (15.85 - 2.75)
        lug = (
            cq.Workplane("XY")
            .box(31, 5.5, TOP - z + 5, centered=(True, True, False))
            .val()
            .translate((0, y, z - 5))
        )
        deck = deck.fuse(lug)
    roots = [
        edge
        for edge in deck.Edges()
        if abs(edge.Center().z - (TOP - 5)) < 0.001
        and abs(abs(edge.Center().y) - 10.35) < 0.001
        and edge.BoundingBox().xlen > 30
    ]
    assert len(roots) == 2
    deck = deck.fillet(2.0, roots)
    for x, y, z in SHEET_HOLES:
        deck = deck.cut(post(1.10, 10, (x, y - 8 if y > 0 else y - 2, z), (0, 1, 0)))
    for x, y in BASE_HOLES:
        deck = deck.cut(post(1.65, 8, (x, y, TOP - 6)))
    shape = deck.clean()
    assert len(shape.Solids()) == 1 and shape.isValid()
    return shape


def numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def placement(t):
    return dict(
        pos=numbers(t[:3, 3]),
        quat=numbers(np.roll(Rotation.from_matrix(t[:3, :3]).as_quat(), 1)),
    )


def add_keyframes(duck):
    import csv

    sys.path.insert(0, str(ROOT / "study"))
    import pose_test

    names = [f"{s}_{j}" for s in legs.SIDES for j in legs.NAMES] + list(JOINTS)
    order = [e.get("name") for e in duck.find("worldbody").iter("joint")]
    keys = ET.SubElement(duck, "keyframe")
    with (HERE / "examples.csv").open() as fp:
        poses = {
            r["name"]: [float(r[n + "_rad"]) for n in names] for r in csv.DictReader(fp)
        }
    travel = np.loadtxt(HERE / "pickup.csv", delimiter=",", skiprows=1)
    poses.update(
        pickup_contact=[*travel[0], np.deg2rad(3.32761821)],
        lifted=[*travel[-1], np.deg2rad(3.32761821)],
    )
    for name, vals in poses.items():
        placement = pose_test.ANCHOR[0] @ np.linalg.inv(
            legs.leg_fk("left", vals[:5])[0]
        )
        q = dict(zip(names, vals, strict=True))
        base = [
            *placement[:3, 3],
            *np.roll(Rotation.from_matrix(placement[:3, :3]).as_quat(), 1),
        ]
        targets = [
            q[a.get("joint")] if a.tag == "position" else 0
            for a in duck.find("actuator")
        ]
        ET.SubElement(
            keys,
            "key",
            name=name,
            qpos=numbers([*base, *[q[n] for n in order]]),
            ctrl=numbers(targets),
        )


def build():
    bracket_part = bracket()
    bracket_part.exportStep(str(HERE / "connector.step"))
    cq.exporters.export(
        bracket_part,
        str(HERE / "connector.stl"),
        tolerance=0.025,
        angularTolerance=0.10,
    )
    duck = ET.fromstring(
        re.sub(
            r'<!--\s*(<geom\b[^>]*?mesh="wj-[^"]+"\s*/>)\s*-->',
            r"\1",
            (INPUT / "robot_motors.xml").read_text(),
        )
    )
    duck.set("model", "SO-Duck stock arm on the first neck pivot")
    neck = duck.find(".//body[@name='neck_pitch_assembly']")
    neck.find("joint").set("range", f"-0.349066 {np.deg2rad(30):.12g}")
    for e in list(neck):
        if e.tag == "joint" or (
            e.tag == "geom" and e.get("mesh") in ("neck_left_sheet", "neck_right_sheet")
        ):
            continue
        neck.remove(e)
    # Only the two sheet prints remain on this link. Derive their inertias
    # from their mesh volumes instead of inheriting the removed head servo mass.
    import trimesh

    pieces = []
    for e in neck.findall("geom"):
        mesh = trimesh.load(INPUT / "duck_meshes" / (e.get("mesh") + ".stl"))
        mesh.apply_transform(legs.rest(e))
        pieces.append(mesh)
    props = trimesh.util.concatenate(pieces).mass_properties
    I = props.inertia * 1240.0
    ET.SubElement(
        neck,
        "inertial",
        pos=numbers(props.center_mass),
        mass=numbers([props.volume * 1240]),
        fullinertia=numbers(I[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]),
    )
    contact = duck.find("contact")
    bodies = {e.get("name") for e in duck.iter("body")}
    for e in list(contact):
        if e.get("body1") not in bodies or e.get("body2") not in bodies:
            contact.remove(e)
    acts = duck.find("actuator")
    joints = {e.get("name") for e in duck.find("worldbody").iter("joint")}
    for e in list(acts):
        if e.get("joint") not in joints:
            acts.remove(e)
    assets = duck.find("asset")
    used = {g.get("mesh") for g in duck.iter("geom")}
    for mesh in list(assets):
        if mesh.get("name") not in used:
            assets.remove(mesh)
            continue
        filename = mesh.get("file")
        prefix = (
            "../collision/inputs/duck_meshes/"
            if (INPUT / "duck_meshes" / filename).exists()
            else "assets/"
        )
        mesh.set("file", prefix + filename)
    ET.SubElement(
        assets,
        "mesh",
        name="single_connector",
        file="connector.stl",
        scale=".001 .001 .001",
    )
    datum = np.eye(4)
    datum[:3, 3] = [0.001, 0.00010, 0.0900009]
    connector = ET.SubElement(
        neck,
        "body",
        name="adapter",
        **placement(np.linalg.inv(legs.rest(neck)) @ datum),
    )
    ET.SubElement(
        connector,
        "geom",
        name="adapter",
        type="mesh",
        mesh="single_connector",
        rgba=CONTROL["connector"],
    )
    # Model hardware mass and use the exact CAD mass properties for the print.
    mass = bracket_part.Volume() * 1.24e-6
    com = np.array(bracket_part.Center().toTuple()) / 1000.0
    inertia = np.array(cq.Shape.matrixOfInertia(bracket_part)) * 1.24e-12
    ET.SubElement(
        connector,
        "inertial",
        mass=numbers([mass]),
        pos=numbers(com),
        fullinertia=numbers(inertia[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]),
    )
    # Conservative envelopes for ordinary metal fasteners. One body per stack
    # gives their masses separate from the connector's explicit CAD inertia.
    for i, (x, y) in enumerate(BASE_HOLES):
        hw = ET.SubElement(
            connector,
            "body",
            name=f"base_bolt_{i}",
            pos=numbers(np.array([x, y, TOP - 5]) / 1000),
        )
        for z, r, h, m in [
            (-1.5, 2.75, 3, 0.0006),
            (15, 1.5, 30, 0.0017),
            (23, 6.0, 1, 0.0008),
            (25.0, 3.2, 3, 0.0004),
        ]:
            ET.SubElement(
                hw,
                "geom",
                type="cylinder",
                pos=numbers([0, 0, z / 1000]),
                size=numbers([r / 1000, h / 2000]),
                mass=str(m),
                rgba=".35 .37 .39 1",
            )
    for i, (x, y, z) in enumerate(SHEET_HOLES):
        side = np.sign(y)
        under = 19.05 if side > 0 else 17.80
        hw = ET.SubElement(
            connector,
            "body",
            name=f"sheet_bolt_{i}",
            pos=numbers(np.array([x, side * under, z]) / 1000),
        )
        for d, r, h, m in [
            (1, 1.9, 2, 0.00016),
            (-6, 1, 12, 0.00030),
            (-float(under - 10.35 + 1.25), 2.3, 2.5, 0.00012),
        ]:
            ET.SubElement(
                hw,
                "geom",
                type="cylinder",
                pos=numbers([0, side * d / 1000, 0]),
                quat=".70710678 .70710678 0 0",
                size=numbers([r / 1000, h / 2000]),
                mass=str(m),
                rgba=".35 .37 .39 1",
            )
    so = ET.parse(INPUT / "so101.xml")
    for e in so.findall("default/default"):
        duck.find("default").append(deepcopy(e))
    base = deepcopy(so.find(".//body[@name='base']"))
    base.set("name", "so101_base")
    base.set("pos", numbers(np.array(ARM_ORIGIN) / 1000))
    base.set("quat", numbers(np.roll(CLOCK.as_quat(), 1)))
    connector.append(base)
    for mesh in so.findall("asset/mesh"):
        filename = mesh.get("file")
        prefix = (
            "../collision/inputs/arm_meshes/"
            if (INPUT / "arm_meshes" / filename).exists()
            else "assets/"
        )
        ET.SubElement(assets, "mesh", name=Path(filename).stem, file=prefix + filename)
    for e in so.findall("asset/material"):
        assets.append(deepcopy(e))
    for e in so.findall("actuator/*"):
        acts.append(deepcopy(e))
    ET.SubElement(
        duck.find("worldbody"),
        "geom",
        name="floor",
        type="plane",
        size="1 1 .01",
        rgba=".77 .8 .77 1",
    )
    add_keyframes(duck)
    ET.indent(duck)
    (HERE / "robot.xml").write_text(ET.tostring(duck, encoding="unicode") + "\n")


if __name__ == "__main__":
    build()
