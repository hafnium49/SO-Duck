"""Recheck donor geometry, both bolted interfaces and published posture witnesses."""

from pathlib import Path
import csv
from xml.etree import ElementTree as ET
import cadquery as cq
import mujoco
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from build import HERE, INPUT, JOINTS, SO_JOINTS, BASE_HOLES, SHEET_HOLES, TOP, legs
from model import Robot, pose_test, screen, SHEETS
from pickup import pickup, check_cube


def check_donors(r):
    tree = ET.parse(HERE / "robot.xml")
    so = ET.parse(INPUT / "so101.xml")
    duck = ET.parse(INPUT / "robot_motors.xml")

    def content(el):
        return (el.tag, dict(el.attrib), [content(c) for c in el])

    original = so.find(".//body[@name='base']")
    mounted = tree.find(".//body[@name='so101_base']")
    assert [content(e) for e in original] == [content(e) for e in mounted]
    assert original.get("childclass") == mounted.get("childclass")
    for e in so.findall("actuator/*"):
        assert content(e) == content(tree.find(f"actuator/*[@name='{e.get('name')}']"))
    native = duck.find(".//joint[@name='neck_pitch']")
    installed = tree.find(".//joint[@name='neck_pitch']")
    assert {k: v for k, v in native.attrib.items() if k != "range"} == {
        k: v for k, v in installed.attrib.items() if k != "range"
    }
    assert np.allclose(
        np.rad2deg(np.fromstring(installed.get("range"), sep=" ")),
        [-20, 30],
        atol=0.001,
    )
    for name in r.names[:10]:
        assert content(tree.find(f".//joint[@name='{name}']")) == content(
            duck.find(f".//joint[@name='{name}']")
        )
    assert (
        tree.find(
            ".//body[@name='neck_pitch_assembly']/body[@name='adapter']/body[@name='so101_base']"
        )
        is not None
    )
    assert r.model.nu == 17 and set(r.names) == {
        a.get("joint") for a in tree.findall("actuator/*")
    }
    for name in ("neck_left_sheet", "neck_right_sheet"):
        assert content(tree.find(f".//geom[@mesh='{name}']")) == content(
            duck.find(f".//geom[@mesh='{name}']")
        )
    print(
        "Ten Duck leg joints, original neck pitch and the complete stock SO-101 chain verified"
    )


def check_fit(r):
    shape = cq.importers.importStep(str(HERE / "connector.step")).val()
    mesh = trimesh.load(HERE / "connector.stl")
    assert len(shape.Solids()) == 1 and shape.isValid() and mesh.is_watertight
    assert abs(mesh.volume / shape.Volume() - 1) < 0.002
    q = np.r_[pose_test.REF.ravel(), np.zeros(7)]
    f, _ = r.placements(q)

    # Insert slightly oversized cylindrical gauges into both native interfaces.
    def gauge(label, radius, start, direction, length, targets):
        t = trimesh.creation.cylinder(
            radius=radius / 1000, height=length / 1000, sections=72
        )
        rot, _ = Rotation.align_vectors([direction], [[0.0, 0.0, 1.0]])
        triangles = rot.apply(t.triangles.reshape(-1, 3).copy()).reshape(-1, 3, 3)
        triangles += (np.array(start) + np.array(direction) * length / 2) / 1000
        vv = np.ascontiguousarray(1000 * triangles)
        r.handles[label] = r.lib.load(vv.ctypes.data, len(vv))
        transforms = {**f, label: f["adapter"]}
        for target in targets:
            assert not r.hits(label, target, transforms).any(), (label, target)

    for i, (x, y) in enumerate(BASE_HOLES):
        gauge(f"arm_gauge_{i}", 1.75, (x, y, TOP - 0.3), (0, 0, 1), 26, ["so101_base"])
        gauge(f"deck_gauge_{i}", 1.52, (x, y, TOP - 6), (0, 0, 1), 8, ["adapter"])
    for i, (x, y, z) in enumerate(SHEET_HOLES):
        n = np.sign(y)
        gauge(f"sheet_gauge_{i}", 1.14, (x, n * 14, z), (0, n, 0), 13, [SHEETS])
        gauge(f"lug_gauge_{i}", 1.02, (x, n * 7, z), (0, n, 0), 11, ["adapter"])
    # Measure flat top-face bearing under each 12 mm fender washer, using the
    # donor's tessellated flange. Also measure lower rails seating on the adapter.
    base = f["so101_base"][0]
    adapter = f["adapter"][0]
    tri = 1000 * screen.move(np.linalg.inv(adapter) @ base, r.mesh["so101_base"])
    z = TOP + 17.5
    faces = tri[np.abs(tri[:, :, 2] - z).max(1) < 0.006]
    top = unary_union([Polygon(t[:, :2]) for t in faces])
    bearing = [
        top.intersection(
            Point(x, y).buffer(6, quad_segs=90).difference(Point(x, y).buffer(1.65))
        ).area
        for x, y in BASE_HOLES
    ]
    assert min(bearing) > 78, bearing
    rails = tri[np.abs(tri[:, :, 2] - TOP).max(1) < 0.006]
    contact = unary_union([Polygon(t[:, :2]) for t in rails])
    deck_faces = mesh.triangles[np.abs(mesh.triangles[:, :, 2] - TOP).max(1) < 0.006]
    deck = unary_union([Polygon(t[:, :2]) for t in deck_faces])
    contact_area = contact.intersection(deck).area
    assert contact_area > 400 and tri[:, :, 2].min() >= TOP - 0.002
    print(
        f"One printable solid, {shape.Volume()*1.24e-3:.2f} g at 1.24 g/cm³; flat flange bearing {np.round(bearing,1)} mm², base/deck contact {contact_area:.1f} mm²"
    )


def stored():
    with (HERE / "targets.csv").open() as src:
        rows = list(csv.DictReader(src))
    assert [int(r["target"]) for r in rows] == list(range(96))
    ids = np.array([i for i, r in enumerate(rows) if int(r["found"])])
    q = np.array(
        [
            [float(v) for k, v in rows[i].items() if k.endswith("_rad")] + [0.0]
            for i in ids
        ]
    )
    return ids, q


def examples():
    with (HERE / "examples.csv").open() as src:
        return {
            r["name"]: np.array([float(v) for k, v in r.items() if k.endswith("_rad")])
            for r in csv.DictReader(src)
        }


def foot_error(r, f):
    for i, s in enumerate(legs.SIDES):
        name = r.model.body(r.model.jnt_bodyid[r.model.joint(s + "_ankle").id]).name
        sole = f[name] @ legs.specs(s)[4]
        t = pose_test.ANCHOR[i]
        assert np.linalg.norm(sole[:, :3, 3] - t[:3, 3], axis=1).max() < 0.0002
        angle = Rotation.from_matrix(
            t[:3, :3] @ sole[:, :3, :3].transpose(0, 2, 1)
        ).magnitude()
        assert np.rad2deg(angle).max() < 0.15


def physical(r, q, reserve=0):
    limits = np.stack([r.model.joint(n).range for n in r.names])
    margin = np.deg2rad(np.r_[np.full(16, reserve), 0])
    assert np.all(q >= limits[:, 0] + margin - 2e-7) and np.all(
        q <= limits[:, 1] - margin + 2e-7
    )
    f, cm, floor, hits = r.check(q)
    assert not hits.any() and cm.min() >= 0.005 and floor.min() >= -0.0001, (
        np.flatnonzero(hits),
        cm.min(),
        floor.min(),
    )
    foot_error(r, f)
    return f, cm


def replay():
    r = Robot()
    check_donors(r)
    check_fit(r)
    # Sweep the fixed neck/base interface through the installed range and out
    # to the original upper stop. All these pairs move only with neck pitch.
    angles = np.r_[np.linspace(-20, 30, 101), 31.0]
    sweep = np.tile(np.r_[pose_test.REF.ravel(), np.zeros(7)], (len(angles), 1))
    sweep[:, 10] = np.deg2rad(angles)
    frames, _ = r.placements(sweep)
    for n in ["so101_base", "adapter", *[k for k in r.mesh if "_bolt_" in k]]:
        hits = r.hits(n, "trunk_assembly", frames)
        assert not hits[:-1].any(), n
        if n == "so101_base":
            assert hits[-1]
    print(
        "Installed neck range -20° to +30° clear of trunk at 0.5° intervals; stock base contacts trunk at +31°"
    )
    ids, q = stored()
    f, margin = physical(r, q, 2)
    pos, rot = pose_test.make_targets()
    tcp = f["gripper"] @ screen.a.TCP
    err = 1000 * np.linalg.norm(tcp[:, :3, 3] - pos[ids], axis=1)
    angle = np.rad2deg(
        Rotation.from_matrix(rot[ids] @ tcp[:, :3, :3].transpose(0, 2, 1)).magnitude()
    )
    assert err.max() < 1.5 and angle.max() < 1.0
    assert (ids < 64).sum() >= 52 and (ids >= 64).sum() >= 7
    print(
        f"Grasp targets: {(ids<64).sum()}/64 forward, {(ids>=64).sum()}/32 broad. Worst residuals {err.max():.3f} mm, {angle.max():.3f}°. Minimum static support margin {margin.min()*1000:.2f} mm"
    )
    # Static gravity torque for the driven neck at the stored target endpoints.
    torque = []
    for row in q:
        r.placements(row)
        mujoco.mj_forward(r.model, r.data)
        torque.append(abs(r.data.qfrc_bias[r.model.joint("neck_pitch").dofadr[0]]))
    print(f"Peak unloaded neck gravity torque on these targets {max(torque):.3f} N·m")
    ex = examples()
    for name, row in ex.items():
        ff, cm = physical(r, row[None])
        tcp = (ff["gripper"] @ screen.a.TCP)[0, :3, 3]
        if name == "overhead":
            assert np.linalg.norm(tcp - [0.2, 0, 0.6506]) < 0.0005
        high = max(
            (p @ ff[n][0, 2, :3] + ff[n][0, 2, 3]).max() for n, p in r.extreme.items()
        )
        print(f"{name}: height {high*1000:.1f} mm; TCP mm {np.round(1000*tcp,1)}")
    path, grasp, contact = pickup()
    frames, _ = physical(r, path)
    check_cube(r, path, grasp, contact, frames)


if __name__ == "__main__":
    replay()
