"""Write the three printable parts, the stock meshes and the model.

    python hollow/cad/build_parts.py           # write hollow/parts/ and hollow/robot.xml
    python hollow/cad/build_parts.py --stock   # write only hollow/parts/stock/, the meshes the model needs besides the three parts
    python hollow/cad/build_parts.py --check   # exit 1 unless the committed STEP files are what parts.py draws
    python hollow/cad/build_parts.py --orientations   # overhang area of each part for the six ways to lay it on the bed

hollow/parts/stock/ holds the stock SO-101 links and the servos as the model places them.
It is derived from cad/donors/ and is not committed.

Run from the repository root. STEP and STL are in millimetres, in the frame of the model
body that carries the part. A STEP file carries the time it was written, so --check
compares volume and bounding box, not bytes.
"""

import argparse
import sys
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent), str(HERE.parents[1] / "coverage"), str(HERE.parents[1] / "crosswise")]

import assemble  # noqa: E402
import params  # noqa: E402
import parts  # noqa: E402
import stock  # noqa: E402

OUT = HERE.parent / "parts"
# Chord and angle tolerance of the written meshes: millimetres, radians.
MESH_TOLERANCE = (0.05, 0.2)
# Geom name of each custom part: the function that draws it, its model body, its servo.
CUSTOM = {
    "shoulder_mount": (parts.neck_mount, "upper_mount", "shoulder_yaw_case"),
    "forearm_roll_link": (parts.forearm_link, "forearm_pitch_linkage", "forearm_roll_case"),
    "wrist_flex_link": (parts.wrist_link, "forearm_roll_linkage", "wrist_flex_case"),
}


def servo_frames():
    """Each servo in its body's frame, with the two that move with the chosen lengths."""
    frames = {name: stock.placement(name) for name in stock.SERVOS}
    frames["forearm_roll_case"] = parts.roll_servo_frame()
    frames["wrist_flex_case"] = parts.wrist_servo_frame()
    return frames


def measure(solid):
    box = solid.BoundingBox()
    return solid.Volume(), np.array([box.xmin, box.ymin, box.zmin, box.xmax, box.ymax, box.zmax])


def _same(drawn, stored):
    return abs(drawn[0] - stored[0]) < 1.0 and np.abs(drawn[1] - stored[1]).max() < 0.01  # 0.001 cm3, 0.01 mm


def _holds(step, solid):
    """Whether a STEP file holds this solid: same volume and box, and surfaces within 0.05 mm, as check() asks."""
    stored = cq.importers.importStep(str(step)).val()
    if not _same(measure(solid), measure(stored)):
        return False
    return surface_gap_mm(stock.tessellate(solid, 0.01, 0.1), stock.tessellate(stored, 0.01, 0.1)) < 0.05


def masses(solids):
    """Body name to (kg, centre m): solid volume at the print density, 55 g per servo, stock link inertials."""
    frames = servo_frames()
    servo_centre = stock.servo_mesh().center_mass
    out = {}
    for body, entries in assemble.PRESENT_MASSES.items():
        entries = [(mass, np.array(centre)) for mass, centre in entries]
        for geom, (_, carrier, servo) in CUSTOM.items():
            if carrier == body:
                solid = solids[geom]
                centre = cq.Shape.centerOfMass(solid)
                entries = [
                    (solid.Volume() * 1e-9 * params.DENSITY, np.array([centre.x, centre.y, centre.z]) / 1000),
                    (params.SERVO_KG, (frames[servo] @ np.r_[servo_centre, 1.0])[:3] / 1000),
                ]
        total = sum(mass for mass, _ in entries)
        out[body] = (total, sum(mass * centre for mass, centre in entries) / total)
    return out


def write_stock(out=OUT):
    """The stock links and the servos, in their body frames, as the model's meshes."""
    (out / "stock").mkdir(parents=True, exist_ok=True)
    for name, donor in stock.STOCK.items():
        link = stock.tessellate(params.donor(donor), *MESH_TOLERANCE)
        link.apply_transform(stock.placement(name))
        link.export(out / "stock" / f"{name}.stl")
    for name, frame in servo_frames().items():
        servo = stock.servo_mesh(name != "wrist_roll_case").copy()
        servo.apply_transform(frame)
        servo.export(out / "stock" / f"{name}.stl")


def write(out=OUT, model=True):
    solids = {geom: draw() for geom, (draw, _, _) in CUSTOM.items()}
    out.mkdir(parents=True, exist_ok=True)
    for geom, solid in solids.items():
        name = assemble.CUSTOM[geom]
        step = out / f"{name}.step"
        # A STEP file carries the time it was written. One that already holds this solid is kept.
        if not (step.exists() and _holds(step, solid)):
            cq.exporters.export(cq.Workplane("XY").add(solid), str(step))
        mesh = stock.tessellate(solid, *MESH_TOLERANCE)
        if not mesh.is_watertight:
            raise ValueError(f"{name}: the mesh is not closed")
        mesh.export(out / f"{name}.stl")
    write_stock(out)
    if model:
        frames = params.joint_frames(params.FOREARM_SHORTEN, params.WRIST_SIDE_SHORTEN, params.WRIST_AXIAL_SHORTEN)
        assemble.write_model(HERE.parent / "robot.xml", frames, masses(solids), assemble.built_meshes(), params.RANGES_DEG, neck_servo=False)
    return solids


UP = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}


def orientations(name):
    """For each body axis pointing up: overhang area steeper than 45 degrees, area on the bed (cm2), print height (mm)."""
    import trimesh

    mesh = trimesh.load(OUT / f"{name}.stl")
    out = {}
    for label, up in UP.items():
        up = np.array(up, dtype=float)
        height = mesh.triangles_center @ up
        down = mesh.face_normals @ -up > np.cos(np.radians(45))
        bed = down & (height < height.min() + 0.3)
        out[label] = (float(mesh.area_faces[down & ~bed].sum() / 100), float(mesh.area_faces[bed].sum() / 100), float(np.ptp(mesh.vertices @ up)))
    return out


def surface_gap_mm(a, b, samples=20000):
    """The largest distance from sample points on either mesh to the other mesh's surface."""
    import trimesh

    gap = 0.0
    for one, other in ((a, b), (b, a)):
        points = np.asarray(trimesh.sample.sample_surface(one, samples, seed=0)[0])
        gap = max(gap, float(trimesh.proximity.closest_point(other, points)[1].max()))
    return gap


def check():
    """Whether the committed STEP and STL of each part are what parts.py draws.

    Volume and bounding box alone would miss a hole moved by a millimetre, so the surfaces
    are compared as well: no point of one may lie further than 0.05 mm from the other.
    """
    import trimesh

    good = True
    for geom, (draw, _, _) in CUSTOM.items():
        name = assemble.CUSTOM[geom]
        solid = draw()
        step = cq.importers.importStep(str(OUT / f"{name}.step")).val()
        sizes = measure(solid), measure(step)  # before any mesh exists: a meshed shape reports the mesh's box
        # The STEP file is compared on fine meshes; the STL is the drawn solid at the written tolerance.
        gaps = (
            surface_gap_mm(stock.tessellate(solid, 0.01, 0.1), stock.tessellate(step, 0.01, 0.1)),
            surface_gap_mm(stock.tessellate(solid, *MESH_TOLERANCE), trimesh.load(OUT / f"{name}.stl")),
        )
        same = _same(*sizes) and max(gaps) < 0.05
        print(f"{name}: {sizes[0][0] / 1000:.3f} cm3 drawn, {sizes[1][0] / 1000:.3f} cm3 stored, surfaces within {gaps[0]:.3f} mm (STEP) and {gaps[1]:.3f} mm (STL): {'same' if same else 'DIFFERENT'}")
        good &= same
    return good


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--stock", action="store_true")
    parser.add_argument("--orientations", action="store_true")
    args = parser.parse_args()
    if args.check:
        sys.exit(0 if check() else 1)
    if args.orientations:
        for name in assemble.CUSTOM.values():
            print(name)
            for label, (overhang, bed, height) in sorted(orientations(name).items(), key=lambda item: item[1][0]):
                print(f"  {label} up: {overhang:.1f} cm2 of overhang, {bed:.1f} cm2 on the bed, {height:.0f} mm tall")
        return
    if args.stock:
        write_stock()
        print("Wrote hollow/parts/stock/")
        return
    solids = write()
    for geom, solid in solids.items():
        box = solid.BoundingBox()
        print(f"{assemble.CUSTOM[geom]}: {solid.Volume() * 1e-3 * params.DENSITY / 1000:.1f} g, {box.xlen:.0f} x {box.ylen:.0f} x {box.zlen:.0f} mm")
    print("Wrote hollow/parts/ and hollow/robot.xml")


if __name__ == "__main__":
    main()
