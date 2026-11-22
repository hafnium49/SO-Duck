"""Export the complete assembled model and connector detail as an offline HTML file."""

import json
from itertools import product
import numpy as np
from scipy.spatial import ConvexHull
from model import Robot, ARM_BODIES, SHEETS, HERE, screen, pose_test
from verify import examples, stored
from pickup import CUBE_SIDE, pickup, cube_frames


def simplify(triangles, size):
    points = triangles.reshape(-1, 3)
    grid = np.rint(points / size).astype(int)
    _, index, counts = np.unique(grid, axis=0, return_inverse=True, return_counts=True)
    index = index.ravel()
    vertices = np.column_stack(
        [np.bincount(index, weights=c) / counts for c in points.T]
    )
    faces = index.reshape(-1, 3)
    keep = (
        (faces[:, 0] != faces[:, 1])
        & (faces[:, 1] != faces[:, 2])
        & (faces[:, 0] != faces[:, 2])
    )
    faces = faces[keep]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return np.rint(vertices * 10000).astype(int), faces[first]


def main():
    robot = Robot()
    corners = np.array(list(product((-CUBE_SIDE / 2, CUBE_SIDE / 2), repeat=3)))
    geometry = {**robot.mesh, "cube": corners[ConvexHull(corners).simplices]}
    meshes = []
    for index, (name, mesh) in enumerate(geometry.items()):
        hardware = "_bolt_" in name
        cell = (
            0.00040
            if hardware or name in ("adapter", SHEETS)
            else 0.0009 if name == "so101_base" else 0.0026
        )
        vertices, faces = simplify(mesh, cell)
        color = (
            5
            if hardware
            else (
                3
                if name == "adapter"
                else (
                    4
                    if name == "cube"
                    else 1 if name in ARM_BODIES else 2 if "case" in name else 0
                )
            )
        )
        meshes.append([index, color, vertices.ravel().tolist(), faces.ravel().tolist()])
    poses = []

    def save(mode, label, frames, i, fields, note, tcp=None):
        poses.append(
            dict(
                mode=mode,
                label=label,
                fields=fields,
                note=note,
                tcp=None if tcp is None else tcp[:3].round(8).tolist(),
                c=[
                    frames[n][i, :3].round(8).ravel().tolist() if n in frames else None
                    for n in geometry
                ],
            )
        )

    def motion(mode, labels, rows, note, cube=None):
        f, cm = robot.placements(rows)
        if cube is not None:
            f["cube"] = cube_frames(rows, cube, f)
        for i, label in labels:
            tcp = f["gripper"][i] @ screen.a.TCP
            fields = [
                (
                    "Gripper reference point (x, y, z)",
                    ", ".join(f"{v:.0f}" for v in tcp[:3, 3] * 1000) + " mm",
                ),
                ("Neck pitch", f"{np.rad2deg(rows[i,10]):.1f}°"),
                ("Jaw angle", f"{np.rad2deg(rows[i,-1]):.1f}°"),
                (
                    "Static support margin",
                    f"{1000*pose_test.GROUND.margin(cm[i]):.1f} mm",
                ),
            ]
            save(mode, label, f, i, fields, note, tcp)

    ex = examples()
    whole = {
        "neutral": "Standing",
        "overhead": "Overhead reach",
        "tucked": "Folded arm",
    }
    for name, row in ex.items():
        motion(
            "Whole robot",
            [(0, whole[name])],
            row[None],
            {
                "neutral": "Ten leg actuators, one Duck neck pitch, five SO-101 arm axes and an independent gripper.",
                "overhead": "Both soles remain planted at their nominal pose.",
                "tucked": "A collision-screened folded posture with airborne fingertips. This is an endpoint, without a load-bearing rest.",
            }[name],
        )
    neutral, _ = robot.placements(ex["neutral"])
    keep = {
        n: neutral[n].copy()
        for n in neutral
        if n in ("adapter", SHEETS, "so101_base") or "_bolt_" in n
    }
    info = [
        ("Custom parts", "1 printed adapter"),
        ("Duck interface", "4 × M2 × 12"),
        ("SO-101 interface", "4 × M3 × 30"),
        ("Installed neck pitch", "−20° to +30°"),
    ]
    save(
        "Connector",
        "Assembled",
        keep,
        0,
        info,
        "The two original neck sheets sit outside the adapter cheeks. The original SO-101 base seats on the deck.",
    )
    exploded = {n: fs.copy() for n, fs in keep.items()}
    for n, fs in exploded.items():
        if n == "adapter":
            fs[:, 2, 3] += 0.018
        elif n == "so101_base":
            fs[:, 2, 3] += 0.052
        elif n.startswith("base_bolt_"):
            fs[:, 2, 3] += 0.088
        elif n.startswith("sheet_bolt_"):
            fs[:, 1, 3] += np.sign(fs[0, 1, 3]) * 0.024
    save(
        "Connector",
        "Exploded assembly",
        exploded,
        0,
        info,
        "Parts are separated for visibility. The screws and nuts are shown as cylindrical envelopes.",
    )
    save(
        "Connector",
        "Printed part",
        {"adapter": keep["adapter"]},
        0,
        [
            ("Deck thickness", "5.0 mm"),
            ("Cheek spacing, outside", "31.7 mm"),
            ("Neck-side bores", "4 × Ø2.2 mm"),
            ("Base-side bores", "4 × Ø3.3 mm"),
        ],
        "Orbit below the deck to see the integral cheeks and their asymmetric screw heights.",
    )
    path, grasp, contact = pickup()
    motion(
        "Floor pickup",
        [
            (0, "Approach"),
            (grasp, "Open around cube"),
            (contact, "Both jaws contact"),
            (len(path) - 1, "Lifted 30 mm"),
        ],
        path,
        "20 mm cube at floor level, with both feet planted. After opposing jaw contact, the cube follows the gripper under an assumed no-slip grasp.",
        contact,
    )
    ids, q = stored()
    for mode, mask in [("Forward grasps", ids < 64), ("Broad orientations", ids >= 64)]:
        motion(
            mode,
            [(j, f"Target {i:02}") for j, i in enumerate(ids[mask])],
            q[mask],
            "Verified endpoints on the original target bank. The static support margin uses estimated unloaded masses.",
        )
    points = pose_test.GROUND.sole_points
    sole = points[ConvexHull(points[:, :2]).vertices].copy()
    sole[:, 2] = 0
    data = json.dumps(
        dict(meshes=meshes, poses=poses, sole=sole.round(6).tolist()),
        separators=(",", ":"),
    )
    (HERE / "poses.html").write_text(
        (HERE / "viewer.template.html").read_text().replace("__POSE_DATA__", data)
    )
    print(f"{len(poses)} views, {len(meshes)} mesh groups")


if __name__ == "__main__":
    main()
