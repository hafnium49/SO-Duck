"""Inspect the geometry and poses stored in hollow/viewer.html.

Reports the figures quoted in hollow/README.md from the files in this directory.
This is not a replay of the design: the model that produced the stored poses is
not part of this revision, so balance, joint limits and target errors are not
checked here.

    python hollow/inspect_viewer.py            # needs NumPy
    python hollow/inspect_viewer.py --screen   # also needs python-fcl
"""
import argparse
import base64
import gzip
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
EXPECTED_FRAMES = {"targets": 96, "pickup": 81, "fold": 188}
REVIEW_MESHES = {
    "mount": "neck-socket-mount.preview.stl",
    "forearm": "elbow-to-forearm-roll.preview.stl",
    "wrist": "forearm-roll-to-wrist.preview.stl",
}
FEET = ("foot_assembly", "foot_assembly_2")
SOLE_TOLERANCE = (0.2, 0.15)  # mm and degrees, as in coverage/replay.py
ARM_JOINTS = ("shoulder yaw", "shoulder pitch", "elbow", "forearm roll", "wrist flex", "wrist roll")


def load_viewer():
    html = (HERE / "viewer.html").read_text(encoding="utf-8")
    found = re.search(r'<script id="payload"[^>]*>(.*?)</script>', html, re.S)
    if not found:
        raise SystemExit("viewer.html has no payload block")
    return json.loads(gzip.decompress(base64.b64decode(found.group(1).strip())))


def decode_mesh(group):
    """Vertices on the viewer's 0.1 mm grid (integers) and triangle indices."""
    vertices = np.frombuffer(base64.b64decode(group["v"]), dtype="<i2").reshape(-1, 3)
    faces = np.frombuffer(base64.b64decode(group["f"]), dtype="<u2").reshape(-1, 3)
    return vertices.astype(np.int64), faces.astype(np.int64)


def decode_clip(data, name):
    clip = next(p for p in data["poses"] if p["name"] == name)
    world = np.frombuffer(base64.b64decode(clip["world"]), dtype="<f4")
    return clip, world.reshape(clip["n"], len(data["bodies"]), 3, 4).astype(float)


def read_stl(path):
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + 50 * count:
        raise SystemExit(f"{path.name}: not a binary STL of {count} triangles")
    record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(raw, dtype=record, offset=84, count=count)["v"].astype(float)


def surface_defects(triangles):
    """Zero-area triangles and open edges of a triangle soup on the integer grid."""
    points, index = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = index.reshape(-1, 3)
    a, b, c = (points[faces[:, k]].astype(float) for k in range(3))
    flat = np.linalg.norm(np.cross(b - a, c - a), axis=1) == 0
    kept = faces[~flat]
    edges = np.sort(np.concatenate([kept[:, [0, 1]], kept[:, [1, 2]], kept[:, [2, 0]]]), axis=1)
    _, uses = np.unique(edges, axis=0, return_counts=True)
    return int(flat.sum()), int((uses == 1).sum())


def check_meshes(data, errors):
    print("Review meshes against the viewer")
    for group in data["meshes"]:
        vertices, faces = decode_mesh(group)
        if faces.max() >= len(vertices):
            errors.append(f"triangle index out of range in a {group['color']} mesh")
    for color, filename in REVIEW_MESHES.items():
        group = next(g for g in data["meshes"] if g["color"] == color and not g.get("sectionOf"))
        vertices, faces = decode_mesh(group)
        in_viewer = vertices[faces]
        in_stl = np.rint(read_stl(HERE / "review-meshes" / filename) * 10).astype(np.int64)
        same = in_stl.shape == in_viewer.shape and np.array_equal(in_stl, in_viewer)
        if not same:
            errors.append(f"{filename} differs from the viewer mesh")
        flat, open_edges = surface_defects(in_viewer)
        print(
            f"  {filename}: {len(in_viewer)} triangles, identical to the viewer: {same}, "
            f"zero-area triangles {flat}, open edges {open_edges}"
        )


def check_poses(data, errors):
    bodies = data["bodies"]
    print("Stored poses")
    for name, expected in EXPECTED_FRAMES.items():
        clip, world = decode_clip(data, name)
        print(f"  {name}: {clip['n']} frames")
        if clip["n"] != expected:
            errors.append(f"{name} holds {clip['n']} frames, expected {expected}")
        rotation = world[..., :3]
        skew = np.abs(rotation @ rotation.transpose(0, 1, 3, 2) - np.eye(3)).max()
        if skew > 1e-5:
            errors.append(f"{name} holds a transform that is not a rotation")

    targets, world = decode_clip(data, "targets")
    _, stance = decode_clip(data, "pickup")
    print("Sole closure of the 96 endpoints against the stance of the pickup clip")
    for foot in FEET:
        b = bodies.index(foot)
        reference = stance[0, b]
        shift = np.linalg.norm(world[:, b, :, 3] - reference[:, 3], axis=1) * 1000
        relative = reference[:, :3].T @ world[:, b, :, :3]
        cosine = (np.trace(relative, axis1=1, axis2=2) - 1) / 2
        turn = np.degrees(np.arccos(np.clip(cosine, -1, 1)))
        print(
            f"  {foot}: at most {shift.max():.3f} mm (target {shift.argmax():02d}) "
            f"and {turn.max():.3f} deg (target {turn.argmax():02d})"
        )
        if shift.max() >= SOLE_TOLERANCE[0] or turn.max() >= SOLE_TOLERANCE[1]:
            errors.append(f"{foot} leaves the {SOLE_TOLERANCE[0]} mm / {SOLE_TOLERANCE[1]} deg tolerance")

    angles = np.array(targets["q"])[:, :6]
    low, high = angles.min(0), angles.max(0)
    at_extreme = (np.abs(angles - low) < 0.05) | (np.abs(angles - high) < 0.05)
    print("Arm joint angles of the 96 endpoints (deg)")
    for k, joint in enumerate(ARM_JOINTS):
        print(f"  {joint}: {low[k]:.1f} to {high[k]:.1f}, {at_extreme[:, k].sum()} endpoints at an extreme")
    print(f"  endpoints with at least one arm joint at an extreme: {at_extreme.any(1).sum()} of {len(angles)}")

    shapes = [(g["b"], decode_mesh(g)[0] * 1e-4) for g in data["meshes"] if not g.get("sectionOf") and g["color"] != "cube"]
    feet = {bodies.index(f) for f in FEET}
    print("Heights above the floor (mm)")
    for name in ("targets", "pickup", "fold", "overhead", "park"):
        clip, world = decode_clip(data, name)
        top = np.full(clip["n"], -np.inf)
        low_point = np.full(clip["n"], np.inf)
        for b, vertices in shapes:
            z = np.einsum("vk,fk->fv", vertices, world[:, b, 2, :3]) + world[:, b, 2, 3:4]
            top = np.maximum(top, z.max(1))
            if b not in feet:
                low_point = np.minimum(low_point, z.min(1))
        print(
            f"  {name}: tallest frame {1000 * top.max():.1f}, last frame {1000 * top[-1]:.1f}, "
            f"lowest geometry other than the feet {1000 * low_point.min():.2f}"
        )


def screen(data):
    """Triangle intersections between separately moving bodies, on the display meshes."""
    import fcl

    bodies = data["bodies"]
    parts = defaultdict(list)
    for group in data["meshes"]:
        if not group.get("sectionOf"):
            parts[bodies[group["b"]]].append(decode_mesh(group))
    objects = {}
    for body, groups in parts.items():
        offsets = np.cumsum([0] + [len(v) for v, _ in groups[:-1]])
        vertices = np.ascontiguousarray(np.concatenate([v for v, _ in groups]) * 1e-4)
        faces = np.ascontiguousarray(np.concatenate([f + o for (_, f), o in zip(groups, offsets)]))
        model = fcl.BVHModel()
        model.beginModel(len(vertices), len(faces))
        model.addSubModel(vertices, faces)
        model.endModel()
        objects[body] = fcl.CollisionObject(model, fcl.Transform())

    clips = {p["name"]: decode_clip(data, p["name"])[1] for p in data["poses"]}
    every = np.concatenate(list(clips.values()))
    homogeneous = np.tile(np.eye(4), (len(every), len(bodies), 1, 1))
    homogeneous[:, :, :3] = every
    names = list(objects)

    def rigid(a, b):
        relative = np.linalg.inv(homogeneous[:, bodies.index(a)]) @ homogeneous[:, bodies.index(b)]
        return np.abs(relative - relative[0]).max() < 2e-5

    pairs = [(a, b) for i, a in enumerate(names) for b in names[i + 1:] if not rigid(a, b)]
    request = fcl.CollisionRequest()
    total = sum(len(w) for w in clips.values())
    print(f"Mesh screen: {len(pairs)} pairs of separately moving bodies in {total} stored frames")
    hits = defaultdict(lambda: defaultdict(list))
    for name, world in clips.items():
        for frame in range(len(world)):
            for body, item in objects.items():
                pose = world[frame, bodies.index(body)]
                item.setTransform(fcl.Transform(np.ascontiguousarray(pose[:, :3]), np.ascontiguousarray(pose[:, 3])))
            for a, b in pairs:
                if "cube" in (a, b) and name != "pickup":
                    continue
                if fcl.collide(objects[a], objects[b], request, fcl.CollisionResult()):
                    hits[(a, b)][name].append(frame)
    for (a, b), found in sorted(hits.items()):
        frames = sum(len(f) for f in found.values())
        where = ", ".join(f"{name} {len(f)}/{len(clips[name])}" for name, f in found.items())
        kind = "every stored frame" if frames == total else "pose dependent"
        print(f"  {a} x {b}: {kind} ({where})")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--screen", action="store_true", help="also intersect the display meshes (needs python-fcl)")
    args = parser.parse_args()
    data = load_viewer()
    errors = []
    check_meshes(data, errors)
    check_poses(data, errors)
    if args.screen:
        screen(data)
    if errors:
        print("VIEWER CHECK FAILED\n" + "\n".join("- " + e for e in errors), file=sys.stderr)
        return 1
    print("VIEWER CHECK PASSED: counts, indices, review meshes and sole closure (not a replay of the design)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
