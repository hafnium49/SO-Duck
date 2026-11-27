"""Inspect the geometry and poses stored in hollow/viewer.html.

Reports the figures quoted in hollow/README.md from the files in this directory.
The model that produced the first stored poses is not part of this revision, so
every check works from the body transforms and meshes stored in the viewer.

    python hollow/inspect_viewer.py            # needs NumPy
    python hollow/inspect_viewer.py --screen   # also needs python-fcl
    python hollow/inspect_viewer.py --legs     # also needs python-fcl, MuJoCo and restored assets
    python hollow/inspect_viewer.py --replay   # also needs SciPy, MuJoCo and restored assets
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
EXPECTED_FRAMES = {"targets": 96, "pickup": 81, "fold": 230}
# Viewer colour of each part and its print file in parts/.
PART_MESHES = {
    "mount": "neck_socket_mount.stl",
    "forearm": "elbow_to_forearm_roll.stl",
    "wrist": "forearm_roll_to_wrist.stl",
}
# The open review meshes of the first hollow design. Only the earlier reconstruction in model.py uses them.
REVIEW_MESHES = {
    "mount": "neck-socket-mount.preview.stl",
    "forearm": "elbow-to-forearm-roll.preview.stl",
    "wrist": "forearm-roll-to-wrist.preview.stl",
}
FEET = ("foot_assembly", "foot_assembly_2")
SOLE_TOLERANCE = (0.2, 0.15)  # mm and degrees, as in coverage/replay.py
HOLLOW_BODIES = {"shoulder_mount", "forearm_roll_link", "wrist_flex_link"}
# The cheeks of the elbow-to-forearm-roll link seat on the elbow servo, and the cheek of
# the forearm-roll-to-wrist link on the roll servo's horn. The part tests sweep both.
HOLLOW_SEATS = {frozenset(("forearm_pitch_case", "forearm_roll_link")), frozenset(("forearm_roll_case", "wrist_flex_link"))}
# Contacts that come and go with the pose and are meant to: the jaws on the cube, the
# fixed finger on the parking pad, and leg links joined by a joint, whose display meshes
# overlap at the joint.
EXPECTED_CONTACTS = {
    frozenset(pair)
    for pair in [
        ("gripper", "cube"),
        ("moving_jaw_so101_v1", "cube"),
        ("gripper", "park_pad"),
        ("trunk_assembly", "hip_roll_assembly"),
        ("hip_roll_assembly", "left_roll_to_pitch_assembly"),
        ("left_roll_to_pitch_assembly", "knee_and_ankle_assembly"),
        ("knee_and_ankle_assembly", "knee_and_ankle_assembly_2"),
        ("knee_and_ankle_assembly_2", "foot_assembly"),
        ("trunk_assembly", "hip_roll_assembly_2"),
        ("hip_roll_assembly_2", "right_roll_to_pitch_assembly"),
        ("right_roll_to_pitch_assembly", "knee_and_ankle_assembly_3"),
        ("knee_and_ankle_assembly_3", "knee_and_ankle_assembly_4"),
        ("knee_and_ankle_assembly_4", "foot_assembly_2"),
    ]
}
LEGS = {
    "left": ("left_roll_to_pitch_assembly", "knee_and_ankle_assembly", "knee_and_ankle_assembly_2", "foot_assembly"),
    "right": ("right_roll_to_pitch_assembly", "knee_and_ankle_assembly_3", "knee_and_ankle_assembly_4", "foot_assembly_2"),
}
LEG_PARTS = ("hip link", "thigh", "shin", "foot")
SERVO_KG = 0.055
PRINT_DENSITY = 1240.0  # kg/m3, as assumed for the six-part design
# Targets with no posture that passes every check (postures.UNREACHED). Their frames show
# the best posture found and are left out of the checks below.
NO_POSTURE = (64, 74, 78, 89, 93)
# Bodies of robot.xml whose frame the viewer stores under another name.
MODEL_BODY = {
    "upper_mount": "shoulder_mount",
    "shoulder_yaw_linkage": "shoulder_pitch_link",
    "shoulder_pitch_linkage": "forearm_pitch_link",
    "forearm_pitch_linkage": "forearm_roll_link",
    "forearm_roll_linkage": "wrist_flex_link",
    "wrist_flex_linkage": "wrist_roll_link",
}
ARM_JOINTS = ("shoulder yaw", "shoulder pitch", "elbow", "forearm roll", "wrist flex", "wrist roll")


def load_viewer(path=None):
    html = Path(path or HERE / "viewer.html").read_text(encoding="utf-8")
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


def grid_triangles(triangles_mm):
    """Triangles in millimetres on the viewer's 0.1 mm grid, without those the grid flattens."""
    snapped = np.rint(np.asarray(triangles_mm) * 10).astype(np.int64)
    a, b, c = (snapped[:, k].astype(float) for k in range(3))
    return snapped[np.linalg.norm(np.cross(b - a, c - a), axis=1) > 0]


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
    print("Print files against the viewer")
    for group in data["meshes"]:
        vertices, faces = decode_mesh(group)
        if faces.max() >= len(vertices):
            errors.append(f"triangle index out of range in a {group['color']} mesh")
    for color, filename in PART_MESHES.items():
        group = next(g for g in data["meshes"] if g["color"] == color and not g.get("sectionOf"))
        vertices, faces = decode_mesh(group)
        in_viewer = vertices[faces]
        in_stl = grid_triangles(read_stl(HERE / "parts" / filename))
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

    targets = decode_clip(data, "targets")[0]

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


def check_pickup(data, errors):
    """Clearance between each jaw and the cube through the pickup clip."""
    bodies = data["bodies"]
    world = decode_clip(data, "pickup")[1]
    cube = bodies.index("cube")
    half = np.abs(next(decode_mesh(g)[0] for g in data["meshes"] if g["color"] == "cube")).max(0) * 1e-4
    print("Pickup clip, clearance to the 20 mm cube (mm)")
    for label, body in (("fixed jaw", "gripper"), ("moving jaw", "moving_jaw_so101_v1")):
        samples = []
        for group in data["meshes"]:
            if bodies[group["b"]] != body:
                continue
            vertices, faces = decode_mesh(group)
            a, b, c = (vertices[faces[:, k]] * 1e-4 for k in range(3))
            samples += [a, b, c, (a + b) / 2, (b + c) / 2, (c + a) / 2, (a + b + c) / 3]
        points = np.concatenate(samples)
        b = bodies.index(body)
        placed = np.einsum("fij,pj->fpi", world[:, b, :, :3], points) + world[:, b, None, :, 3]
        local = np.einsum("fpi,fij->fpj", placed - world[:, cube, None, :, 3], world[:, cube, :, :3])
        gap = np.linalg.norm(np.maximum(np.abs(local) - half, 0), axis=2).min(1) * 1000
        depth = np.maximum((half - np.abs(local)).min(2).max(1), 0) * 1000
        if depth.max() > 0.05:
            errors.append(f"the {label} overlaps the cube by {depth.max():.2f} mm")
        touching = np.flatnonzero(gap < 0.05)
        first = f"within 0.05 mm from frame {touching[0]}" if len(touching) else "never within 0.05 mm"
        print(
            f"  {label}: {first}, smallest clearance {gap.min():.2f}, "
            f"at frame 50 {gap[50]:.2f}, deepest overlap {depth.max():.2f}"
        )


def screen(data, errors):
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
    total = sum(len(w) for w in clips.values()) - len(NO_POSTURE)
    print(f"Mesh screen: {len(pairs)} pairs of separately moving bodies in {total} stored frames")
    hits = defaultdict(lambda: defaultdict(list))
    for name, world in clips.items():
        for frame in range(len(world)):
            if name == "targets" and frame in NO_POSTURE:
                continue  # these frames are known to fail a check
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
        where = ", ".join(f"{name} {len(f)}/{len(clips[name]) - (len(NO_POSTURE) if name == 'targets' else 0)}" for name, f in found.items())
        kind = "every stored frame" if frames == total else "pose dependent"
        print(f"  {a} x {b}: {kind} ({where})")
        if frozenset((a, b)) in HOLLOW_SEATS:
            continue
        if HOLLOW_BODIES & {a, b}:
            errors.append(f"a hollow part intersects another body: {a} x {b}")
        elif frames != total and frozenset((a, b)) not in EXPECTED_CONTACTS:
            errors.append(f"two bodies intersect in some stored frames: {a} x {b}")


def leg_trunk(data, errors):
    """Intersections of every leg link with the trunk, on the repository's meshes.

    The trunk and legs are unchanged from the six-part design, so their meshes come from
    coverage/robot.xml and are placed with the body transforms stored in the viewer.
    postures.py additionally requires a gap; this test needs only the viewer.
    """
    import fcl
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(HERE.parent / "coverage" / "robot.xml"))
    state = mujoco.MjData(model)
    names = ["trunk_assembly", *LEGS["left"], *LEGS["right"]]

    def collision_object(name):
        body = model.body(name).id
        vertices, faces, offset = [], [], 0
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != body or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            m = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m] + model.mesh_vertnum[m]].astype(float)
            f = model.mesh_face[model.mesh_faceadr[m]:model.mesh_faceadr[m] + model.mesh_facenum[m]].astype(np.int64)
            turn = np.zeros(9)
            mujoco.mju_quat2Mat(turn, model.geom_quat[g])
            vertices.append(v @ turn.reshape(3, 3).T + model.geom_pos[g])
            faces.append(f + offset)
            offset += len(v)
        vertices = np.ascontiguousarray(np.concatenate(vertices))
        faces = np.ascontiguousarray(np.concatenate(faces))
        shape = fcl.BVHModel()
        shape.beginModel(len(vertices), len(faces))
        shape.addSubModel(vertices, faces)
        shape.endModel()
        return fcl.CollisionObject(shape, fcl.Transform())

    objects = {name: collision_object(name) for name in names}
    request = fcl.CollisionRequest()
    found_any = []

    def report(label, frames):
        """frames maps a body name to its (n, 3, 4) world transforms."""
        count = len(frames["trunk_assembly"])
        hit = {}
        for side, leg in LEGS.items():
            for part, name in zip(LEG_PARTS, leg):
                found = []
                for i in range(count):
                    for body in (name, "trunk_assembly"):
                        pose = frames[body][i]
                        objects[body].setTransform(
                            fcl.Transform(np.ascontiguousarray(pose[:, :3]), np.ascontiguousarray(pose[:, 3]))
                        )
                    if fcl.collide(objects[name], objects["trunk_assembly"], request, fcl.CollisionResult()):
                        found.append(i)
                hit[f"{side} {part}"] = found
        thigh = sorted(set(hit["left thigh"]) | set(hit["right thigh"]))
        any_part = sorted(set().union(*hit.values()))
        found_any.append(len(any_part))
        counts = ", ".join(f"{key} {len(found)}" for key, found in hit.items() if found) or "none"
        print(f"  {label}: {counts}")
        print(f"    poses with a thigh in the trunk {len(thigh)} of {count}, with any leg part {len(any_part)} of {count}")
        return thigh

    bodies = data["bodies"]
    print("Leg meshes of the repository against the trunk")
    for name in ("targets", "pickup", "fold"):
        world = decode_clip(data, name)[1]
        report(f"viewer {name}", {b: world[:, bodies.index(b)] for b in names})
        if found_any[-1]:
            errors.append(f"a leg part intersects the trunk in the {name} clip")


def solid(vertices, faces):
    """Signed volume (m3) and centroid (m) of a triangle surface given on the 0.1 mm grid."""
    a, b, c = (vertices[faces[:, k]] * 1e-4 for k in range(3))
    six = np.einsum("ij,ij->i", a, np.cross(b, c))
    return six.sum() / 6, ((a + b + c) / 4 * six[:, None]).sum(0) / six.sum()


def arm_masses(data, world):
    """Estimated arm masses as body name -> [(kg, centre in the body frame)].

    Servos count 55 g at the centroid of their case mesh. The three hollow parts take the
    signed volume of their review surface at the print density of the six-part design. The
    stock SO-101 links take the link inertials of collision/inputs/so101.xml.
    """
    bodies = data["bodies"]

    def mesh(body):
        return next(decode_mesh(g) for g in data["meshes"] if bodies[g["b"]] == body and not g.get("sectionOf"))

    def servo(body):
        return SERVO_KG, solid(*mesh(body))[1]

    def printed(body):
        volume, centre = solid(*mesh(body))
        return volume * PRINT_DENSITY, centre

    def frame(body):
        pose = np.eye(4)
        pose[:3] = world[0, bodies.index(body)]
        return pose

    # The upper-arm frame is the SO-101 one turned about the elbow-parallel axis. The turn is
    # the angle between the two descriptions of the elbow position. The shoulder frame is the
    # SO-101 one turned half a turn about its second axis.
    elbow = (np.linalg.inv(frame("forearm_pitch_link")) @ frame("forearm_roll_link"))[:2, 3]
    turn = np.arctan2(elbow[1], elbow[0]) - np.arctan2(-0.028, -0.11257)
    spin = np.array([[np.cos(turn), -np.sin(turn), 0], [np.sin(turn), np.cos(turn), 0], [0, 0, 1]])
    return {
        "trunk_assembly": [printed("shoulder_mount"), servo("shoulder_yaw_case")],
        "shoulder_pitch_link": [(0.100006, np.array([0.0307604, -1.66727e-05, 0.0252713]))],
        "forearm_pitch_link": [(0.103, spin @ np.array([-0.0898471, -0.00838224, 0.0184089]))],
        "forearm_roll_link": [printed("forearm_roll_link"), servo("forearm_roll_case")],
        "wrist_flex_link": [printed("wrist_flex_link"), servo("wrist_flex_case")],
        "wrist_roll_link": [(0.079, np.array([-0.000103312, -0.0386143, 0.0281156]))],
        "gripper": [(0.087, np.array([0.000213627, 0.000245138, -0.025187]))],
        "moving_jaw_so101_v1": [(0.012, np.array([-0.00157495, -0.0300244, 0.0192755]))],
    }


def model_masses(bodies):
    """Mass and centre of every body of robot.xml, by the viewer body that carries its frame."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(HERE / "robot.xml"))
    table = defaultdict(list)
    for k in range(1, model.nbody):
        if model.body_mass[k] <= 0:
            continue
        centre, body = model.body_ipos[k].copy(), k
        while MODEL_BODY.get(model.body(body).name, model.body(body).name) not in bodies:
            # a body fixed in its parent: carry the centre into the parent's frame
            turn = np.zeros(9)
            mujoco.mju_quat2Mat(turn, model.body_quat[body])
            centre = turn.reshape(3, 3) @ centre + model.body_pos[body]
            body = model.body_parentid[body]
            assert body > 0, model.body(k).name
        name = model.body(body).name
        table[MODEL_BODY.get(name, name)].append((float(model.body_mass[k]), centre))
    return table


def replay(data, errors):
    """The 96 stored endpoints against the published targets, the sole anchors and the support margin.

    Tool and sole poses come straight from the stored body transforms. The support margin uses the
    masses of robot.xml. Needs the stock meshes of cad/build_parts.py --stock.
    """
    sys.path[:0] = [str(HERE.parent / "study")]
    import mujoco
    import pose_test
    from scipy.spatial.transform import Rotation

    bodies = data["bodies"]
    world = decode_clip(data, "targets")[1]

    def placed(body):
        pose = np.tile(np.eye(4), (len(world), 1, 1))
        pose[:, :3] = world[:, bodies.index(body)]
        return pose

    def turn_between(expected, measured):
        return np.degrees(Rotation.from_matrix(expected @ measured.transpose(0, 2, 1)).magnitude())

    print("Stored endpoints against the published targets")
    positions, rotations = pose_test.make_targets()
    tool = placed("gripper") @ pose_test.arm.TCP
    tool_mm = 1000 * np.linalg.norm(tool[:, :3, 3] - positions, axis=1)
    tool_deg = turn_between(rotations, tool[:, :3, :3])
    print(f"  tool: at most {tool_mm.max():.3f} mm and {tool_deg.max():.3f} deg from its target")
    passing = np.setdiff1d(np.arange(len(world)), NO_POSTURE)
    print(f"  tool: at most {tool_mm[passing].max():.3f} mm and {tool_deg[passing].max():.3f} deg without targets {list(NO_POSTURE)}")
    if tool_mm[passing].max() >= 1.5 or tool_deg[passing].max() >= 1.0:
        errors.append("a stored endpoint misses its target by 1.5 mm or 1 deg")
    for k, side in enumerate(pose_test.legs.SIDES):
        sole = placed(LEGS[side][-1]) @ pose_test.legs.specs(side)[4]
        anchor = pose_test.ANCHOR[k]
        sole_mm = 1000 * np.linalg.norm(sole[:, :3, 3] - anchor[:3, 3], axis=1)
        sole_deg = turn_between(anchor[None, :3, :3], sole[:, :3, :3])
        print(f"  {side} sole: at most {sole_mm.max():.3f} mm and {sole_deg.max():.3f} deg from its anchor")
        if sole_mm.max() >= SOLE_TOLERANCE[0] or sole_deg.max() >= SOLE_TOLERANCE[1]:
            errors.append(f"the {side} sole leaves its anchor")

    table = model_masses(bodies)
    total = sum(mass for entries in table.values() for mass, _ in entries)
    moment = np.zeros((len(world), 3))
    for name, entries in table.items():
        pose = world[:, bodies.index(name)]
        for mass, centre in entries:
            moment += mass * (pose[:, :, :3] @ centre + pose[:, :, 3])
    margin = 1000 * pose_test.GROUND.margin(moment / total)
    held = margin[passing] >= 5.0
    print(f"  mass {total:.3f} kg; support margin at least 5 mm in {int(held.sum())} of the {len(passing)} endpoints with a passing posture")
    print(f"    smallest margin {margin[passing].min():.2f} mm (target {int(passing[margin[passing].argmin()]):02d})")
    for i in NO_POSTURE:
        print(f"    target {i:02d}, no passing posture: tool {tool_mm[i]:.1f} mm from its target, margin {margin[i]:.2f} mm")
    if not held.all():
        errors.append(f"support margin under 5 mm at targets {passing[~held].tolist()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--screen", action="store_true", help="also intersect the display meshes (needs python-fcl)")
    parser.add_argument("--legs", action="store_true", help="also intersect the repository's leg and trunk meshes")
    parser.add_argument("--replay", action="store_true", help="also check targets, sole anchors and support margin")
    args = parser.parse_args()
    data = load_viewer()
    errors = []
    check_meshes(data, errors)
    check_poses(data, errors)
    check_pickup(data, errors)
    if args.screen:
        screen(data, errors)
    if args.legs:
        leg_trunk(data, errors)
    if args.replay:
        replay(data, errors)
    if errors:
        print("VIEWER CHECK FAILED\n" + "\n".join("- " + e for e in errors), file=sys.stderr)
        return 1
    print("VIEWER CHECK PASSED for the checks that ran. See hollow/README.md for what they cover.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
