"""Check published target poses against the packaging model and both anchored feet."""

import sys
import csv
from itertools import combinations

import numpy as np
from scipy.spatial.transform import Rotation
from build import HERE, JOINTS

sys.path.insert(0, str(HERE.parent / "crosswise"))
from geometry import Robot, pose_test, screen, FINGER

UPPER = ("upper_mount", *[j + "_linkage" for j in JOINTS[:5]])
PARTS = (
    {j + "_case" for j in JOINTS}
    | {j + "_link" for j in JOINTS[1:]}
    | {"shoulder_mount", "gripper", FINGER}
)


# Required gap between each leg link and the trunk, metres. The stock leg alone brings
# the hip link within 0.02 mm of the trunk at 23 degrees of hip roll, so the hip link
# gets a smaller gap than the links below it.
LEG_LINKS = ("hip link", "thigh", "shin", "foot")
LEG_GAP = np.array([0.0003, 0.001, 0.001, 0.001])
# Targets with no posture that passes every check. resolve.py found none for them.
# Their rows in targets.csv hold the earlier posture, in which a leg link meets the trunk.
UNREACHED = (64,)


def robot_model():
    return Robot(HERE / "robot.xml", JOINTS, UPPER)


def collision_pairs(robot, seats=(), more_parts=()):
    # Native wrist servo to stock gripper, closed donor jaw, and deck/bolted plate.
    # All other adjacent moving arm surfaces are tested, including each pitch joint's two links.
    # A design whose meshes overlap where a link seats on a servo passes those pairs as seats.
    skip = {
        frozenset(pair)
        for pair in [
            ("wrist_roll_case", "gripper"),
            ("gripper", FINGER),
            ("shoulder_mount", "trunk_assembly"),
            *seats,
        ]
    }
    parts = PARTS | set(more_parts)
    for a, b in combinations(robot.mesh, 2):
        if (
            (a in parts or b in parts)
            and robot.body[a] != robot.body[b]
            and frozenset((a, b)) not in skip
        ):
            yield a, b
    legs = leg_bodies(robot)
    yield from ((a, b) for a in legs[0] for b in legs[1])
    for leg in legs:
        yield from ((a, b) for i, a in enumerate(leg) for b in leg[i + 2 :])
        # Every link below the hip-yaw link, which is the trunk's own joint partner.
        yield from ((a, "trunk_assembly") for a in leg[1:])


def leg_bodies(robot):
    """Body names of each leg in joint order, from the hip-yaw link to the foot."""
    return [
        [
            robot.model.body(
                robot.model.jnt_bodyid[robot.model.joint(f"{s}_{n}").id]
            ).name
            for n in screen.legs.NAMES
        ]
        for s in screen.legs.SIDES
    ]


def leg_clearance(robot, frames):
    """Distance in metres from each hip link, thigh, shin and foot to the trunk.

    Returns one row per posture, shaped (postures, 2 legs, 4 links). An intersecting
    link reports a distance of zero or less.
    """
    import fcl

    if not hasattr(robot, "solids"):
        robot.solids = {}

    def placed(name, i):
        if name not in robot.solids:
            points = np.ascontiguousarray(robot.mesh[name].reshape(-1, 3))
            faces = np.arange(len(points)).reshape(-1, 3)
            shape = fcl.BVHModel()
            shape.beginModel(len(points), len(faces))
            shape.addSubModel(points, faces)
            shape.endModel()
            robot.solids[name] = fcl.CollisionObject(shape, fcl.Transform())
        pose = frames[name][i]
        robot.solids[name].setTransform(
            fcl.Transform(
                np.ascontiguousarray(pose[:3, :3]), np.ascontiguousarray(pose[:3, 3])
            )
        )
        return robot.solids[name]

    links = [leg[1:] for leg in leg_bodies(robot)]
    gap = np.empty((len(frames["trunk_assembly"]), 2, 4))
    for i in range(len(gap)):
        trunk = placed("trunk_assembly", i)
        for s, leg in enumerate(links):
            for k, name in enumerate(leg):
                gap[i, s, k] = fcl.distance(
                    placed(name, i), trunk, fcl.DistanceRequest(), fcl.DistanceResult()
                )
    return gap


def report(robot, q):
    f, c = robot.placements(np.c_[np.rad2deg(q[:, 10:]), np.zeros(len(q))], q[:, :10])
    collision = np.zeros(len(q), bool)
    for a, b in collision_pairs(robot):
        collision |= robot.hits(a, b, f)
    floor = np.min(
        [robot.lowest(n, f) for n in robot.mesh if not n.startswith("foot_assembly")],
        axis=0,
    )
    return f, pose_test.GROUND.margin(c), floor, collision


def read_targets():
    with (HERE / "targets.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    tags = [f"{s}_{j}" for s in screen.legs.SIDES for j in screen.legs.NAMES] + list(
        JOINTS
    )
    q = np.array([[float(row[j + "_rad"]) for j in tags] for row in rows])
    assert [int(r["target"]) for r in rows] == list(range(96)) and np.isfinite(q).all()
    return tags, q


def verify(robot):
    tags, q = read_targets()
    assert robot.model.nu == 17
    ranges = np.array([robot.model.joint(j).range for j in tags])
    assert np.all(q >= ranges[:, 0] + np.deg2rad(2) - 1e-7) and np.all(
        q <= ranges[:, 1] - np.deg2rad(2) + 1e-7
    )
    frames, margin, floor, collision = report(robot, q)
    tcp = frames["gripper"] @ screen.a.TCP
    positions, rotations = pose_test.make_targets()
    anchors = pose_test.ANCHOR

    def error(actual, position, rotation):
        mm = 1000 * np.linalg.norm(actual[:, :3, 3] - position, axis=1)
        degrees = np.rad2deg(
            Rotation.from_matrix(
                rotation @ actual[:, :3, :3].transpose(0, 2, 1)
            ).magnitude()
        )
        return mm, degrees

    mm, deg = error(tcp, positions, rotations)
    gap = leg_clearance(robot, frames)
    ok = (mm < 1.5) & (deg < 1) & (margin >= 0.005) & (floor >= -0.0001) & ~collision
    ok &= (gap >= LEG_GAP).all(axis=(1, 2))
    print(f"Maximum TCP error {mm.max():.3f} mm and {deg.max():.3f} deg")
    for k, side in enumerate(screen.legs.SIDES):
        foot_id = robot.model.jnt_bodyid[robot.model.joint(side + "_ankle").id]
        foot = frames[robot.model.body(foot_id).name] @ screen.legs.specs(side)[4]
        distance, angle = error(foot, anchors[k, :3, 3], anchors[k, :3, :3])
        ok &= (distance < 0.2) & (angle < 0.15)
        print(f"{side} foot max error {distance.max():.3f} mm, {angle.max():.3f} deg")
    print(
        f"Minimum support margin {margin[ok].min()*1000:.2f} mm, floor gap {floor[ok].min()*1000:.2f} mm"
    )
    closest = 1000 * gap[ok].min(axis=(0, 1))
    print(
        "Smallest gap to the trunk: "
        + ", ".join(f"{name} {value:.2f} mm" for name, value in zip(LEG_LINKS, closest))
    )
    print(
        f"Front {ok[:64].sum()}/64. Broad {ok[64:].sum()}/32. Failed IDs {np.flatnonzero(~ok).tolist()}"
    )
    assert np.flatnonzero(~ok).tolist() == list(UNREACHED)
    q = q[ok]

    # The moving finger is the only mesh that changes with aperture. The jaw
    # changes whole-body COM too, so repeat the balance check at each angle.
    finger_pairs = [(a, b) for a, b in collision_pairs(robot) if FINGER in (a, b)]
    for opening in range(2, 21, 2):
        arm = np.c_[np.rad2deg(q[:, 10:]), np.full(len(q), opening)]
        sweep, com = robot.placements(arm, q[:, :10])
        for a, b in finger_pairs:
            hits = robot.hits(a, b, sweep)
            assert not hits.any(), (opening, a, b, np.flatnonzero(hits))
        assert robot.lowest(FINGER, sweep).min() >= -0.0001
        assert pose_test.GROUND.margin(com).min() >= 0.005
    print(f"All {len(q)} passing targets clear at 11 sampled jaw angles from 0 to 20 degrees")


def read_arm(name):
    return np.atleast_2d(np.loadtxt(HERE / (name + ".csv"), delimiter=",", skiprows=1))


def pickup():
    from motion import JAW_OPEN, JAW_CONTACT

    ik = np.rad2deg(read_arm("floor"))
    n = len(ik)
    descent = np.column_stack([ik[::-1], np.full(n, JAW_OPEN)])
    closure = np.column_stack(
        [np.tile(ik[0], (20, 1)), np.linspace(JAW_OPEN, JAW_CONTACT, 21)[1:]]
    )
    lift = np.column_stack([ik[1:], np.full(n - 1, JAW_CONTACT)])
    return np.vstack([descent, closure, lift]), n - 1, n + len(closure) - 1


def fixed_stance(robot, joints, seated=False):
    frames, com = robot.placements(joints)
    limits = np.stack([robot.model.joint(name).range for name in (*JOINTS, "gripper")])
    assert np.all(np.deg2rad(joints) >= limits[:, 0] - 1e-7)
    assert np.all(np.deg2rad(joints) <= limits[:, 1] + 1e-7)
    for a, b in collision_pairs(robot):
        hits = robot.hits(a, b, frames)
        if seated and {a, b} == {"gripper", "park_pad"}:
            assert (
                hits[-1] and not hits[:-1].any()
            ), "Pad contact only on the final sample"
        else:
            assert not hits.any(), (a, b, np.flatnonzero(hits))
    for n in robot.mesh:
        low = robot.lowest(n, frames)
        if n.startswith("foot_assembly"):
            assert np.abs(low).max() < 0.00011
        else:
            assert low.min() > -0.00001, (n, low.min())
    margin = pose_test.GROUND.margin(com)
    assert margin.min() >= 0.005
    # The legs do not move in this stance, so one posture gives the gap for all.
    first = {name: value[:1] for name, value in frames.items()}
    assert (leg_clearance(robot, first) >= LEG_GAP).all()
    return frames


def verify_mounts(robot):
    import cadquery as cq
    import xml.etree.ElementTree as ET

    # Use the STEP assembly's common torso datum to verify each STL's local
    # coordinate frame and the mass recorded in the assembled MJCF.
    solids = cq.importers.importStep(str(HERE / "mounts.step")).val().Solids()
    assert len(solids) == 6 and all(s.isValid() for s in solids)
    root = ET.parse(HERE / "robot.xml")
    frames, _ = robot.placements(np.zeros((1, 7)))
    for part in ["shoulder_mount", *[j + "_link" for j in JOINTS[1:]]]:
        world = np.linalg.inv(pose_test.W0) @ frames[part][0]
        pts = 1000 * screen.move(world, robot.extreme[part])
        bounds = np.r_[pts.min(0), pts.max(0)]
        matched = []
        for solid in solids:
            b = solid.BoundingBox()
            if np.allclose(
                bounds, [b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax], atol=0.15
            ):
                matched.append(solid)
        assert len(matched) == 1, part
        shape = matched[0]
        mass = float(root.find(f".//geom[@name='{part}']").get("mass"))
        assert abs(shape.Volume() * 1.24e-6 - mass) < 1e-7, part
        solids.remove(shape)
    print("Six valid STEP solids match the assembled STL bounds and link masses")


def fold_path():
    from motion import between

    high, via, parked = [
        np.r_[np.rad2deg(read_arm(name)[0]), 0.0]
        for name in ("high", "fold_waypoint", "park")
    ]
    return np.vstack([between(high, via)[:-1], between(via, parked)])


def verify_secondary(robot):
    from motion import ROOF_MM, PAD_HEIGHT_MM
    from verify import check_cube

    path, grasp, contact = pickup()
    frames = fixed_stance(robot, path)
    check_cube(robot, path, grasp, contact, frames)

    fold = fold_path()
    folded = fixed_stance(robot, fold, seated=True)
    frames = {name: values[-1:] for name, values in folded.items()}
    print(
        f"Overhead to parked transition clear in {len(fold)} samples at ≤0.5 degrees per joint"
    )
    top = pose_test.W0[2, 3] + 0.001 * (ROOF_MM + PAD_HEIGHT_MM)
    indentation = 1000 * (top - robot.lowest("gripper", frames)[0])
    jaw_gap = 1000 * (robot.lowest(FINGER, frames)[0] - top)
    assert 0 < indentation < 1 and jaw_gap > 0
    height = max(
        (points @ frames[k][0, 2, :3] + frames[k][0, 2, 3]).max()
        for k, points in robot.extreme.items()
    )
    print(
        f"Parked height {1000*height:.1f} mm, fixed-finger indentation {indentation:.2f} mm, jaw clearance over pad plane {jaw_gap:.2f} mm"
    )
    frames = fixed_stance(robot, np.c_[np.rad2deg(read_arm("high")), [0.0]])
    tcp = (frames["gripper"] @ screen.a.TCP)[0, :3, 3]
    print("Overhead TCP (mm)", np.round(tcp * 1000, 2))
    assert (
        np.linalg.norm(tcp[:2] - [0.2, 0]) < 0.0003 and abs(tcp[2] - 0.65060) < 0.0003
    )


if __name__ == "__main__":
    robot = robot_model()
    verify_mounts(robot)
    verify(robot)
    verify_secondary(robot)
