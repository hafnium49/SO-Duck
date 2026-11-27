"""Replay, search and viewer update for the postures of the hollow design.

    python hollow/postures.py                  # replay targets.csv, pickup.csv and the viewer clips
    python hollow/postures.py --search         # search targets whose row fails or is over the load limit, rewrite targets.csv
    python hollow/postures.py --search-pickup  # search a pickup crouch that passes, rewrite pickup.csv
    python hollow/postures.py --make-clips     # find the overhead and parked postures and the fold between them
    python hollow/postures.py --write-viewer   # write the postures of every clip into viewer.html

Everything runs on the model that cad/build_parts.py writes and applies the checks of the
six-part replay, coverage/replay.py, including the required gap between legs and trunk.
The search stores, for each target, the passing posture with the lowest shoulder torque
under the payload. Run from the repository root after scripts/restore_assets.py,
collision/build_bvh.py and hollow/cad/build_parts.py --stock.
"""

import argparse
import base64
import csv
import gzip
import json
import re

import numpy as np
from scipy.spatial.transform import Rotation

from model import HERE, robot_model, tested_pairs
from inspect_viewer import decode_clip, load_viewer
from replay import FINGER, JOINTS, LEG_GAP, LEG_LINKS, leg_clearance, pair_hits, pose_test, screen
import resolve
import loads
import params

LEGS = [f"{side}_{joint}" for side in screen.legs.SIDES for joint in screen.legs.NAMES]
COLUMNS = [name + "_rad" for name in LEGS + list(JOINTS)]
# Targets with no posture that passes every check. Their rows hold the best posture found.
UNREACHED = (64, 74, 78, 89, 93)
# Targets whose best passing posture loads the shoulder servo beyond the stock SO-101's
# worst case. They are reached, and not counted as covered.
OVER_LIMIT = (5, 18, 50, 53, 60)
MARGIN = resolve.MARGIN


def read_targets():
    with (HERE / "targets.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert [int(row["target"]) for row in rows] == list(range(96))
    return np.array([[float(row[c]) for c in COLUMNS] for row in rows]), [row["source"] for row in rows]


def write_targets(q, source, ok, margin):
    with (HERE / "targets.csv").open("w", newline="") as stream:
        out = csv.writer(stream, lineterminator="\n")
        out.writerow(["target", "bank", "source", "pass", "support_margin_mm"] + COLUMNS)
        for i, row in enumerate(q):
            bank = "front" if i < 64 else "broad"
            out.writerow([i, bank, source[i], int(ok[i]), f"{1000 * margin[i]:.2f}"] + [f"{v:.10f}" for v in row])


def read_pickup():
    table = np.loadtxt(HERE / "pickup.csv", delimiter=",", skiprows=1)
    return table[:, 1:]


def write_pickup(q):
    with (HERE / "pickup.csv").open("w", newline="") as stream:
        out = csv.writer(stream, lineterminator="\n")
        out.writerow(["frame"] + COLUMNS + ["gripper_rad"])
        for i, row in enumerate(q):
            out.writerow([i] + [f"{v:.10f}" for v in row])


def place(robot, q):
    """Frames and centre of mass for rows of 16 joint angles, or 17 with the jaw."""
    jaw = np.rad2deg(q[:, 16]) if q.shape[1] > 16 else np.zeros(len(q))
    return robot.placements(np.c_[np.rad2deg(q[:, 10:16]), jaw], q[:, :10])


def tool(frames):
    return frames["gripper"] @ screen.a.TCP


def offsets(actual, wanted):
    """Distance in mm and angle in degrees between two sets of frames."""
    mm = 1000 * np.linalg.norm(actual[:, :3, 3] - wanted[:, :3, 3], axis=1)
    turn = wanted[:, :3, :3] @ actual[:, :3, :3].transpose(0, 2, 1)
    return mm, np.rad2deg(Rotation.from_matrix(turn).magnitude())


def soles(robot, frames):
    """Largest distance and angle of either sole from its anchor."""
    worst = np.zeros(2)
    for k, side in enumerate(screen.legs.SIDES):
        body = robot.model.body(robot.model.jnt_bodyid[robot.model.joint(side + "_ankle").id]).name
        sole = frames[body] @ screen.legs.specs(side)[4]
        mm, deg = offsets(sole, np.tile(pose_test.ANCHOR[k], (len(sole), 1, 1)))
        worst = np.maximum(worst, [mm.max(), deg.max()])
    return worst


def target_frames():
    positions, rotations = pose_test.make_targets()
    frames = np.tile(np.eye(4), (len(positions), 1, 1))
    frames[:, :3, :3], frames[:, :3, 3] = rotations, positions
    return positions, rotations, frames


def replay_targets(robot, pairs):
    q, _ = read_targets()
    positions, rotations, wanted = target_frames()
    ok, margin = resolve.passes(robot, pairs, q, np.arange(96), positions, rotations)
    frames, _ = place(robot, q)
    mm, deg = offsets(tool(frames), wanted)
    gap = 1000 * leg_clearance(robot, {name: value[ok] for name, value in frames.items()}).min(axis=(0, 1))
    print(f"Targets: front {ok[:64].sum()}/64, broad {ok[64:].sum()}/32, failed {np.flatnonzero(~ok).tolist()}")
    print(f"  tool within {mm[ok].max():.3f} mm and {deg[ok].max():.3f} deg; soles within {soles(robot, frames)[0]:.3f} mm")
    print(f"  smallest support margin {1000 * margin[ok].min():.2f} mm (target {np.where(ok, margin, np.inf).argmin():02d})")
    print("  smallest gap to the trunk: " + ", ".join(f"{n} {g:.2f} mm" for n, g in zip(LEG_LINKS, gap)))
    for t in np.flatnonzero(~ok):
        print(f"  target {t:02d}: support margin {1000 * margin[t]:.2f} mm")
    assert np.flatnonzero(~ok).tolist() == list(UNREACHED)
    inside = within_limit(robot, q)
    covered = ok & inside
    print(
        f"  within the stock shoulder load: {covered.sum()} (front {covered[:64].sum()}/64, broad {covered[64:].sum()}/32); "
        f"reached but over it {np.flatnonzero(ok & ~inside).tolist()}"
    )
    print(
        f"  largest shoulder torque of those {covered.sum()}: {shoulder_torque(robot, q[covered]).max():.3f} N m empty, "
        f"{shoulder_torque(robot, q[covered], params.PAYLOAD_KG).max():.3f} N m with {1000 * params.PAYLOAD_KG:.0f} g"
    )
    assert np.flatnonzero(ok & ~inside).tolist() == list(OVER_LIMIT)
    return q, ok, margin


def shoulder_torque(robot, q, payload=0.0):
    """Gravity torque on the shoulder-pitch servo for each posture, N m."""
    return np.abs(loads.arm_torques(robot, np.atleast_2d(q), payload)[:, loads.SHOULDER])


def within_limit(robot, q):
    """Whether each posture loads the shoulder no more than the stock SO-101's worst case, empty and with the payload."""
    empty, loaded = loads.stock_baseline()
    return (shoulder_torque(robot, q) <= empty) & (shoulder_torque(robot, q, params.PAYLOAD_KG) <= loaded)


def search_targets(robot, pairs, starts):
    """Search every target whose row fails a check or is over the load limit.

    Among the candidates of a target the first that passes, in order of rising shoulder
    torque under the payload, is taken. It replaces the stored row unless that row passes
    with a lower torque.
    """
    q, _ = read_targets()
    positions, rotations, _ = target_frames()
    everything = np.arange(96)
    ok, _ = resolve.passes(robot, pairs, q, everything, positions, rotations)
    settled = ok & within_limit(robot, q)
    missing = np.flatnonzero(~settled).tolist()
    print(f"Stored rows that pass within the load limit: {int(settled.sum())} of 96; searching {len(missing)}", flush=True)
    if missing:
        found = resolve.candidates(robot, resolve.chain_frames(robot), pairs, missing, positions, rotations, starts)
        print(f"Converged candidates: {sum(len(rows) for rows in found.values())} for {len(missing)} targets", flush=True)
        rank = lambda rows: shoulder_torque(robot, rows, params.PAYLOAD_KG)
        for t, row in resolve.choose(robot, pairs, found, positions, rotations, rank=rank).items():
            if not ok[t] or rank(row)[0] < rank(q[t])[0]:
                q[t] = row
    ok, margin = resolve.passes(robot, pairs, q, everything, positions, rotations)
    inside = within_limit(robot, q)
    write_targets(q, ["re-solved"] * 96, ok, margin)
    covered = ok & inside
    print(f"Rewrote targets.csv. Reached {int(ok.sum())}; within the load limit {int(covered.sum())} (front {int(covered[:64].sum())}/64, broad {int(covered[64:].sum())}/32)")
    print(f"  no passing posture: {np.flatnonzero(~ok).tolist()}")
    print(f"  reached but over the load limit: {np.flatnonzero(ok & ~inside).tolist()}")
    if covered.any():
        print(f"  largest shoulder torque of a covered target: {shoulder_torque(robot, q[covered]).max():.3f} N m empty, {shoulder_torque(robot, q[covered], params.PAYLOAD_KG).max():.3f} N m with {1000 * params.PAYLOAD_KG:.0f} g")


def fixed_legs(robot, pairs, q, name, pad_contact=()):
    """Checks for a clip in which the legs hold one posture."""
    low, high = resolve.joint_limits(robot)
    assert np.ptp(q[:, :10], axis=0).max() < 1e-9, f"{name}: the legs move"
    assert np.all(q[:, :16] >= low - 1e-7) and np.all(q[:, :16] <= high + 1e-7), f"{name}: joint limit"
    frames, com = place(robot, q)
    margin = pose_test.GROUND.margin(com)
    assert margin.min() >= MARGIN, f"{name}: support margin {1000 * margin.min():.2f} mm"
    assert soles(robot, frames)[0] < 0.2, f"{name}: a sole leaves its anchor"
    gap = leg_clearance(robot, {key: value[:1] for key, value in frames.items()})
    assert (gap >= LEG_GAP).all(), f"{name}: a leg link is too close to the trunk"
    for pair in pairs:
        hit = pair_hits(robot, pair, frames)
        if set(pair[:2]) == {"gripper", "park_pad"}:
            hit[list(pad_contact)] = False
        assert not hit.any(), (name, *pair[:2], np.flatnonzero(hit)[:5])
    floor = np.min([robot.lowest(n, frames) for n in robot.mesh if not n.startswith("foot_assembly")], axis=0)
    assert floor.min() >= -0.0001, f"{name}: below the floor"
    print(
        f"{name}: {len(q)} frames, support margin at least {1000 * margin.min():.2f} mm, "
        f"lowest geometry other than the feet {1000 * floor.min():.2f} mm, "
        f"hip link gap {1000 * gap[0, :, 0].min():.2f} mm, thigh gap {1000 * gap[0, :, 1].min():.2f} mm"
    )
    return frames


def pickup_path():
    """Tool frames of the 81 pickup frames: down a vertical line over the cube, 20 frames while the jaw closes, and up."""
    high, low = params.PICKUP_TOOL_Z_MM
    height = np.r_[np.linspace(high, low, 31), np.full(20, low), np.linspace(low, high, 31)[1:]]
    frames = np.tile(frame((*params.PICKUP_TOOL_XY_MM, 0.0), params.PICKUP_TOOL_QUAT), (len(height), 1, 1))
    frames[:, 2, 3] = height / 1000
    return frames


def replay_pickup(robot, pairs):
    q = read_pickup()
    frames = fixed_legs(robot, pairs, q, "Pickup")
    path = tool(frames)
    mm, _ = offsets(path, pickup_path())
    assert mm.max() < 0.01, f"Pickup: the tool is {mm.max():.3f} mm off its path"
    _, deg = offsets(path, np.tile(path[0], (len(path), 1, 1)))
    sideways = 1000 * np.abs(path[:, :2, 3] - path[0, :2, 3]).max()
    assert sideways < 0.01 and deg.max() < 0.05, "Pickup: the tool leaves its vertical line"
    height = 1000 * path[:, 2, 3]
    print(f"  tool stays on a vertical line within {sideways:.4f} mm; height {height[0]:.2f}, {height[30]:.2f}, {height[-1]:.2f} mm")
    return q


ARM_COLUMNS = [name + "_rad" for name in JOINTS]
CLIP_STARTS = 2048  # starting arm postures of each clip search


def read_arm(name):
    """Rows of six arm angles in radians from a clip file."""
    return np.atleast_2d(np.loadtxt(HERE / f"{name}.csv", delimiter=",", skiprows=1))


def write_arm(name, rows):
    with (HERE / f"{name}.csv").open("w", newline="") as stream:
        out = csv.writer(stream, lineterminator="\n")
        out.writerow(ARM_COLUMNS)
        for row in np.atleast_2d(rows):
            out.writerow([f"{v:.10f}" for v in row])


def frame(position_mm, quaternion):
    """A 4x4 frame in metres from a position in millimetres and a quaternion (w, x, y, z)."""
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(np.roll(quaternion, -1)).as_matrix()
    out[:3, 3] = np.asarray(position_mm, dtype=float) / 1000
    return out


def at_reference(arm, jaw=0.0):
    """Rows of 17 angles: the reference stance, the given arm angles in radians, the jaw."""
    arm = np.atleast_2d(arm)
    return np.c_[np.tile(pose_test.REF.ravel(), (len(arm), 1)), arm, np.full(len(arm), jaw)]


def clear(robot, pairs, q, pad_contact=()):
    """Whether each row passes the checks of fixed_legs. pad_contact lists the rows in which the gripper may rest on the pad."""
    if not len(q):
        return np.zeros(0, bool)
    low, high = resolve.joint_limits(robot)
    frames, com = place(robot, q)
    ok = np.all(q[:, :16] >= low - 1e-7, axis=1) & np.all(q[:, :16] <= high + 1e-7, axis=1)
    ok &= pose_test.GROUND.margin(com) >= MARGIN
    ok &= np.min([robot.lowest(n, frames) for n in robot.mesh if not n.startswith("foot_assembly")], axis=0) >= -0.0001
    for pair in pairs:
        keep = np.flatnonzero(ok)
        if not len(keep):
            break
        hit = pair_hits(robot, pair, {name: value[keep] for name, value in frames.items()})
        if set(pair[:2]) == {"gripper", "park_pad"}:
            hit[np.isin(keep, list(pad_contact))] = False
        ok[keep[hit]] = False
    return ok


def arm_starts(robot, seed, count=CLIP_STARTS):
    low, high = resolve.joint_limits(robot)
    return low[10:] + (high[10:] - low[10:]) * resolve.qmc.Sobol(6, scramble=True, seed=seed).random(count)


def reach(robot, arm, position_mm, quaternion=None, steps=150):
    """Move arm postures at the reference stance until the tool is at the position, and at the rotation if one is given.

    Returns the arm angles and whether each posture arrived inside the joint limits.
    """
    chain = resolve.chain_frames(robot)
    low, high = resolve.joint_limits(robot)
    q = at_reference(arm)[:, :16]
    target = frame(position_mm, (1, 0, 0, 0) if quaternion is None else quaternion)
    wanted = np.tile(target[:3, 3], (len(q), 1)), np.tile(target[:3, :3], (len(q), 1, 1))
    rows = slice(0, 3 if quaternion is None else 6)
    for step in range(steps + 1):
        tip, foot, jacobian = pose_test.FK(q, chain)
        error = pose_test.errors(wanted, tip, foot)[:, rows]
        if step == steps:
            break
        part = jacobian[:, rows, 10:]
        normal = part.transpose(0, 2, 1) @ part + 1e-6 * np.eye(6)
        move = np.linalg.solve(normal, part.transpose(0, 2, 1) @ error[:, :, None])[:, :, 0]
        q[:, 10:] = np.clip(q[:, 10:] + np.clip(move, -0.2, 0.2), low[10:], high[10:])
    return q[:, 10:], np.abs(error).max(axis=1) < 1e-6


def overhead_postures(robot, pairs, height_mm):
    """Arm postures that pass every check with the tool at this height on the centre line, largest support margin first."""
    arm, arrived = reach(robot, arm_starts(robot, 300), (*params.OVERHEAD_XY_MM, height_mm))
    rows = at_reference(arm[arrived])
    rows = rows[clear(robot, pairs, rows)]
    if not len(rows):
        return rows[:, 10:16]
    margin = pose_test.GROUND.margin(place(robot, rows)[1])
    return rows[np.argsort(-margin), 10:16]


def find_overhead(robot, pairs):
    """The greatest whole height in millimetres at which a posture passes, and the postures there."""
    low = 400
    best = overhead_postures(robot, pairs, low)
    assert len(best), "no overhead posture passes at 400 mm"
    high = low + 32
    while True:  # climb until a height fails, then close in on the last that passes
        found = overhead_postures(robot, pairs, high)
        if not len(found):
            break
        low, best, high = high, found, high + 32
    while high - low > 1:
        middle = (low + high) // 2
        found = overhead_postures(robot, pairs, middle)
        if len(found):
            low, best = middle, found
        else:
            high = middle
    return low, best


def find_park(robot, pairs):
    """Arm postures that rest the fixed fingertip on the pad as the earlier design did, and pass every other check."""
    arm, arrived = reach(robot, arm_starts(robot, 301, 2 * CLIP_STARTS), params.PARK_TOOL_MM, params.PARK_TOOL_QUAT)
    rows = at_reference(arm[arrived])
    return rows[clear(robot, pairs, rows, pad_contact=range(len(rows))), 10:16]


def path_between(points):
    """Arm angles in radians through the given postures, in steps of at most half a degree per joint."""
    from motion import between

    degrees = np.rad2deg(np.atleast_2d(points))
    legs = [between(a, b)[:-1] for a, b in zip(degrees[:-1], degrees[1:])]
    return np.deg2rad(np.vstack(legs + [degrees[-1:]]))


def path_clear(robot, pairs, path):
    """Whether a path at the reference stance passes every check, the pad being touched only at its end."""
    rows = at_reference(path)
    coarse = np.unique(np.r_[np.arange(0, len(rows), 8), len(rows) - 1])
    if not clear(robot, pairs, rows[coarse], pad_contact=[len(coarse) - 1]).all():
        return False
    return bool(clear(robot, pairs, rows, pad_contact=[len(rows) - 1]).all())


def find_fold(robot, pairs, overhead, parks, tries=128):
    """A parked posture and waypoints with a clear path from overhead: direct if possible, else through one sampled posture."""
    parks = parks[np.argsort(np.abs(parks - overhead).max(axis=1))][:6]
    samples = arm_starts(robot, 302, tries)
    for park in parks:
        if path_clear(robot, pairs, path_between([overhead, park])):
            return park, np.zeros((0, 6))
    for park in parks:
        middle = (overhead + park) / 2
        for blend in (0.25, 0.5, 0.75, 1.0):
            for sample in samples:
                via = middle + blend * (sample - middle)
                if clear(robot, pairs, at_reference(via)).all() and path_clear(robot, pairs, path_between([overhead, via, park])):
                    return park, via[None]
        print(f"  no fold found to the parked posture {np.rad2deg(park).round(1).tolist()}", flush=True)
    raise AssertionError("no clear fold from overhead to parked")


def make_clips(robot, pairs):
    height, overheads = find_overhead(robot, pairs)
    print(f"Overhead: {height} mm is the highest that passes ({len(overheads)} postures)", flush=True)
    parks = find_park(robot, pairs)
    assert len(parks), "no parked posture passes"
    print(f"Parked: {len(parks)} postures pass", flush=True)
    park, via = find_fold(robot, pairs, overheads[0], parks)
    write_arm("overhead", overheads[0])
    write_arm("park", park)
    write_arm("fold_waypoint", via)
    print(f"Fold: {len(via)} waypoint(s), {len(fold_path())} frames. Wrote overhead.csv, park.csv, fold_waypoint.csv")


def fold_path():
    """Overhead through the waypoints to parked at the reference stance: rows of 17 angles."""
    via = np.loadtxt(HERE / "fold_waypoint.csv", delimiter=",", skiprows=1).reshape(-1, 6)
    return at_reference(path_between(np.vstack([read_arm("overhead"), via, read_arm("park")])))


def overhead_height_mm(robot):
    """Height of the tool in the stored overhead posture."""
    return float(1000 * tool(place(robot, at_reference(read_arm("overhead")))[0])[0, 2, 3])


def pad_rest(robot, frames):
    """How far the fixed fingertip sinks into the pad's top, and how far the moving jaw stays above it, in millimetres."""
    last = {name: value[-1:] for name, value in frames.items()}
    top = (last["park_pad"][0] @ [0, 0, robot.mesh["park_pad"].reshape(-1, 3)[:, 2].max(), 1])[2]
    return 1000 * (top - robot.lowest("gripper", last)[0]), 1000 * (robot.lowest(FINGER, last)[0] - top)


def replay_clips(robot, pairs):
    """Overhead, fold and parked postures from their files, at the reference stance."""
    frames = fixed_legs(robot, pairs, at_reference(read_arm("overhead")), "Overhead")
    point = 1000 * tool(frames)[0, :3, 3]
    assert np.abs(point[:2] - params.OVERHEAD_XY_MM).max() < 0.3, "Overhead: the tool is off the centre line"
    print(f"  tool at {point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f} mm")
    path = fold_path()
    assert np.abs(np.diff(np.rad2deg(path[:, 10:16]), axis=0)).max() <= 0.5 + 1e-9
    frames = fixed_legs(robot, pairs, path, "Fold", pad_contact=[len(path) - 1])
    sink, jaw = pad_rest(robot, frames)
    assert 0 < sink < 1 and jaw > 0, f"Parked: fingertip {sink:.2f} mm into the pad, jaw {jaw:.2f} mm above it"
    print(f"  parked: fixed fingertip {sink:.2f} mm into the pad, moving jaw {jaw:.2f} mm above its top")
    return path


def viewer_frames(data, name):
    world = decode_clip(data, name)[1]
    frames = np.tile(np.eye(4), (*world.shape[:2], 1, 1))
    frames[:, :, :3] = world
    return frames


def compare_viewer(data, name, frames):
    """Largest difference between the viewer's stored transforms and the model's."""
    stored = viewer_frames(data, name)
    worst = 0.0
    for b, body in enumerate(data["bodies"]):
        if body in frames:
            worst = max(worst, np.abs(stored[:, b] - frames[body]).max())
    return worst


def replay_viewer(robot):
    """The viewer shows the postures of the CSV files in every clip."""
    data = load_viewer()
    for name, q in clip_rows().items():
        clip = next(c for c in data["poses"] if c["name"] == name)
        assert clip["n"] == len(q), f"{name}: the viewer holds {clip['n']} frames, the files {len(q)}"
        worst = compare_viewer(data, name, place(robot, q)[0])
        print(f"Viewer clip {name} against the CSV postures: largest transform difference {worst:.1e}")
        assert worst < 2e-6, name


def search_pickup(robot, pairs, starts):
    """A crouch that passes every check along the stored tool path."""
    path = pickup_path()
    jaw = read_pickup()[:, 16]
    chain = resolve.chain_frames(robot)
    free = resolve.free_arm_ranges(robot, pairs)
    index = np.arange(len(path))
    positions, rotations = path[:, :3, 3], path[:, :3, :3]
    best = None
    for k, (yaw, spread, roll) in enumerate(resolve.SEARCHES):
        low, high = resolve.joint_limits(robot, yaw, free, roll)
        draws = resolve.qmc.Sobol(16, scramble=True, seed=700 + k).random(starts)
        start = np.tile(np.r_[pose_test.REF.ravel(), np.zeros(6)], (starts, 1))
        start[:, 10:] = low[10:] + (high[10:] - low[10:]) * draws[:, :6]
        start[:, :10] += (draws[:, 6:] - 0.5) * 2 * spread
        grasp = np.full(starts, 30)
        crouch, solved = resolve.converge(start, chain, positions[grasp], rotations[grasp], low, high)
        crouch = crouch[solved]
        ok, margin = resolve.passes(robot, pairs, crouch, np.full(len(crouch), 30), positions, rotations)
        for row in crouch[ok][np.argsort(-margin[ok])][:6]:
            rows = np.tile(row, (31, 1))
            for f in range(30, -1, -1):  # settle the grasp, then walk the arm up the path with the legs held
                rows[f] = rows[min(f + 1, 30)]
                for _ in range(60):
                    tip, foot, jacobian = pose_test.FK(rows[f : f + 1], chain)
                    error = pose_test.errors((positions[f : f + 1], rotations[f : f + 1]), tip, foot)[0, :6]
                    if np.abs(error).max() < 1e-9:
                        break
                    arm = jacobian[0, :6, 10:]
                    rows[f, 10:] += np.linalg.solve(arm.T @ arm + 1e-12 * np.eye(6), arm.T @ error)
            ok_path, margin_path = resolve.passes(robot, pairs, rows, index[:31], positions, rotations)
            if ok_path.all() and (best is None or margin_path.min() > best[0]):
                best = (margin_path.min(), rows.copy())
        print(f"  hip yaw {yaw}, roll limit {roll}: {len(crouch)} crouches reach the grasp, {int(ok.sum())} pass there", flush=True)
    assert best is not None, "no pickup crouch passes"
    rows = best[1]
    full = np.vstack([rows, np.tile(rows[30], (20, 1)), rows[29::-1]])
    write_pickup(np.c_[full, jaw])
    print(f"Rewrote pickup.csv; smallest support margin along the path {1000 * best[0]:.2f} mm")


ASSEMBLY_DEG = (0.0, 48.0, -59.0, 0.0, 14.0, 8.0, 12.0)  # the viewer's opening pose: six arm angles and the jaw


def clip_rows():
    """The postures of every viewer clip, as rows of 17 angles."""
    targets, _ = read_targets()
    return {
        "targets": np.c_[targets, np.zeros(len(targets))],
        "assembly": at_reference(np.deg2rad(ASSEMBLY_DEG[:6]), np.deg2rad(ASSEMBLY_DEG[6])),
        "pickup": read_pickup(),
        "fold": fold_path(),
        "overhead": at_reference(read_arm("overhead")),
        "park": at_reference(read_arm("park")),
    }


def viewer_clips(data, robot):
    """Store the model's transforms of every clip in the viewer's data."""
    bodies = data["bodies"]
    for name, q in clip_rows().items():
        clip = next(c for c in data["poses"] if c["name"] == name)
        frames, _ = place(robot, q)
        stored = viewer_frames(data, name)
        world = np.zeros((len(q), len(bodies), 3, 4))
        for b, body in enumerate(bodies):
            if body in frames:
                world[:, b] = frames[body][:, :3]
            else:  # the cube keeps its stored placement: the gripper's path over it has not changed
                world[:, b] = stored[:, b, :3] if len(stored) == len(q) else stored[0, b, :3]
        point = tool(frames)
        axis = frames["wrist_flex_link"]
        clip["n"] = len(q)
        clip["q"] = [[round(float(v), 2) for v in row] for row in np.rad2deg(q[:, 10:17])]
        clip["xyz"] = [[round(float(v), 2) for v in row] for row in 1000 * point[:, :3, 3]]
        clip["roll"] = [[round(float(v), 7) for v in [*row[:3, 3], *row[:3, 2]]] for row in axis]
        clip["world"] = base64.b64encode(world.astype("<f4").tobytes()).decode()
        print(f"Viewer clip {name}: {len(q)} frames")


def write_viewer(robot):
    import viewer_build

    data = load_viewer()
    viewer_clips(data, robot)
    viewer_build.save(data, (HERE / "viewer.html").read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--search", action="store_true")
    parser.add_argument("--search-pickup", action="store_true")
    parser.add_argument("--write-viewer", action="store_true")
    parser.add_argument("--make-clips", action="store_true")
    parser.add_argument("--starts", type=int, default=128, help="starting postures per search")
    args = parser.parse_args()
    robot = robot_model()
    pairs = tested_pairs(robot)
    if args.search:
        search_targets(robot, pairs, args.starts)
    elif args.search_pickup:
        search_pickup(robot, pairs, args.starts)
    elif args.make_clips:
        make_clips(robot, pairs)
    elif args.write_viewer:
        write_viewer(robot)
    else:
        replay_targets(robot, pairs)
        replay_pickup(robot, pairs)
        replay_clips(robot, pairs)
        replay_viewer(robot)
        print("POSTURE REPLAY PASSED")


if __name__ == "__main__":
    main()
