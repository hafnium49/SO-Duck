"""Replay, search and viewer update for the postures of the hollow design.

    python hollow/postures.py                  # replay targets.csv, pickup.csv and the viewer clips
    python hollow/postures.py --search         # search targets whose row fails or is over the load limit, rewrite targets.csv
    python hollow/postures.py --search-pickup  # search a pickup crouch that passes, rewrite pickup.csv
    python hollow/postures.py --write-viewer   # write the CSV postures into viewer.html

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
from replay import JOINTS, LEG_GAP, LEG_LINKS, leg_clearance, pair_hits, pose_test, screen
import resolve
import loads
import params

LEGS = [f"{side}_{joint}" for side in screen.legs.SIDES for joint in screen.legs.NAMES]
COLUMNS = [name + "_rad" for name in LEGS + list(JOINTS)]
# Targets with no posture that passes every check. Their rows hold the best posture found.
UNREACHED = (64, 74, 78, 89, 93)
# Targets whose best passing posture loads the shoulder servo beyond the stock SO-101's
# worst case. They are reached, and not counted as covered.
OVER_LIMIT = (5, 18, 21, 50, 53, 60)
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


def replay_pickup(robot, pairs):
    q = read_pickup()
    frames = fixed_legs(robot, pairs, q, "Pickup")
    path = tool(frames)
    _, deg = offsets(path, np.tile(path[0], (len(path), 1, 1)))
    sideways = 1000 * np.abs(path[:, :2, 3] - path[0, :2, 3]).max()
    assert sideways < 0.01 and deg.max() < 0.05, "Pickup: the tool leaves its vertical line"
    height = 1000 * path[:, 2, 3]
    print(f"  tool stays on a vertical line within {sideways:.4f} mm; height {height[0]:.2f}, {height[30]:.2f}, {height[-1]:.2f} mm")
    return q


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


def replay_viewer(robot, pairs, targets, pickup):
    """The viewer shows the CSV postures, and its other clips pass at the reference stance."""
    data = load_viewer()
    for name, q in (("targets", targets), ("pickup", pickup)):
        worst = compare_viewer(data, name, place(robot, q)[0])
        print(f"Viewer clip {name} against the CSV postures: largest transform difference {worst:.1e}")
        assert worst < 2e-6, name
    reference = pose_test.REF.ravel()
    for name in ("fold", "overhead", "park"):
        clip = next(c for c in data["poses"] if c["name"] == name)
        q = np.c_[np.tile(reference, (clip["n"], 1)), np.deg2rad(np.array(clip["q"]))]
        contact = [clip["n"] - 1] if name in ("fold", "park") else []
        frames = fixed_legs(robot, pairs, q, name.capitalize(), pad_contact=contact)
        # Stored angles are rounded to 0.01 deg, which moves the tool by up to 0.3 mm.
        assert compare_viewer(data, name, frames) < 5e-4, name


def search_pickup(robot, pairs, starts):
    """A crouch that passes every check along the stored tool path."""
    q = read_pickup()
    path = tool(place(robot, q)[0])
    jaw = q[:, 16]
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


def write_viewer(robot):
    """Store the model's transforms for every frame whose CSV posture differs from the viewer."""
    data = load_viewer()
    targets, _ = read_targets()
    total = 0
    for name, q in (("targets", targets), ("pickup", read_pickup())):
        clip = next(c for c in data["poses"] if c["name"] == name)
        frames, _ = place(robot, q)
        stored = viewer_frames(data, name)
        differs = np.zeros(len(q), bool)
        for b, body in enumerate(data["bodies"]):
            if body in frames:
                differs |= np.abs(stored[:, b] - frames[body]).reshape(len(q), -1).max(1) >= 1e-6
        world = np.frombuffer(base64.b64decode(clip["world"]), dtype="<f4").reshape(len(q), len(data["bodies"]), 3, 4).copy()
        point = tool(frames)
        jaw = np.rad2deg(q[:, 16]) if q.shape[1] > 16 else np.zeros(len(q))
        for i in np.flatnonzero(differs):
            for b, body in enumerate(data["bodies"]):
                if body in frames:  # the cube keeps its stored placement
                    world[i, b] = frames[body][i, :3]
            clip["xyz"][i] = [round(float(v), 2) for v in 1000 * point[i, :3, 3]]
            clip["q"][i] = [round(float(v), 2) for v in [*np.rad2deg(q[i, 10:16]), jaw[i]]]
            axis = frames["wrist_flex_link"][i]
            clip["roll"][i] = [round(float(v), 7) for v in [*axis[:3, 3], *axis[:3, 2]]]
        clip["world"] = base64.b64encode(world.astype("<f4").tobytes()).decode()
        total += int(differs.sum())
        print(f"Viewer clip {name}: rewrote {int(differs.sum())} of {len(q)} frames")
    blob = base64.b64encode(gzip.compress(json.dumps(data, separators=(",", ":")).encode(), 9, mtime=0)).decode()
    html = (HERE / "viewer.html").read_text(encoding="utf-8")
    html, count = re.subn(r'(<script id="payload"[^>]*>).*?(</script>)', lambda m: m.group(1) + blob + m.group(2), html, count=1, flags=re.S)
    assert count == 1
    if total:
        (HERE / "viewer.html").write_text(html, encoding="utf-8", newline="\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--search", action="store_true")
    parser.add_argument("--search-pickup", action="store_true")
    parser.add_argument("--write-viewer", action="store_true")
    parser.add_argument("--starts", type=int, default=128, help="starting postures per search")
    args = parser.parse_args()
    robot = robot_model()
    pairs = tested_pairs(robot)
    if args.search:
        search_targets(robot, pairs, args.starts)
    elif args.search_pickup:
        search_pickup(robot, pairs, args.starts)
    elif args.write_viewer:
        write_viewer(robot)
    else:
        targets, _, _ = replay_targets(robot, pairs)
        pickup = replay_pickup(robot, pairs)
        replay_viewer(robot, pairs, targets, pickup)
        print("POSTURE REPLAY PASSED")


if __name__ == "__main__":
    main()
