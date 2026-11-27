"""Search whole-body postures that reach the published targets with the legs off the trunk.

Rows of targets.csv that pass every check of replay.py are kept as they are. For the
others this script searches new postures with the solver of study/pose_test.py and
keeps, for each target, the passing posture with the largest support margin.

    OPENBLAS_NUM_THREADS=1 python coverage/resolve.py            # search and rewrite targets.csv
    OPENBLAS_NUM_THREADS=1 python coverage/resolve.py --dry-run  # report only

Run from the repository root after scripts/restore_assets.py and collision/build_bvh.py.
"""

import argparse
import csv

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc

from replay import (
    FINGER,
    HERE,
    JOINTS,
    LEG_GAP,
    collision_pairs,
    leg_clearance,
    pair_hits,
    pose_test,
    read_targets,
    robot_model,
    screen,
)

LENGTH = pose_test.L
RESERVE = np.deg2rad(2)
# Solver tolerances sit inside the replay limits of 1.5 mm, 1 deg, 0.2 mm and 0.15 deg.
SOLVED = np.array([1.2, 0.8, 0.15, 0.12])
MARGIN = 0.005
# Hip-yaw range in degrees (toes-out positive), spread of the starting leg posture in
# radians, and the largest hip roll toward the trunk in degrees. A narrow yaw range keeps
# the thighs away from the trunk shell. The roll limit keeps the hip link away from it:
# the hip link is 0.37 mm from the trunk at 15 degrees of roll in the reference stance.
SEARCHES = [
    ((-28, 28), 0.0, None),
    ((-12, 18), 0.0, None),
    ((-6, 14), 0.10, None),
    ((2, 12), 0.10, None),
    ((-12, 18), 0.20, None),
    ((-20, 22), 0.25, None),
    ((-28, 28), 0.30, None),
    ((-8, 16), 0.30, None),
    ((-28, 28), 0.20, 14),
    ((-12, 18), 0.20, 14),
    ((-20, 22), 0.30, 12),
    ((-6, 14), 0.25, 10),
]


def chain_frames(robot):
    """Joint frames of the arm and the tool frame, as study/pose_test.py expects them."""
    frames = []
    for name in JOINTS:
        body = robot.model.jnt_bodyid[robot.model.joint(name).id]
        frame = np.eye(4)
        frame[:3, :3] = Rotation.from_quat(np.roll(robot.model.body_quat[body], -1)).as_matrix()
        frame[:3, 3] = robot.model.body_pos[body]
        frames.append(frame)
    return np.array(frames + [screen.a.TCP])


def free_arm_ranges(robot, pairs):
    """For each arm joint, the range around zero in which its two neighbouring links stay apart.

    The model ranges of some joints reach past the angle at which the link on one side
    of the joint meets the link on the other side. No posture beyond that angle can
    pass, so the search stays inside it.
    """
    geoms = {}
    for i in range(robot.model.ngeom):
        name = robot.model.geom(i).name
        if name in robot.mesh:
            geoms.setdefault(robot.model.geom_bodyid[i], []).append(name)
    geoms.update({robot.body[n]: geoms.get(robot.body[n], []) + [n] for n in ("gripper", FINGER)})
    tested = {frozenset(pair[:2]): pair for pair in pairs}
    angles = np.arange(-180, 181)
    ranges = []
    for k, joint in enumerate(JOINTS):
        child = robot.model.jnt_bodyid[robot.model.joint(joint).id]
        parent = robot.model.body_parentid[child]
        sweep = np.zeros((len(angles), 7))
        sweep[:, k] = angles
        frames, _ = robot.placements(sweep)
        hit = np.zeros(len(angles), bool)
        for a in set(geoms.get(parent, [])):
            for b in set(geoms.get(child, [])):
                if frozenset((a, b)) in tested:
                    hit |= pair_hits(robot, tested[frozenset((a, b))], frames)
        zero = len(angles) // 2
        assert not hit[zero], joint
        below = np.flatnonzero(hit[:zero])
        above = np.flatnonzero(hit[zero:])
        ranges.append((angles[below[-1] + 1] if len(below) else -180, angles[zero + above[0] - 1] if len(above) else 180))
    return np.deg2rad(ranges)


def joint_limits(robot, yaw=None, free=None, roll=None):
    """Lower and upper solver bounds of the 16 joints, inside the 2 degree reserve."""
    names = [f"{s}_{j}" for s in screen.legs.SIDES for j in screen.legs.NAMES] + list(JOINTS)
    limits = np.array([robot.model.joint(n).range for n in names])
    low, high = limits[:, 0] + RESERVE, limits[:, 1] - RESERVE
    if free is not None:
        low[10:], high[10:] = np.maximum(low[10:], free[:, 0]), np.minimum(high[10:], free[:, 1])
    if yaw is not None:
        inward, outward = np.deg2rad(yaw)
        low[0], high[0] = max(low[0], inward), min(high[0], outward)
        low[5], high[5] = max(low[5], -outward), min(high[5], -inward)
    if roll is not None:
        high[1], low[6] = min(high[1], np.deg2rad(roll)), max(low[6], -np.deg2rad(roll))
    return low, high


def pose_errors(q, frames, positions, rotations):
    """Tool and right-sole errors in mm and degrees, and the solver's Jacobian."""
    tip, foot, jacobian = pose_test.FK(q, frames)
    error = pose_test.errors((positions, rotations), tip, foot)
    size = np.linalg.norm(error.reshape(-1, 4, 3), axis=2)
    size[:, [0, 2]] *= 1000 * LENGTH
    size[:, [1, 3]] *= 180 / np.pi
    return error, size, jacobian


def converge(q, frames, positions, rotations, low, high, steps=110):
    """Damped least squares on tool pose and right sole, as in pose_test.solve."""
    q = np.clip(q, low, high)
    for _ in range(steps):
        error, size, jacobian = pose_errors(q, frames, positions, rotations)
        active = ~(size < SOLVED / 2).all(axis=1)
        if not active.any():
            break
        qs, error, jacobian = q[active], error[active], jacobian[active]
        jacobian[:, 6:] *= 2.0
        error[:, 6:] *= 2.0
        free = np.ones_like(qs)
        for sweep in range(3):
            a = jacobian * free[:, None, :]
            normal = a @ a.swapaxes(1, 2)
            normal[:, np.arange(12), np.arange(12)] += 0.035**2
            step = (a.swapaxes(1, 2) @ np.linalg.solve(normal, error[:, :, None]))[:, :, 0]
            if sweep < 2:
                free[((qs < low + 0.002) & (step < 0)) | ((qs > high - 0.002) & (step > 0))] = 0.0
        scale = np.max(np.abs(step), axis=1).clip(min=0.24) / 0.24
        q[active] = np.clip(qs + step / scale[:, None], low, high)
    size = pose_errors(q, frames, positions, rotations)[1]
    return q, (size < SOLVED).all(axis=1)


def candidates(robot, frames, pairs, targets, positions, rotations, starts):
    """Converged postures for each requested target, from every search in SEARCHES."""
    found = {t: [] for t in targets}
    reference = np.r_[pose_test.REF.ravel(), np.zeros(6)]
    free = free_arm_ranges(robot, pairs)
    print("Arm joint ranges with neighbouring links apart (deg):", np.rad2deg(free).round().astype(int).tolist())
    for k, (yaw, spread, roll) in enumerate(SEARCHES):
        low, high = joint_limits(robot, yaw, free, roll)
        draws = qmc.Sobol(16, scramble=True, seed=900 + k).random(starts)
        start = np.tile(reference, (starts, 1))
        start[:, 10:] = low[10:] + (high[10:] - low[10:]) * draws[:, :6]
        start[:, :10] += (draws[:, 6:] - 0.5) * 2 * spread
        q = np.repeat(start, len(targets), axis=0)
        index = np.tile(np.arange(len(targets)), starts)
        q, solved = converge(q, frames, positions[targets][index], rotations[targets][index], low, high)
        for row, t in zip(q[solved], np.asarray(targets)[index[solved]]):
            found[t].append(row)
    return {t: np.array(rows).reshape(-1, 16) for t, rows in found.items()}


def passes(robot, pairs, q, targets, positions, rotations, full=True):
    """Every check of replay.verify for postures q of the given targets.

    Returns the pass flag and the support margin of each posture. With full=False the
    mesh intersections and the jaw sweep are skipped.
    """
    low, high = joint_limits(robot)
    arm = np.c_[np.rad2deg(q[:, 10:]), np.zeros(len(q))]
    frames, com = robot.placements(arm, q[:, :10])
    margin = pose_test.GROUND.margin(com)
    ok = np.all(q >= low - 1e-7, axis=1) & np.all(q <= high + 1e-7, axis=1) & (margin >= MARGIN)

    def close(actual, position, rotation, mm, degrees):
        distance = 1000 * np.linalg.norm(actual[:, :3, 3] - position, axis=1)
        angle = np.rad2deg(Rotation.from_matrix(rotation @ actual[:, :3, :3].transpose(0, 2, 1)).magnitude())
        return (distance < mm) & (angle < degrees)

    ok &= close(frames["gripper"] @ screen.a.TCP, positions[targets], rotations[targets], 1.5, 1.0)
    for k, side in enumerate(screen.legs.SIDES):
        body = robot.model.body(robot.model.jnt_bodyid[robot.model.joint(side + "_ankle").id]).name
        anchor = pose_test.ANCHOR[k]
        ok &= close(frames[body] @ screen.legs.specs(side)[4], anchor[:3, 3], anchor[:3, :3], 0.2, 0.15)
    floor = np.min([robot.lowest(n, frames) for n in robot.mesh if not n.startswith("foot_assembly")], axis=0)
    ok &= floor >= -0.0001
    if not full or not ok.any():
        return ok, margin

    keep = np.flatnonzero(ok)
    part = {name: value[keep] for name, value in frames.items()}
    clear = (leg_clearance(robot, part) >= LEG_GAP).all(axis=(1, 2))
    ok[keep[~clear]] = False
    keep, part = keep[clear], {name: value[clear] for name, value in part.items()}
    if not len(keep):
        return ok, margin
    hit = np.zeros(len(keep), bool)
    for pair in pairs:
        hit |= pair_hits(robot, pair, part)
    ok[keep[hit]] = False
    keep = keep[~hit]
    if not len(keep):
        return ok, margin

    # The moving finger is the only mesh that changes with the jaw opening.
    finger_pairs = [pair for pair in pairs if FINGER in pair[:2]]
    bad = np.zeros(len(keep), bool)
    for opening in range(2, 21, 2):
        sweep, com = robot.placements(np.c_[np.rad2deg(q[keep, 10:]), np.full(len(keep), opening)], q[keep, :10])
        for pair in finger_pairs:
            bad |= pair_hits(robot, pair, sweep)
        bad |= robot.lowest(FINGER, sweep) < -0.0001
        bad |= pose_test.GROUND.margin(com) < MARGIN
    ok[keep[bad]] = False
    return ok, margin


def choose(robot, pairs, found, positions, rotations, block=8, depth=240, rank=None):
    """For each target the first passing candidate in order of preference.

    Candidates are tried in order of falling support margin, or of rising rank(q) when a
    rank function is given. A target still open after the first
    candidates has the leg gap measured for all of them, and only those with the gap are
    tried further.
    """
    ranked = {}
    for t, q in found.items():
        if len(q):
            ok, margin = passes(robot, pairs, q, np.full(len(q), t), positions, rotations, full=False)
            order = np.argsort(-margin if rank is None else rank(q), kind="stable")
            ranked[t] = q[order[ok[order]]]
    chosen = {}
    for first in range(0, depth, block):
        waiting = [t for t, q in ranked.items() if t not in chosen and len(q) > first]
        if not waiting:
            break
        q = np.concatenate([ranked[t][first : first + block] for t in waiting])
        owner = np.concatenate([np.full(len(ranked[t][first : first + block]), t) for t in waiting])
        ok, _ = passes(robot, pairs, q, owner, positions, rotations)
        for t, row in zip(owner[ok], q[ok]):
            chosen.setdefault(int(t), row)  # rows are in order of falling margin
        print(f"  candidates {first + 1} to {first + block}: {len(waiting)} targets tried, {len(chosen)} of {len(found)} settled", flush=True)
    for t in found:
        if t in chosen or not len(ranked.get(t, [])):
            continue
        q = ranked[t]
        arm = np.c_[np.rad2deg(q[:, 10:]), np.zeros(len(q))]
        gap = leg_clearance(robot, robot.placements(arm, q[:, :10])[0])
        q = q[(gap >= LEG_GAP).all(axis=(1, 2))]
        ok, _ = passes(robot, pairs, q, np.full(len(q), t), positions, rotations) if len(q) else (np.zeros(0, bool), None)
        if ok.any():
            chosen[t] = q[np.flatnonzero(ok)[0]]
        print(f"  target {t}: {len(found[t])} converged, {len(ranked[t])} with margin and floor, {len(q)} with the leg gap, {int(ok.sum())} pass", flush=True)
    for t in found:
        if t not in chosen and not len(ranked.get(t, [])):
            print(f"  target {t}: {len(found[t])} converged, none with margin and floor")
    return chosen


def search(robot, pairs, stored, starts):
    """Keep the stored postures that pass and search the others.

    Returns the postures, the pass flag of each and the flag of the stored ones kept.
    A target with no passing posture keeps its stored row.
    """
    frames = chain_frames(robot)
    positions, rotations = pose_test.make_targets()
    everything = np.arange(len(stored))
    kept, _ = passes(robot, pairs, stored, everything, positions, rotations)
    print(f"Stored postures that pass every check: {kept.sum()} of {len(stored)}")

    missing = np.flatnonzero(~kept).tolist()
    result = stored.copy()
    solved = kept.copy()
    if missing:
        found = candidates(robot, frames, pairs, missing, positions, rotations, starts)
        print(f"Converged candidates: {sum(len(q) for q in found.values())} for {len(missing)} targets")
        for t, row in choose(robot, pairs, found, positions, rotations).items():
            result[t], solved[t] = row, True
    ok, margin = passes(robot, pairs, result, everything, positions, rotations)
    assert (ok == solved).all()
    print(f"Front {ok[:64].sum()}/64. Broad {ok[64:].sum()}/32. No passing posture for {np.flatnonzero(~ok).tolist()}")
    print(f"Smallest support margin of a passing posture {1000 * margin[ok].min():.2f} mm")
    return result, ok, kept


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--starts", type=int, default=128, help="starting postures per search")
    parser.add_argument("--dry-run", action="store_true", help="report without writing targets.csv")
    args = parser.parse_args()

    robot = robot_model()
    _, stored = read_targets()
    result, ok, kept = search(robot, list(collision_pairs(robot)), stored, args.starts)
    missing = np.flatnonzero(~kept).tolist()
    if args.dry_run:
        return
    with (HERE / "targets.csv").open() as stream:
        rows = list(csv.reader(stream))
    for t in missing:
        if ok[t]:
            rows[t + 1] = [str(t)] + [f"{value:.12f}".rstrip("0").rstrip(".") for value in result[t]]
    with (HERE / "targets.csv").open("w", newline="") as stream:
        csv.writer(stream, lineterminator="\n").writerows(rows)
    print(f"Rewrote {int(ok[missing].sum())} rows of targets.csv; kept {int(kept.sum())}")


if __name__ == "__main__":
    main()
