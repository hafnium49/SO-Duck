"""Choose how much shorter the forearm and wrist links must be, from the load limit alone.

    OPENBLAS_NUM_THREADS=1 python hollow/cad/lengths.py [--starts 128] [--jobs 6]

Each combination moves the joints only; no part is drawn and the arm's own collisions
are not checked. Each worker needs about 2 GB of memory. For every target the posture
with the lowest loaded shoulder torque is taken among those that reach it inside the
joint ranges with the support margin and the leg gap. A target counts as covered when
that posture's shoulder torque is within the stock SO-101 baseline, empty and loaded.
The smallest shortening that covers at least 85 targets is the one the CAD has to pack.
"""

import argparse
import csv
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent), str(HERE.parents[1] / "coverage"), str(HERE.parents[1] / "crosswise")]

import params  # noqa: E402

FOREARM_MM = (40, 45, 50, 55)
WRIST_SIDE_MM = (40, 45, 50)
WRIST_AXIAL_MM = 12
COVERAGE_GATE = 85
COLUMNS = ("forearm", "wrist_side", "wrist_axial", "reached", "covered", "front", "broad", "shoulder_empty", "shoulder_loaded", "chosen")


def evaluate(forearm, wrist_side, wrist_axial, starts):
    """Coverage within the load limit for one shortening, joints only.

    reached counts targets with any passing posture. covered counts those whose lowest-load
    posture is within the stock limit; shoulder_empty and shoulder_loaded are the worst
    torques among them.
    """
    import assemble
    import loads
    import resolve
    from geometry import Robot
    from model import UPPER
    from replay import JOINTS, LEG_GAP, leg_clearance, pose_test

    frames = params.joint_frames(forearm, wrist_side, wrist_axial)
    path = HERE.parent / f"_lengths_{round(1000 * forearm)}_{round(1000 * wrist_side)}.xml"
    try:
        assemble.write_model(path, frames, assemble.estimated_masses(frames), assemble.display_meshes(), assemble.stored_ranges())
        robot = Robot(path, JOINTS, UPPER)
    finally:
        path.unlink(missing_ok=True)
    chain = resolve.chain_frames(robot)
    positions, rotations = pose_test.make_targets()
    limit = loads.stock_baseline()
    best = {}

    def torques(q):
        row = np.atleast_2d(q)
        return (
            float(loads.arm_torques(robot, row)[0, loads.SHOULDER]),
            float(loads.arm_torques(robot, row, params.PAYLOAD_KG)[0, loads.SHOULDER]),
        )

    def within(t):
        return t in best and best[t][0] <= limit[0] and best[t][1] <= limit[1]

    def search(targets, budget, depth):
        found = resolve.candidates(robot, chain, [], targets, positions, rotations, budget)
        for t in targets:
            q = found[t]
            if not len(q):
                continue
            ok, _ = resolve.passes(robot, [], q, np.full(len(q), t), positions, rotations, full=False)
            q = q[ok]
            if not len(q):
                continue
            loaded = loads.arm_torques(robot, q, params.PAYLOAD_KG)[:, loads.SHOULDER]
            order = np.argsort(loaded, kind="stable")
            for first in range(0, min(len(order), depth), 8):
                block = order[first : first + 8]
                arm = np.c_[np.rad2deg(q[block, 10:]), np.zeros(len(block))]
                clear = (leg_clearance(robot, robot.placements(arm, q[block, :10])[0]) >= LEG_GAP).all(axis=(1, 2))
                if clear.any():
                    found_loads = torques(q[block[np.flatnonzero(clear)[0]]])
                    if t not in best or found_loads[1] < best[t][1]:
                        best[t] = found_loads
                    break

    search(list(range(96)), starts, 64)
    search([t for t in range(96) if not within(t)], 1024, 2000)
    covered = [t for t in range(96) if within(t)]
    return dict(
        forearm=forearm,
        wrist_side=wrist_side,
        wrist_axial=wrist_axial,
        reached=len(best),
        covered=len(covered),
        front=sum(t < 64 for t in covered),
        broad=sum(t >= 64 for t in covered),
        shoulder_empty=round(max(best[t][0] for t in covered), 4),
        shoulder_loaded=round(max(best[t][1] for t in covered), 4),
        loads={str(t): [round(v, 3) for v in best[t]] for t in sorted(best)},
    )


def smallest_passing(rows, limit):
    """The row with the least total shortening that meets both load limits and the coverage gate."""
    passing = [
        row
        for row in rows
        if float(row["shoulder_empty"]) <= limit[0]
        and float(row["shoulder_loaded"]) <= limit[1]
        and int(row["covered"]) >= COVERAGE_GATE
    ]
    if not passing:
        return None
    return min(passing, key=lambda row: (round(float(row["forearm"]) + float(row["wrist_side"]), 6), float(row["forearm"])))


def _job(arguments):
    """Evaluate one combination, keeping the result in the cache directory if one is given."""
    import json

    *combination, cache = arguments
    file = None if cache is None else Path(cache) / "{:.0f}_{:.0f}_{:.0f}_{}.json".format(*(1000 * np.array(combination[:3])), combination[3])
    if file is not None and file.exists():
        return json.loads(file.read_text())
    row = evaluate(*combination)
    if file is not None:
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(json.dumps(row))
    return row


def main():
    import loads

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--starts", type=int, default=128)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--cache", help="directory that keeps finished combinations, so an interrupted run resumes")
    args = parser.parse_args()
    grid = [(f / 1000, w / 1000, WRIST_AXIAL_MM / 1000, args.starts, args.cache) for f in FOREARM_MM for w in WRIST_SIDE_MM]
    limit = loads.stock_baseline()
    # Combinations with the same total shortening form a tier. Tiers run in rising order
    # and the study stops after the first tier that holds a passing combination.
    tiers = sorted({round(f + w, 6) for f, w, *_ in grid})
    rows, chosen = [], None
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for total in tiers:
            rows += list(pool.map(_job, [job for job in grid if round(job[0] + job[1], 6) == total]))
            chosen = smallest_passing(rows, limit)
            if chosen is not None:
                break
    rows.sort(key=lambda row: (round(row["forearm"] + row["wrist_side"], 6), row["forearm"]))
    with (HERE / "length_study.csv").open("w", newline="") as stream:
        out = csv.DictWriter(stream, COLUMNS, lineterminator="\n")
        out.writeheader()
        for row in rows:
            out.writerow({**{key: row[key] for key in COLUMNS[:-1]}, "chosen": int(row is chosen)})
    print(f"Stock SO-101 limit: {limit[0]:.3f} N m empty, {limit[1]:.3f} N m loaded")
    for row in rows:
        mark = "  <- chosen" if row is chosen else ""
        print(
            f"forearm -{1000 * row['forearm']:.0f} mm, wrist -{1000 * row['wrist_side']:.0f} mm: reached {row['reached']}, "
            f"within the load limit {row['covered']} ({row['front']}/64, {row['broad']}/32), "
            f"worst of those {row['shoulder_empty']:.3f} / {row['shoulder_loaded']:.3f} N m{mark}"
        )
    if chosen is None:
        print("No combination passes. Stop and report this table.")


if __name__ == "__main__":
    main()
