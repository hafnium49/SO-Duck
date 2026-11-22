"""Retarget the study's 96 tool poses to the current arm and double-support stance."""
import csv
import sys
from pathlib import Path
from scipy.spatial.transform import Rotation

from geometry import ARM, HERE, Robot, np, pose_test, screen


def retarget(output):
    robot = Robot()
    limits = robot.model.jnt_range[[robot.model.joint(name).id for name in ARM]]
    frames = screen.a.frames(90, 0, 0, False)
    # Same deterministic search budget as the published nominal O90 bank.
    q, errors, _, _, _, _, _ = pose_test.solve(frames, starts=24, active=True, steps=100, arm_limits=limits)
    world, com = robot.placements(np.c_[np.rad2deg(q[:,10:]), np.zeros(len(q))], leg_angles=q[:,:10])
    # Confirm the tool and the right sole on the actual compiled model.
    tcp = world["gripper"] @ screen.a.TCP
    foot_name = robot.model.body(robot.model.jnt_bodyid[robot.model.joint("right_ankle").id]).name
    foot = world[foot_name] @ screen.legs.specs("right")[4]
    target_pos, target_rot = pose_test.make_targets()
    def angle_error(expected, measured):
        delta = expected @ measured.transpose(0, 2, 1)
        return np.rad2deg(Rotation.from_matrix(delta).magnitude())
    actual = np.column_stack([np.linalg.norm(tcp[:, :3, 3] - target_pos, axis=1)*1000,
                              angle_error(target_rot, tcp[:, :3, :3]),
                              np.linalg.norm(foot[:, :3, 3] - pose_test.ANCHOR[1, :3, 3], axis=1)*1000,
                              angle_error(pose_test.ANCHOR[1, :3, :3], foot[:, :3, :3])])
    assert np.allclose(actual, errors, atol=.0001)
    ok = (actual < [1.5, 1., .2, .15]).all(axis=1)
    margins = pose_test.GROUND.margin(com)
    reasons = [[] for _ in q]
    pairs = set(robot.pairs())
    # The study also screens the native legs when their joints participate in IK.
    legs = [[robot.model.body(robot.model.jnt_bodyid[robot.model.joint(f"{side}_{joint}").id]).name
             for joint in screen.legs.NAMES] for side in screen.legs.SIDES]
    pairs.update((left, right) for left in legs[0] for right in legs[1])
    for leg in legs:
        pairs.update((a,b) for i,a in enumerate(leg) for b in leg[i+2:])
        pairs.update((a,b) for a in leg for b in ("neck_pitch_assembly", "head_pitch_to_yaw"))
        pairs.update((a, "trunk_assembly") for a in leg[3:])
    for a,b in sorted(pairs):
        for i in np.flatnonzero(robot.hits(a,b,world)):
            reasons[i].append(f"{a}/{b}")
    ground = np.min([robot.lowest(n,world) for n in robot.mesh if not n.startswith("foot_assembly")],axis=0)
    passed = ok & (margins >= .005) & (ground >= -.0001) & np.array([not r for r in reasons])
    joint_names = [f"{side}_{joint}" for side in screen.legs.SIDES for joint in screen.legs.NAMES] + list(ARM)
    with output.open("w", newline="") as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow(["target", "bank", "pass", "pose_and_feet_ok", "tcp_error_mm", "tcp_error_deg",
                         "right_foot_error_mm", "right_foot_error_deg", "support_margin_mm", "floor_gap_mm",
                         "intersections"] + [n + "_rad" for n in joint_names])
        for i, angles in enumerate(q):
            writer.writerow([i, "front" if i<64 else "broad", int(passed[i]), int(ok[i]),
                             *actual[i].round(5), round(margins[i]*1000,4), round(ground[i]*1000,4),
                             "|".join(reasons[i]), *angles.round(10)])
    print(f"Nominal double support: front {sum(passed[:64])}/64, broad orientation {sum(passed[64:])}/32")


if __name__ == "__main__":
    retarget(Path(sys.argv[1]) if len(sys.argv)>1 else HERE / "nominal_targets.csv")
