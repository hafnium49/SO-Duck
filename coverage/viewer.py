"""Export an offline interactive view of the exact replay postures."""

import json
from itertools import product

from scipy.spatial import ConvexHull

from replay import (
    FINGER,
    HERE,
    np,
    pickup,
    pose_test,
    fold_path,
    read_targets,
    robot_model,
    screen,
    UNREACHED,
)
from export_explorer import simplify
from motion import CUBE_SIDE, cube_frames


def main():
    robot = robot_model()
    cube = np.array(list(product((-CUBE_SIDE / 2, CUBE_SIDE / 2), repeat=3)))
    geometry = {**robot.mesh, "cube": cube[ConvexHull(cube).simplices]}
    meshes = []
    for i, (name, triangles) in enumerate(geometry.items()):
        v, f = simplify(triangles, 0.0005 if name in ("park_pad", "cube") else 0.0028)
        if name in ("park_pad", "cube"):
            color = 4
        elif name.endswith("_link") or name == "shoulder_mount":
            color = 3
        elif name.endswith("_case"):
            color = 2
        elif name in ("gripper", FINGER):
            color = 1
        else:
            color = 0
        meshes.append([i, color, v.ravel().tolist(), f.ravel().tolist()])

    poses = []

    def add(mode, label, index, q, frames, com):
        tcp = frames["gripper"][index] @ screen.a.TCP
        poses.append(
            dict(
                mode=mode,
                label=label,
                x=(tcp[:3, 3] * 1000).round(1).tolist(),
                q=q[index].round(1).tolist(),
                m=round(1000 * pose_test.GROUND.margin(com[index]), 1),
                tcp=tcp[:3].round(8).tolist(),
                c=[
                    (
                        frames[n][index, :3].round(8).ravel().tolist()
                        if n in frames
                        else None
                    )
                    for n in geometry
                ],
            )
        )

    _, q = read_targets()
    arm = np.c_[np.rad2deg(q[:, 10:]), np.zeros(len(q))]
    frames, com = robot.placements(arm, q[:, :10])
    for i in range(len(q)):
        add(
            "Forward" if i < 64 else "Broad orientation",
            f"Target {i:02}" + (" (no passing posture)" if i in UNREACHED else ""),
            i,
            arm,
            frames,
            com,
        )
    path, grasp, contact = pickup()
    frames, com = robot.placements(path)
    frames["cube"] = cube_frames(path, contact, frames)
    for i, label in [
        (0, "Approach"),
        (grasp, "Open"),
        (contact, "Contact"),
        (-1, "Lifted 30 mm"),
    ]:
        add("Floor pickup", label, i, path, frames, com)
    path = fold_path()
    frames, com = robot.placements(path)
    for fraction, label in [
        (0, "Overhead"),
        (0.25, "25% folded"),
        (0.5, "50% folded"),
        (0.75, "75% folded"),
        (1, "Seated on rear pad"),
    ]:
        i = round(fraction * (len(path) - 1))
        add("Park and reach", label, i, path, frames, com)

    points = pose_test.GROUND.sole_points
    sole = points[ConvexHull(points[:, :2]).vertices].copy()
    sole[:, 2] = 0
    data = json.dumps(
        dict(meshes=meshes, poses=poses, sole=sole.round(6).tolist()),
        separators=(",", ":"),
    )
    template = (HERE.parent / "crosswise" / "explorer.template.html").read_text()
    replacements = {
        "SO-Duck · Crosswise pose explorer": "SO-Duck · Grasp coverage",
        "SO-Duck pose explorer": "SO-Duck grasp coverage",
        "Crosswise forearm connector · Open Duck Mini V2 and SO-101": "Six-axis arm · 96 planted-feet grasp targets",
        ">Arm motion<": ">Pose explorer<",
        ">Movement<": ">Target set or task<",
        ">Crosswise carrier<": ">New printed links<",
        ">Stock SO-101 arm<": ">SO-101 gripper<",
        "pose.q[3]": "pose.q[2]",
        "__POSE_DATA__": data,
    }
    for before, after in replacements.items():
        assert template.count(before) == 1, before
        template = template.replace(before, after)
    (HERE / "poses.html").write_text(template)


if __name__ == "__main__":
    main()
