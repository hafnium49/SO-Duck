"""How far the arm is from its zero pose if one horn is fitted one tooth off.

    python hollow/horn_check.py

The STS3215 output has 25 teeth, so a horn can sit 14.4 degrees off. With every servo
at its middle position the arm must stand in the zero pose of robot.xml. This prints,
for each joint, how far a named point moves when that joint alone is one tooth off:
the figures of the assembly checks in BUILD.md. Run from the repository root.
"""

import numpy as np

from model import robot_model
from replay import FINGER, JOINTS, screen

TOOTH_DEG = 360 / 25
JAW_OPEN_DEG = 90.0
LABELS = ("shoulder yaw", "shoulder pitch", "elbow", "forearm roll", "wrist flex", "wrist roll", "gripper")


def points(robot, angles):
    """The tool point and the tip of the moving jaw in the trunk frame, millimetres."""
    frames, _ = robot.placements(np.atleast_2d(angles))
    trunk = np.linalg.inv(frames["trunk_assembly"][0])
    tool = (trunk @ frames["gripper"][0] @ screen.a.TCP)[:3, 3]
    jaw = robot.mesh[FINGER].reshape(-1, 3)
    tip = jaw[np.argmin(jaw[:, 1])]  # the jaw runs along -y of its body
    return 1000 * tool, 1000 * (trunk @ frames[FINGER][0] @ np.r_[tip, 1.0])[:3]


def table(robot):
    """For each joint: its label, the point to measure, and how far one tooth moves it, in millimetres."""
    rows = []
    for k, label in enumerate(LABELS):
        start = np.zeros(7)
        point, use_tip = "tool point", False
        if label == "wrist roll":  # the tool point and the closed jaw lie on the roll axis
            start[6] = JAW_OPEN_DEG
            point, use_tip = f"tip of the moving jaw, opened {JAW_OPEN_DEG:.0f} degrees", True
        elif label == "gripper":
            point, use_tip = "tip of the moving jaw", True
        off = start.copy()
        off[k] += TOOTH_DEG
        distance = np.linalg.norm(points(robot, off)[use_tip] - points(robot, start)[use_tip])
        rows.append((label, point, float(distance)))
    return rows


def zero_pose(robot):
    """Joint origins of the zero pose in the trunk frame, millimetres, and the tool point."""
    frames, _ = robot.placements(np.zeros((1, 7)))
    trunk = np.linalg.inv(frames["trunk_assembly"][0])
    origins = {}
    for joint in JOINTS:
        body = robot.model.jnt_bodyid[robot.model.joint(joint).id]
        name = next(n for n in frames if robot.body[n] == body and n in robot.mesh)
        origins[joint] = 1000 * (trunk @ frames[name][0])[:3, 3]
    return origins, points(robot, np.zeros(7))[0]


def main():
    robot = robot_model()
    origins, tool = zero_pose(robot)
    print("Zero pose, trunk frame (x forward, y left, z up), mm")
    for joint, origin in origins.items():
        print(f"  {joint} axis at {origin[0]:.0f}, {origin[1]:.0f}, {origin[2]:.0f}")
    print(f"  tool point at {tool[0]:.0f}, {tool[1]:.0f}, {tool[2]:.0f}")
    print(f"One tooth is {TOOTH_DEG:.1f} degrees")
    for label, point, distance in table(robot):
        print(f"  {label}: the {point} moves {distance:.0f} mm")


if __name__ == "__main__":
    main()
