"""Replay the original 20 mm test cube at the same location and yaw."""

from itertools import product
import numpy as np
from scipy.spatial import ConvexHull
from model import HERE, screen

FINGER = "moving_jaw_so101_v1"
CUBE_SIDE = 0.020
CUBE_XY = (0.18209751, -0.0001019656)
CUBE_YAW = np.deg2rad(-2.7898)


def pickup():
    ik = np.loadtxt(HERE / "pickup.csv", delimiter=",", skiprows=1)
    n = len(ik)
    opened, contact = np.deg2rad([20.0, 3.32761821])
    descent = np.column_stack([ik[::-1], np.full(n, opened)])
    close = np.column_stack(
        [np.tile(ik[0], (20, 1)), np.linspace(opened, contact, 21)[1:]]
    )
    lift = np.column_stack([ik[1:], np.full(n - 1, contact)])
    return np.vstack((descent, close, lift)), n - 1, n + len(close) - 1


def cube_frames(joints, contact, placements):
    yaw = CUBE_YAW
    cube = np.eye(4)
    cube[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    cube[:3, 3] = [*CUBE_XY, CUBE_SIDE / 2]
    result = np.repeat(cube[None], len(joints), axis=0)
    palm = placements["gripper"]
    result[contact:] = palm[contact:] @ np.linalg.inv(palm[contact]) @ cube
    return result


def cheek_points(vertices):
    """Clip triangles to a 20 mm slab in cube y and z; retain actual contact features."""
    half = CUBE_SIDE / 2
    selected = vertices[
        (vertices[:, :, 1:].max(1) >= -half).all(1)
        & (vertices[:, :, 1:].min(1) <= half).all(1)
    ]
    points = []
    for triangle in selected:
        polygon = list(triangle)
        for axis, sign in product((1, 2), (-1, 1)):
            clipped = []
            for i, end in enumerate(polygon):
                start = polygon[i - 1]
                u, v = half - sign * start[axis], half - sign * end[axis]
                if (u >= 0) != (v >= 0):
                    clipped.append(start + u / (u - v) * (end - start))
                if v >= 0:
                    clipped.append(end)
            polygon = clipped
        points.extend(polygon)
    return np.array(points)


def check_cube(robot, joints, grasp, contact, frames):
    cube = cube_frames(joints, contact, frames)
    half = CUBE_SIDE / 2
    corners = np.array(list(product((-half, half), repeat=3)))
    vv = np.ascontiguousarray(1000 * corners[ConvexHull(corners).simplices])
    robot.handles["cube"] = robot.lib.load(vv.ctypes.data, len(vv))
    for part in robot.mesh:
        if part not in ("gripper", FINGER):
            assert not robot.hits(part, "cube", {**frames, "cube": cube}).any(), part
    palm = np.linalg.inv(cube) @ frames["gripper"]
    finger = np.linalg.inv(cube) @ frames[FINGER]
    # On each cheek find the deepest vertex of the true triangle/footprint intersection.
    # A cheek can be wholly above the cube early in the descent.
    widths = []
    for frame_a, frame_b in zip(palm, finger):
        a = cheek_points(screen.move(frame_a, robot.mesh["gripper"]))
        b = cheek_points(screen.move(frame_b, robot.mesh[FINGER]))
        left = a[:, 0].max() if len(a) else -np.inf
        right = b[:, 0].min() if len(b) else np.inf
        assert left <= -half + 0.00002 and right >= half - 0.00002
        widths.append(right - left)
    assert 0.040 < widths[grasp] < 0.042
    assert max(abs(np.array(widths[contact:]) - CUBE_SIDE)) < 0.00002
    lift = cube[-1, 2, 3] - cube[contact, 2, 3]
    assert abs(lift - 0.030) < 0.00002
    print(
        f"Cube {CUBE_SIDE*1000:.0f} mm, open gap {widths[grasp]*1000:.2f} mm, "
        f"closed gap {widths[contact]*1000:.2f} mm, assumed no-slip lift {lift*1000:.2f} mm"
    )
