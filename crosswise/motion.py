"""Model-space poses, degrees. The cube stays on the floor until jaw contact."""
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
NEUTRAL = np.zeros(7)
# 200 mm forward of the torso, near the centerline. Gripper reference point.
HIGH = np.array([-15.95, 45., .01, 57.86, -5.05, 1.71, 0.])
PARK = np.array([-14.94244, 44.47507, -8.04716, -38.67846, -92.43304, -150.09347, 0.])
PAD_CENTER_MM = (-104., 23.)
ROOF_MM = 107.8909
PAD_RADIUS_MM = 5.
PAD_HEIGHT_MM = 4.
JAW_OPEN = 20.
JAW_CONTACT = 3.32761821
CUBE_SIDE = .020
CUBE_XY = (.18209751, -.0001019656)
CUBE_YAW = np.deg2rad(-2.7898)


def between(start, end):
    steps = max(1, int(np.ceil(np.max(np.abs(np.subtract(end, start))) / .5)))
    return np.linspace(start, end, steps + 1)


def pickup():
    ik = np.loadtxt(HERE / "pickup_ik.csv", delimiter=",", skiprows=1)[:, 1:]
    count = len(ik)
    approach = np.column_stack([ik[::-1], np.full(count, JAW_OPEN)])
    closing = np.column_stack([np.tile(ik[0], (20, 1)), np.linspace(JAW_OPEN, JAW_CONTACT, 21)[1:]])
    lifting = np.column_stack([ik[1:], np.full(count - 1, JAW_CONTACT)])
    return np.vstack([approach, closing, lifting]), count - 1, count + len(closing) - 1


def fold():
    return between(NEUTRAL, PARK)


def cube_frames(joints, contact, placements):
    yaw = CUBE_YAW
    cube = np.eye(4)
    cube[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    cube[:3, 3] = [*CUBE_XY, CUBE_SIDE / 2]
    result = np.repeat(cube[None], len(joints), axis=0)
    palm = placements["gripper"]
    result[contact:] = palm[contact:] @ np.linalg.inv(palm[contact]) @ cube
    return result
