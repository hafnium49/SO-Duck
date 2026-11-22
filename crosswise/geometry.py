"""Replay the shipped MJCF using the full STL surfaces for interference tests."""
from collections import defaultdict
from itertools import combinations
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(ROOT / "collision"), str(ROOT / "study")]
import screen
import pose_test

ARM = ("neck_pitch", "head_pitch", "head_yaw", "forearm_pitch", "wrist_flex", "wrist_roll")
FINGER = "moving_jaw_so101_v1"
DISTAL = ("carrier", "yaw_case", "pitch_case", "forearm", "wrist", "gripper", FINGER)


class Robot:
    def __init__(self, xml=HERE / "robot.xml", arm=ARM, split_bodies=()):
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        tree = ET.parse(xml)
        assets = {m.get("name"): m for m in tree.findall("./asset/mesh")}
        groups = defaultdict(list)
        self.body = {}
        split = {"neck_yaw_assembly", *split_bodies}
        for body in tree.iter("body"):
            for geom in body.findall("geom[@mesh]"):
                mesh = assets[geom.get("mesh")]
                scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                part = body.get("name")
                if part in split and geom.get("name"):
                    part = geom.get("name")
                triangles = screen.read_stl(xml.parent / mesh.get("file")) * scale
                groups[part].append(screen.move(screen.legs.rest(geom), triangles))
                self.body[part] = self.model.body(body.get("name")).id
        self.mesh = {k: np.concatenate(v) for k, v in groups.items()}
        self.extreme = {}
        self.lib = screen.collision_api()
        self.handles = {}
        for name, tris in self.mesh.items():
            vertices = tris.reshape(-1, 3)
            self.extreme[name] = vertices[ConvexHull(vertices).vertices]
            self.load(name, tris)
        self.arm_indices = [self.model.joint(j).qposadr[0] for j in (*arm, "gripper")]
        self.data.qpos[2] = pose_test.W0[2, 3]
        self.leg_indices = [self.model.joint(f"{side}_{joint}").qposadr[0]
                            for side in screen.legs.SIDES for joint in screen.legs.NAMES]
        self.data.qpos[self.leg_indices] = pose_test.REF.ravel()

    def load(self, name, tris):
        mm = np.ascontiguousarray(tris * 1000)
        self.handles[name] = self.lib.load(mm.ctypes.data, len(mm))

    def placements(self, joints, leg_angles=None):
        frames = []
        com = []
        for index, row in enumerate(np.deg2rad(joints)):
            legs = pose_test.REF.ravel() if leg_angles is None else leg_angles[index]
            self.data.qpos[self.leg_indices] = legs
            world = pose_test.W0 if leg_angles is None else (
                pose_test.ANCHOR[0] @ np.linalg.inv(screen.legs.leg_fk("left", legs[:5])[0]))
            self.data.qpos[:3] = world[:3, 3]
            self.data.qpos[3:7] = np.roll(Rotation.from_matrix(world[:3,:3]).as_quat(), 1)
            self.data.qpos[self.arm_indices] = row
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)
            frame = np.tile(np.eye(4), (self.model.nbody, 1, 1))
            frame[:, :3, :3] = self.data.xmat.reshape(-1, 3, 3)
            frame[:, :3, 3] = self.data.xpos
            frames.append(frame)
            com.append(self.data.subtree_com[self.model.body("base").id].copy())
        frames = np.asarray(frames)
        return {name: frames[:, idx] for name, idx in self.body.items()}, np.array(com)

    def hits(self, a, b, frames):
        rel = np.ascontiguousarray(np.linalg.inv(frames[b]) @ frames[a])
        rel[:, :3, 3] *= 1000
        hit = np.zeros(len(rel), dtype="uint8")
        self.lib.collisions(self.handles[a], self.handles[b], rel.ctypes.data, hit.ctypes.data, len(hit))
        return hit.astype(bool)

    def pairs(self):
        # Same rigid body, donor joint mating faces, and the yaw case to its
        # parent bracket are checked by CAD assembly fit or inherited from the donor.
        skip = {frozenset(p) for p in [
            ("park_pad", "trunk_assembly"), ("yaw_case", "head_pitch_to_yaw"), ("forearm", "wrist"),
            ("wrist", "gripper"), ("gripper", FINGER)]}
        for a, b in combinations(self.mesh, 2):
            if (a in DISTAL or b in DISTAL or "park_pad" in (a, b)
                    or {a,b} == {"head_pitch_to_yaw", "trunk_assembly"}):
                if self.body[a] != self.body[b] and frozenset((a, b)) not in skip:
                    yield a, b

    def lowest(self, name, frames):
        z = frames[name][:, 2, :]
        return (z[:, :3] @ self.extreme[name].T).min(axis=1) + z[:, 3]
