"""Donor triangle meshes and rigid-body placements in the planted-feet frame."""

from itertools import combinations
import sys
from xml.etree import ElementTree as ET
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
import mujoco
import numpy as np
import trimesh
from build import HERE, ROOT, JOINTS, legs

sys.path[:0] = [str(ROOT / "collision"), str(ROOT / "study")]
import screen
import pose_test

LEG_NAMES = tuple(f"{s}_{j}" for s in legs.SIDES for j in legs.NAMES)
ARM_BODIES = (
    "so101_base",
    "shoulder",
    "upper_arm",
    "lower_arm",
    "wrist",
    "gripper",
    "moving_jaw_so101_v1",
)
SHEETS = "neck_pitch_assembly"


class Robot:
    def __init__(self):
        xml = HERE / "robot.xml"
        tree = ET.parse(xml)
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.names = LEG_NAMES + JOINTS
        self.indices = [self.model.joint(j).qposadr[0] for j in self.names]
        self.mesh = {}
        self.extreme = {}
        self.handles = {}
        self.body = {}
        self.lib = screen.collision_api()
        assets = {e.get("name"): e for e in tree.findall("asset/mesh")}
        for b in tree.iter("body"):
            meshes = []
            for g in b.findall("geom"):
                if g.get("class") == "collision":
                    continue
                if g.get("mesh"):
                    a = assets[g.get("mesh")]
                    v = screen.read_stl(HERE / a.get("file")) * np.fromstring(
                        a.get("scale", "1 1 1"), sep=" "
                    )
                elif g.get("type") == "cylinder":
                    r, half = np.fromstring(g.get("size"), sep=" ")
                    v = trimesh.creation.cylinder(
                        radius=r, height=2 * half, sections=40
                    ).triangles
                else:
                    continue
                meshes.append(screen.move(legs.rest(g), v))
            if not meshes:
                continue
            n = b.get("name")
            v = np.concatenate(meshes)
            self.mesh[n] = v
            flat = v.reshape(-1, 3)
            self.extreme[n] = flat[ConvexHull(flat).vertices]
            vv = np.ascontiguousarray(v * 1000)
            self.handles[n] = self.lib.load(vv.ctypes.data, len(vv))
            self.body[n] = self.model.body(n).id
        self.pairs = list(self.collision_pairs())
        self.data.qpos[2] = pose_test.W0[2, 3]
        self.data.qpos[self.indices[:10]] = pose_test.REF.ravel()
        mujoco.mj_kinematics(self.model, self.data)

    def collision_pairs(self):
        hw = [n for n in self.mesh if "_bolt_" in n]
        hybrid = {"adapter", *hw, *ARM_BODIES}
        for a, b in combinations(self.mesh, 2):
            pair = {a, b}
            if not pair & hybrid:
                continue
            if a in hw and b in hw:
                continue  # fixed relative, checked at assembly
            if "adapter" in pair and (pair & set(hw) or "so101_base" in pair):
                continue
            if any(n.startswith("sheet_bolt_") for n in pair) and SHEETS in pair:
                continue
            if any(n.startswith("base_bolt_") for n in pair) and "so101_base" in pair:
                continue
            if (
                a in ARM_BODIES
                and b in ARM_BODIES
                and abs(ARM_BODIES.index(a) - ARM_BODIES.index(b)) == 1
            ):
                continue
            yield a, b
        chains = [
            [
                self.model.body(
                    self.model.jnt_bodyid[self.model.joint(s + "_" + j).id]
                ).name
                for j in legs.NAMES
            ]
            for s in legs.SIDES
        ]
        for a in chains[0]:
            for b in chains[1]:
                yield a, b
        for chain in chains:
            for i, a in enumerate(chain):
                for b in chain[i + 2 :]:
                    yield a, b
                yield a, SHEETS
            for a in chain[3:]:
                yield a, "trunk_assembly"

    def placements(self, q):
        transforms = []
        com = []
        for row in np.atleast_2d(q):
            t = pose_test.ANCHOR[0] @ np.linalg.inv(legs.leg_fk("left", row[:5])[0])
            self.data.qpos[:3] = t[:3, 3]
            self.data.qpos[3:7] = np.roll(Rotation.from_matrix(t[:3, :3]).as_quat(), 1)
            self.data.qpos[self.indices] = row
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)
            f = np.broadcast_to(np.eye(4), (self.model.nbody, 4, 4)).copy()
            f[:, :3, :3] = self.data.xmat.reshape(-1, 3, 3)
            f[:, :3, 3] = self.data.xpos
            transforms.append(f)
            com.append(self.data.subtree_com[self.model.body("base").id].copy())
        full = np.stack(transforms)
        return {n: full[:, i] for n, i in self.body.items()}, np.array(com)

    def hits(self, a, b, f):
        rel = np.ascontiguousarray(np.linalg.inv(f[b]) @ f[a])
        rel[:, :3, 3] *= 1000
        hits = np.zeros(len(rel), dtype="uint8")
        self.lib.collisions(
            self.handles[a],
            self.handles[b],
            rel.ctypes.data,
            hits.ctypes.data,
            len(hits),
        )
        return hits.astype(bool)

    def check(self, q):
        f, c = self.placements(q)
        hit = np.zeros(len(c), bool)
        for a, b in self.pairs:
            hit |= self.hits(a, b, f)
        gap = np.min(
            [
                np.min(f[n][:, 2, :3] @ pts.T, axis=1) + f[n][:, 2, 3]
                for n, pts in self.extreme.items()
                if not n.startswith("foot_assembly")
            ],
            axis=0,
        )
        return f, pose_test.GROUND.margin(c), gap, hit
