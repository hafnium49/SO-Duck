"""Gravity loads on the arm servos, and the stock SO-101 arm's loads for comparison.

    python hollow/loads.py

Torques are static gravity torques in N m. They leave out motion, friction, grip force
and heat. The stock baseline is the load this servo is known to carry in practice.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from functools import lru_cache

import mujoco
import numpy as np
from scipy.optimize import minimize

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE / "cad"), str(HERE.parent / "coverage"), str(HERE.parent / "crosswise")]

import params  # noqa: E402

GRAVITY = 9.81
REFINED_STARTS = 6  # sampled poses from which the worst case is climbed
SHOULDER = 1  # column of shoulder pitch in arm_torques
STRIPPED = ("geom", "asset", "default", "actuator", "contact", "sensor", "equality", "tendon", "keyframe")


@lru_cache(maxsize=None)
def stock_baseline(samples=params.LOAD_SAMPLES, seed=0):
    """Worst shoulder-lift gravity torque of the stock SO-101 arm, empty and with the payload.

    The arm of collision/inputs/so101.xml stands on a table with its link inertials. The
    worst case is searched inside its joint ranges: random poses first, then a climb from
    the best of them, so the result does not depend on the sample. The payload is a point
    mass at the gripperframe site.
    """
    root = ET.parse(HERE.parent / "collision" / "inputs" / "so101.xml").getroot()
    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag in STRIPPED:
                parent.remove(child)
    for element in root.iter():
        element.attrib.pop("class", None)
        element.attrib.pop("childclass", None)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    shoulder = model.joint("shoulder_lift").dofadr[0]
    site = model.site("gripperframe").id
    weight = np.array([0, 0, params.PAYLOAD_KG * GRAVITY])
    low, high = model.jnt_range.T
    rng = np.random.default_rng(seed)
    jacobian = np.zeros((3, model.nv))

    def torques(pose):
        data.qpos[:] = pose
        mujoco.mj_forward(model, data)
        mujoco.mj_jacSite(model, data, jacobian, None, site)
        torque = data.qfrc_bias[shoulder]
        return abs(torque), abs(torque + jacobian[:, shoulder] @ weight)

    poses = low + (high - low) * rng.random((samples, model.nq))
    sampled = np.array([torques(pose) for pose in poses])
    worst = []
    for column in (0, 1):
        best = sampled[:, column].max()
        for start in poses[np.argsort(-sampled[:, column])[:REFINED_STARTS]]:
            found = minimize(lambda pose: -torques(pose)[column], start, method="L-BFGS-B", bounds=list(zip(low, high)))
            best = max(best, -found.fun)
        worst.append(float(best))
    return tuple(worst)


def arm_torques(robot, q, payload_kg=0.0):
    """Absolute gravity torque at the six arm joints, shape (n, 6), for postures q of shape (n, 16)."""
    from replay import JOINTS

    model, data = robot.model, robot.data
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    dofs = [model.joint(name).dofadr[0] for name in JOINTS]
    site = model.site("gripperframe").id
    weight = np.array([0, 0, payload_kg * GRAVITY])
    jacobian = np.zeros((3, model.nv))
    torques = np.zeros((len(q), 6))
    for i, row in enumerate(np.asarray(q)):
        robot.placements(np.c_[np.rad2deg(row[None, 10:16]), [0.0]], row[None, :10])
        mujoco.mj_forward(model, data)
        mujoco.mj_jacSite(model, data, jacobian, None, site)
        torques[i] = data.qfrc_bias[dofs] + jacobian[:, dofs].T @ weight
    return np.abs(torques)


def main():
    import model as hollow_model
    import postures

    empty, loaded = stock_baseline()
    grams = round(1000 * params.PAYLOAD_KG)
    print(f"Stock SO-101 shoulder, worst pose: {empty:.3f} N m empty, {loaded:.3f} N m with {grams} g")
    robot = hollow_model.robot_model()
    q, _ = postures.read_targets()
    groups = (
        ("covered", [t for t in range(96) if t not in postures.UNREACHED + postures.OVER_LIMIT]),
        ("reached but over the limit", list(postures.OVER_LIMIT)),
    )
    for name, targets in groups:
        without = arm_torques(robot, q[targets])[:, SHOULDER]
        carrying = arm_torques(robot, q[targets], params.PAYLOAD_KG)[:, SHOULDER]
        print(
            f"Hollow arm shoulder over the {len(targets)} targets {name}: worst {without.max():.3f} N m empty "
            f"and {carrying.max():.3f} N m with {grams} g; median {np.median(without):.3f} and {np.median(carrying):.3f}"
        )
        if name != "covered":
            for t, a, b in zip(targets, without, carrying):
                print(f"  target {t}: {a:.3f} N m empty, {b:.3f} N m with {grams} g")
    print("Worst torque at each arm joint over the covered targets, with the payload (N m):")
    worst = arm_torques(robot, q[groups[0][1]], params.PAYLOAD_KG).max(axis=0)
    print("  " + ", ".join(f"{joint} {value:.3f}" for joint, value in zip(("shoulder yaw", "shoulder pitch", "elbow", "forearm roll", "wrist flex", "wrist roll"), worst)))


if __name__ == "__main__":
    main()
