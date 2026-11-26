"""Every dimension, clearance and mass rule of the buildable hollow arm.

Lengths are metres here and in the model, millimetres in the CAD. The joint frames
below were read once from the model reconstructed from the viewer (hollow/model.py)
and are the arm's kinematics before any shortening.
"""

from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DONOR_COMMIT = "141aea7bcac2744d92998e30268c413202a56f96"  # TheRobotStudio/SO-ARM100

# Joint name: position (m) and quaternion (w, x, y, z) of the joint's body in its parent.
FRAME = {
    "shoulder_yaw": ((0.001000000011, 9.999971545e-05, 0.177000007), (1.0, -6.177711678e-10, 2.242059479e-10, 2.750608807e-06)),
    "shoulder_pitch": ((0.0303991998, -0.01827775876, 0.05419999652), (0.6678264187, -0.6678264172, 0.2323959454, 0.2323959455)),
    "forearm_pitch": ((-0.0479319306, -0.1056339645, -4.13096353e-08), (-0.4354342695, 2.506430322e-10, -6.718601977e-10, 0.9002205268)),
    "forearm_roll": ((0.1400000007, -0.02600000073, 0.01819999992), (0.5000003498, 0.4999996508, 0.5000003493, 0.4999996502)),
    "wrist_flex": ((-0.08000000012, -0.01870000033, 0.06199999952), (-0.4999985457, 0.4999985454, 0.5000014539, 0.500001455)),
    "wrist_roll": ((7.562153178e-10, -0.06110000031, 0.01809999854), (0.01721081203, -0.01721081204, 0.7068972967, 0.7068972965)),
    "gripper": ((0.0202, 0.0188, -0.0234), (0.707107, 0.707107, -1.85362e-08, 1.85362e-08)),
}

# Shortening of the two links, metres. Chosen by hollow/cad/lengths.py from the load limit.
FOREARM_SHORTEN = 0.040  # elbow axis to forearm-roll joint, along the forearm
WRIST_SIDE_SHORTEN = 0.040  # wrist-flex axis toward the roll axis
WRIST_AXIAL_SHORTEN = 0.012  # wrist-flex axis along the roll axis
FOREARM_RANGE = (0.040, 0.055)
WRIST_SIDE_RANGE = (0.040, 0.050)
WRIST_AXIAL_MAX = 0.012

MOVING_CLEARANCE_MM = 1.5  # between parts that move relative to each other
MIN_WALL_MM = 1.6
BED_MM = 180.0
DENSITY = 1240.0  # kg/m3; custom parts count at full solid volume
SERVO_KG = 0.055
PAYLOAD_KG = 0.1  # at the gripperframe site
LOAD_SAMPLES = 20000


def joint_frames(forearm=None, wrist_side=None, wrist_axial=None):
    """Joint frames with the shortening applied. Arguments default to the chosen values."""
    forearm = FOREARM_SHORTEN if forearm is None else forearm
    wrist_side = WRIST_SIDE_SHORTEN if wrist_side is None else wrist_side
    wrist_axial = WRIST_AXIAL_SHORTEN if wrist_axial is None else wrist_axial
    frames = {name: (np.array(position), np.array(quaternion)) for name, (position, quaternion) in FRAME.items()}
    frames["forearm_roll"][0][0] -= forearm
    frames["wrist_flex"][0][0] += wrist_side
    frames["wrist_flex"][0][2] -= wrist_axial
    return frames


def donor(name):
    """An SO-101 donor solid in millimetres, as published."""
    import cadquery as cq

    shapes = cq.importers.importStep(str(HERE / "donors" / f"{name}.step")).vals()
    solids = [solid for shape in shapes for solid in shape.Solids()]
    return max(solids, key=lambda solid: solid.Volume())
