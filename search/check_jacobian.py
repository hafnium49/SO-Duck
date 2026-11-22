"""Numerical differential test: kinematic Jacobian vs 6D central differences."""
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "search"))
from arm_model import fk, frames

def main():
    q = np.deg2rad([8, -12, 34, 7, 49, -15])
    transforms = frames(90, 0, 90, True)
    pose, jac = fk(q, transforms)
    singular = np.linalg.svd(jac, compute_uv=False)
    assert singular[-1] > .15, ("Rank near singular:", singular[-1])
    step = 1e-6
    worst = 0.0
    for i in range(6):
        delta = np.eye(6)[i] * step
        hi = fk(q + delta, transforms)[0]
        lo = fk(q - delta, transforms)[0]
        numeric = np.r_[
            (hi[:3, 3] - lo[:3, 3]) / (2 * step * .15),
            Rotation.from_matrix(hi[:3, :3] @ lo[:3, :3].T).as_rotvec() / (2 * step)
        ]
        worst = max(worst, float(np.max(np.abs(jac[:, i] - numeric))))
    assert worst < 1e-5, f"Central-difference mismatch: {worst}"
    print("PASS: 6-DoF rank and Jacobian finite differences", worst)

if __name__ == "__main__":
    main()
