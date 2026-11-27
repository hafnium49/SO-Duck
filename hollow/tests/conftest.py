"""Shared paths and fixtures for the hollow-arm tests. Run pytest from the repository root."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
for folder in ("study", "crosswise", "coverage", "hollow", "hollow/cad"):
    sys.path.insert(0, str(ROOT / folder))


@pytest.fixture(scope="session")
def root():
    return ROOT


@pytest.fixture(scope="session")
def robot():
    """The hollow-arm model with its meshes loaded."""
    import model

    if not (ROOT / "hollow" / "parts" / "stock" / "shoulder_yaw_case.stl").exists():
        import build_parts  # the stock meshes are derived, not committed

        build_parts.write_stock()
    return model.robot_model()


@pytest.fixture(scope="session")
def passing_q():
    """Stored postures of the covered targets: a passing posture within the load limit. Shape (n, 16)."""
    import numpy as np
    import postures

    q, _ = postures.read_targets()
    return np.delete(q, list(postures.UNREACHED) + list(postures.OVER_LIMIT), axis=0)
