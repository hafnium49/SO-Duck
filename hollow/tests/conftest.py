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
