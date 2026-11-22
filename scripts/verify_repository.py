"""Source, documentation, and CAD smoke checks; NOT hardware validation."""
import ast
from pathlib import Path
root = Path(__file__).resolve().parents[1]
required = ["README.md", "SO_Duck_pose_explorer.html",
            "cad/Case_mounted_forearm.step", "search/arm_model.py",
            "search/scan_arm.py", "search/check_jacobian.py",
            "docs/ARCHITECTURE.md", "docs/VALIDATION.md", "docs/PROVENANCE.md",
            "docs/WHOLE_BODY_STUDY.md"]
missing = [p for p in required if not (root / p).is_file()]
assert not missing, f"Missing required files: {missing}"
for p in root.rglob("*.py"):
    ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
step = (root / "cad/Case_mounted_forearm.step").read_text(encoding="utf-8")
assert step.startswith("ISO-10303-21;") and "END-ISO-10303-21;" in step
print(f"PASS: {len(required)} required files and Python syntax")
