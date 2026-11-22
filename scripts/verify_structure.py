#!/usr/bin/env python3
"""Dependency-free, conservative repository smoke check (not a physics test)."""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REQUIRED = [
    'README.md', 'SO_Duck_pose_explorer.html',
    'cad/Case_mounted_forearm.step', 'cad/build_forearm.py',
    'search/arm_model.py', 'search/scan_robot.py',
    'legs/legs.py', 'study/pose_test.py',
    'collision/screen.py', 'collision/inputs/license-duck.txt',
    'collision/inputs/license-so.txt',
]
missing = [p for p in REQUIRED if not (ROOT / p).exists()]
if missing:
    raise SystemExit('Missing project inputs: ' + ', '.join(missing))
count = 0
for path in ROOT.rglob('*.py'):
    if any(x in path.parts for x in ('.venv', 'venv')):
        continue
    ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    count += 1
print(f'OK: {len(REQUIRED)} required files and {count} Python source files parse.')
