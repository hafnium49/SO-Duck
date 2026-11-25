# Reproducing the 39 MB study

The repository can restore the full retained original study tree from Git-tracked lossless asset chunks: the modeling programs, simulation XML inputs, original mesh assets, generated CAD, finite target banks and CSV outputs. Machine-specific build logs, local caches (`__pycache__`) and obsolete draft notes are not research inputs.

## Requirements

- Python with NumPy and SciPy for forward kinematics, Jacobian scans, contact/mass and IK optimization. For exact environments inspect `requirements-core.txt` and the scripts; the experiment was not locked to a full historical pip environment.
- CadQuery and its OpenCascade implementation to regenerate the STEP/STL forearm from `cad/build_forearm.py` (optional).
- VTK Python bindings, C++17 compiler and Python C++ extension toolchain to rebuild the collision acceleration helper (optional). The cached collision CSVs are included.
- A local browser to inspect `study/legacy_pose_explorer.html`. This displays saved poses, not a physics runtime. `Forward_neck_and_wrist.glb` is model geometry, not a joint control model.

## Reproduction order

From repository root:

```sh
python scripts/restore_assets.py
python scripts/verify_study.py
python search/checks.py
python search/scan_arm.py
python search/scan_robot.py
python search/refine_clock.py
python study/pose_test.py O90 --two --clock 90 --starts 24 --steps 100
python study/pose_test.py Replant22 --two --clock 90 --starts 24 --steps 100 --replant 0.22
python study/pose_test.py Replant27 --two --clock 90 --starts 24 --steps 100 --replant 0.27
python collision/build_bvh.py
python collision/screen.py study/poses_O90.csv study/poses_Replant22.csv study/poses_Replant27.csv
python legs/torque.py
python cad/build_forearm.py
```

**Caution:** several study programs write CSV files into the current directory and may replace the committed snapshots. Run regeneration in a temporary clone or working tree if exact preserved results matter. `scripts/verify_study.py` validates the committed inventory without modifying data. The C++ helper is built from source; no prebuilt native binary is committed.

## Testing levels

1. **Repository integrity and schemas** — file hashes, required paths, CAD/XML header signatures, known CSV headers; run `python scripts/verify_study.py` (no NumPy required).
2. **Algorithmic spot checks** — run `python search/checks.py` with numerical dependencies (Jacobian finite differences and constraints).
3. **Study regeneration** — run optimization, pose-screen and torque scripts, compare generated outputs within numerical tolerance; stochastic initializations and package versions can change some solutions.
4. **Physical validation, not present** — fit and bearing alignment, electronics, servo model/gear ratio and thermal tests, biped balance under a moving load, safe fall recovery, manipulability with a calibrated gripper.

## Terminology

- `two_part` in the analysis means *two separately orientable axes*; the mechanical concept still has **one custom rigid forearm**.
- `replant` denotes a solved alternate end stance, not a feasible robot stepping trajectory.
- `collision_checked` and `hit_count=0` mean *no intersections among tested triangle pairs at that endpoint*, not a guarantee of clearance along motion.
- Percentile Jacobian gains are for *a specified normalized twist and rate metric*. They do not establish universal conditioning bounds or payload.
