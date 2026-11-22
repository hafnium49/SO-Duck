# Validation status — distinguish simulation from hardware

| Level | Status | Evidence |
|---|---|---|
| Source inventory, checksums, CSV schema | Locally checkable | `scripts/verify_study.py`, `docs/SHA256SUMS.txt` |
| Kinematic FK and Jacobian differentiation | Reproducible computation, not formal proof | `search/arm_model.py`, `search/checks.py` |
| Sampled full-body Jacobian condition metrics | Computational, finite samples | `search/*.csv` |
| End-effector 6D pose IK | Computed for finite target banks | `study/pose_test.py`, `study/poses_*.csv` |
| Nominal center-of-mass support margin | Static geometric approximation | `legs/`, `study/` |
| Endpoint triangle-mesh intersection | Finite pair/pose screen only | `collision/` program and CSVs |
| Forearm bearing / servo CAD mating | Design-stage geometry | `cad/` STEP/STL and `Mount_fit.md` |
| Physical build and mounted tolerance measurement | **Not demonstrated** | None |
| Motor thermal behavior, real gear ratios / speed | **Not demonstrated** | None |
| Dynamic locomotion while manipulating and fall recovery | **Not demonstrated** | None |
| Continuous path collision avoidance | **Not demonstrated** | None |

**Do not present a 95th-percentile Jacobian result as a global maximum.** The chosen 150 mm translational task-space normalization and equal-rate metric are assumptions, not universal tolerances.

**Do not interpret foot replants as feasible gait transitions.** The study compares independent statically valid endpoints from multiple assumed foot placements.

**Do not interpret computed gravity torque as validated servo continuous capacity.** The servo STS3215 model, calibration, speed, voltage and thermal envelope remain an engineering risk.
