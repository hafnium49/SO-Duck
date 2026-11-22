# Reproducibility manifest — initial public snapshot (2026-11-22)

The complete local whole-body seed produced for the project is larger than the initial GitHub publication. **No missing files are represented as present**.

| Artifact | In this repo? | What it supports |
|---|---|---|
| Custom one-piece forearm STEP | Yes: cad/Case_mounted_forearm.step | Editable proposed hardware geometry |
| Offline viewer with saved sampled poses | Yes: SO_Duck_pose_explorer.html | Visual inspection, not physics |
| Six upper-pose-joint kinematics | Yes: search/arm_model.py | Analytic task Jacobian and forward kinematics |
| Central-difference Jacobian check | Yes: search/check_jacobian.py | Local numerical derivative consistency |
| Uniform joint-space arm-only sample | Yes: search/scan_arm.py | Recomputable arm-only Jacobian percentiles |
| Whole-body static support/IK study narrative | Yes: docs/WHOLE_BODY_STUDY.md | Traceable assumptions and previously recorded outcomes |
| Complete whole-body leg/IK/collision scripts | **No** | Historical study not independently reproducible here |
| Raw whole-body pose CSV and collision meshes | **No** | Stored in the earlier study archive, not in this source snapshot |
| Complete physical robot and measured hardware | **No** | No hardware success claim |

### Verify the published materials

1. Install Python 3.11, numpy and scipy from requirements-core.txt.
2. Run python scripts/verify_repository.py.
3. Run python search/check_jacobian.py.
4. Run python search/scan_arm.py --samples 8192 --output arm_only_results.csv.
5. To inspect CAD in Python, install CadQuery separately and run python scripts/export_step.py --inspect-only.

The emitted arm-only statistics should not be compared directly to the archived **whole-body, contact-constrained** statistics. In particular, the latter allow feasible leg motion while enforcing foot contacts. Workspace coverage, collision screening and torque estimates need the original complete solver and geometry and have not been verified by this CI.

### Source fidelity

- The editable STEP is a derived CAD model, not merely a screenshot.
- Upstream source references and license notices appear in docs/PROVENANCE.md and LICENSES/.
- The included viewer is a single offline HTML file; its collision geometry is simplified for visualization.
- Published scripts use SI units for kinematics. STEP CAD coordinates are in millimetres.

When the rest of the model is imported, each source/input set should carry its own revision ID and hash to distinguish reproduced results from historical tables.
