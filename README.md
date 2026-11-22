# SO-Duck | whole-body bipedal manipulation research

## Current design · complete SO-101 on the first Duck neck joint

One custom adapter bolts the original SO-101 base to the moving plates of the original Duck neck pitch axis. It retains all five stock SO-101 arm axes, its independent gripper, and the ten Duck leg actuators. The installed neck pitch range is −20° to +30°. The modeled assembly reaches 59 of 96 fixed-foot grasp targets and can geometrically pick up a 20 mm cube from the floor. Hardware performance remains untested.

[Assembly and verified results](stock/README.md) · [Connector STEP](stock/connector.step) · [Printable STL](stock/connector.stl) · [Interactive model](stock/poses.html)

## Earlier four-axis Duck neck study

**Public computational study · 22 November 2026 · hardware not yet validated**

SO-Duck studies a **six-joint end-effector pose chain** made by retaining Open Duck Mini V2's four-axis neck and adding the final two SO-101 wrist axes, plus an independently actuated SO-101 gripper. The robot retains Duck's ten actuated leg joints and uses one proposed custom, case-mounted forearm. The shell is not intended to carry the manipulator's structural load.

**Status:** This repository records CAD, kinematics, finite joint-space surveys, numerical whole-body IK, endpoint collision screening, and estimated static torque/balance. It is *not* a physically demonstrated, walking-and-grasping robot. The workspace counts are results on specified finite target banks, not guarantees of arbitrary pose reachability.

## At a glance

| System | Joint count | Role |
|---|---:|---|
| Duck legs | 10 | Locomotion and constrained torso pose |
| Duck neck | 4 | Neck pitch, head pitch, yaw, terminal roll |
| SO-101 distal wrist | 2 | Wrist flex and wrist roll |
| SO-101 gripper | 1 | Jaw opening |
| **Total** | **17** | **16 pose-changing joints + 1 gripper** |

**After cloning:** run `python scripts/restore_assets.py` to unpack 45 native geometry files from the lossless Git-tracked chunks, then run `python scripts/verify_study.py` to validate the entire asset/source inventory. See [archive details](archives/README.md).

The end-effector can have a rank-six Jacobian in some configurations, but not in all configurations. With both soles fixed, the legs supply four **constrained** torso-motion directions in the nominal stance; leg DoFs are not unrestricted independent end-effector coordinates.

## Study snapshot

- **Mechanical concept:** one replacement forearm retaining the moving-case terminal roll joint, mounting via the terminal servo and opposite support bearing; no load transmitted through the stock Duck head shell.
- **Geometric conditioning:** sampled forward-facing orthogonal-shaft layout: fifth-percentile minimum gain 0.162, 95th-percentile maximum gain 4.602, 95th-percentile Jacobian condition number 22.86, in the specified *dimensionless* task scaling; these are separate sampled percentiles.
- **Finite target bank:** 45/64 forward grasps reached, balanced and collision-screened at a nominal planted stance; 62/64 among three tested stances; 26/32 all-orientation samples among those stances. The replant counts are **endpoint-only** tests; there is no verified walking trajectory between stances.
- **Static torque:** calculated maximum second-neck-pitch torque across 45 accepted forward grasps 0.756 N·m unloaded and 0.929 N·m with 50 g, in a specified heavier-link sensitivity model. Not a sustained motor torque rating.

Details, caveats, and derivation are in [Methods](docs/METHODS.md), [Results](docs/RESULTS.md), and [Validation](docs/VALIDATION.md).

## Open the model

- **[Interactive whole-body pose explorer](SO_Duck_pose_explorer.html)** — saved grasp solutions, stance choices, joint angles, conditioning, COM margins. It is not a live dynamics simulation.
- [One-piece forearm (STEP)](cad/Case_mounted_forearm.step) is available directly. After restoring geometry, `cad/Forward_neck_and_wrist.glb`, `cad/Case_mounted_forearm.stl`, and `cad/Forward_mount.png` are available locally.

## Repository map

| Directory | Source and results |
|---|---|
| [`cad/`](cad/) | Construction program, reference STEP, candidate STEP/STL, GLB, fit analysis |
| [`search/`](search/) | Upper-chain FK/Jacobians, orientation scans, percentiles, sampling and cross-checks |
| [`legs/`](legs/) | Duck MJCF, sole mesh, double/single-support kinematics, mass and static loads |
| [`study/`](study/) | Whole-body target solver, sampled end-effector poses, candidate stance results |
| [`collision/`](collision/) | Upstream STL meshes, hybrid mesh, BVH/VTK intersection source, endpoint results |
| [`docs/`](docs/) | Methods, findings, provenance, raw-data schema, limitations, reproduction |
| [`scripts/`](scripts/) | Inventory and repository integrity checks |

## Reproduction

Python 3.11+ is a reasonable development baseline; the original study source does not enforce this minimum. Core Python dependencies are NumPy and SciPy. CadQuery is optional for CAD regeneration; VTK and a C++17 compiler are optional for rebuilding collision checks. See [the reproduction guide](docs/REPRODUCIBILITY.md) for commands, requirements, expected outputs, and distinctions between checks and validation.

```bash
python scripts/restore_assets.py
python scripts/verify_study.py
python search/checks.py
# Later, on a machine with NumPy, SciPy, VTK and C++17 as needed:
python study/pose_test.py O90 --two --clock 90 --starts 24 --steps 100
```

## Attribution and scope

Upstream: [Open Duck Mini V2](https://github.com/apirrone/Open_Duck_Mini/tree/v2), [Duck Playground](https://github.com/apirrone/Open_Duck_Playground), [SO-101](https://github.com/TheRobotStudio/SO-ARM100), plus parametric mechanical precedents in [Yo.Town Gym](https://github.com/yotown/gym). See [Provenance](docs/PROVENANCE.md), [Prior art](docs/PRIOR_ART.md), and bundled upstream Apache-2.0 licenses. This repository is public; no new blanket license is inferred for original material unless a separate license is explicitly added.

**Engineering boundary:** simulation feasibility, static balancing, finite-mesh intersection, and modeled servo torques do not prove real-world strength, contact stability, motor speed/gear-ratio suitability, safe thermal duty, cable clearance, or walking while grasping.
