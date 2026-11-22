# SO-Duck: a biped that can reach

**Public research prototype · CAD / simulation study · hardware unverified**

SO-Duck combines the [Open Duck Mini V2](https://github.com/apirrone/Open_Duck_Mini/tree/v2) bipedal chassis and its four-joint neck with the final two wrist joints and gripper of the [SO-101](https://github.com/TheRobotStudio/SO-ARM100) arm.

The design goal is a **minimally modified, 6-DoF tip-pose manipulator on a small walking robot**, with one additional jaw actuator and as few custom printed parts as possible. We are exploring a one-piece, forward-facing, case-mounted forearm that preserves the four neck joints, supports the opposite idler bearing, and permits forward/backward wrist flex.

## Architecture

| Subsystem | Actuated joints |
| --- | ---: |
| Duck legs | 10 (5 per leg) |
| Duck neck: pitch, pitch, yaw, roll | 4 |
| SO-101 wrist: flex and roll | 2 |
| SO-101 gripper opening | 1 |
| **Total** | **17** (16 pose-changing + 1 jaw) |

The stock Duck head shell is *not* the load-bearing arm mount. This design is not yet a physically proven prototype.

## Project status

- **Mechanical**: proposed case-mounted single-part forearm; CAD interference checks are preliminary, not a proof of durability or fit on an assembled robot.
- **Kinematics**: six-dimensional instantaneous tip control is possible in some configurations; not every desired pose is reachable, and singularities / joint limits remain.
- **Whole-body**: simulated double-foot-support target solving, collision screening and example Jacobian-conditioning studies; no verified manipulation or walking on physical SO-Duck hardware.
- **Actuators**: torque, gearing, power, cable routing and thermal limits require measured validation.

## Reproduction and provenance

The source is organized into CAD generation, kinematic optimization, biped contact/balance, target IK, and collision experiments. See `docs/ARCHITECTURE.md`, `docs/VALIDATION.md`, and `docs/PROVENANCE.md` for the method and caveats. The interactive viewer visualizes **saved postures**; it is not a live physics simulator.

External references: [Duck Playground](https://github.com/apirrone/Open_Duck_Playground) · [SO-101](https://github.com/TheRobotStudio/SO-ARM100) · [Yo.Town Gym](https://github.com/yotown/gym), a parametric CAD reference for both donor designs.

Upstream materials retain their Apache-2.0 notices. Unless explicitly indicated, **no new license has been granted to original SO-Duck content** merely by publishing this repository. The claims and numerical results in this repository are computational results subject to the stated assumptions, not hardware specifications.

## Development

Run `python scripts/verify_structure.py` for the source/asset smoke check. Dependencies for individual studies appear in `requirements-*.txt`.

Contributions and reproducibility reports are welcome; simulation claims should include the model revision, joint axes, chosen task-space metric and feasibility constraints.
