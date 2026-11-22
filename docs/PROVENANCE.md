# Upstream geometry and license provenance

This repository combines independently published mechanical and simulation models. **It is public because it is a research record, not because the entire content is original or patent-free.**

| Component | Upstream | Attribution / local files |
|---|---|---|
| Duck mechanical form, mounting and native meshes | [apirrone/Open_Duck_Mini, `v2`](https://github.com/apirrone/Open_Duck_Mini/tree/v2) | `collision/inputs/duck_meshes/`, head/terminal reference; original Apache-2.0 text under `LICENSES/` |
| Duck simulated links, mass and joint limits | [apirrone/Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground) | `legs/duck.xml`, `collision/inputs/robot_motors.xml`; check source-specific license scope |
| SO-101 arm, wrist, jaw and servo models | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | `collision/inputs/arm_meshes/`, `cad/under_arm_so101_v1.step`, `collision/inputs/so101.xml`; Apache-2.0 source |
| Parametric CAD methodology / prior art | [yotown/gym](https://github.com/yotown/gym) | cited as separate precedent; **not asserted as the source** of these study meshes |
| Derived hybrid forearm | This study, via `cad/build_forearm.py` | `cad/Case_mounted_forearm.step`, `.stl`, `collision/candidate_forearm.stl` |
| Kinematic/collision/pose analysis | This study | scripts and CSV datasets in `search/`, `legs/`, `study/`, `collision/` |

The upstream license copies are in `LICENSES/`, and the original collision-input directory also retains duplicate copies for provenance. The original repositories may contain separate licenses for specific external or mechanical assets; upstream attribution should be retained when redistributing derived geometry. Original study contributions have **no blanket license declared** without a deliberately added top-level LICENSE.

Model geometry was gathered for research. No bench metrology, mechanical fatigue testing or tested motor thermal envelopes are implied.

## Stock-arm revision

The stock arm in `stock/robot.xml` uses the complete base subtree and actuators from the SO-101 follower simulation. The six supplemental arm/base STL files in `stock/assets/` are from [TheRobotStudio/SO-ARM100 at 141aea7](https://github.com/TheRobotStudio/SO-ARM100/tree/141aea7bcac2744d92998e30268c413202a56f96/Simulation/SO101). Seven original Duck torso electronics meshes in that directory are from [Open Duck Mini V2 at b23317a](https://github.com/apirrone/Open_Duck_Mini/tree/b23317a485b3cec7d8417f352478778b3475173c). The other donor meshes and source joint specifications use the archived inputs listed above. The one new mounting solid and the parametric CAD source are in `stock/`.
