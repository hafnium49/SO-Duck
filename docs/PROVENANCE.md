# Upstream geometry and license provenance

This repository combines independently published mechanical and simulation models. **It is public because it is a research record, not because the entire content is original or patent-free.**

| Component | Upstream | Attribution / local files |
|---|---|---|
| Duck mechanical form, mounting and native meshes | [apirrone/Open_Duck_Mini, `v2`](https://github.com/apirrone/Open_Duck_Mini/tree/v2) | `collision/inputs/duck_meshes/`, head/terminal reference; original Apache-2.0 text under `LICENSES/` |
| Duck simulated links, mass and joint limits | [apirrone/Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground) | `legs/duck.xml`, `collision/inputs/robot_motors.xml`; check source-specific license scope |
| SO-101 arm, wrist, jaw and servo models | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | `collision/inputs/arm_meshes/`, `cad/under_arm_so101_v1.step`, `collision/inputs/so101.xml`; Apache-2.0 source |
| Parametric CAD methodology / prior art | [yotown/gym](https://github.com/yotown/gym) | cited as separate precedent; **not asserted as the source** of these study meshes |
| Crosswise connector | SO-Duck fore–aft wrist CAD study, stock-forearm variant | `crosswise/connector_assembly.step` retains the original located yaw case, carrier, terminal case and SO-101 fork. The standalone carrier STEP/STL use Duck print coordinates in mm. Native wrist and jaw geometry retain TheRobotStudio attribution. |
| Derived hybrid forearm | This study, via `cad/build_forearm.py` | `cad/Case_mounted_forearm.step`, `.stl`, `collision/candidate_forearm.stl` |
| Kinematic/collision/pose analysis | This study | scripts and CSV datasets in `search/`, `legs/`, `study/`, `collision/` |

The upstream license copies are in `LICENSES/`, and the original collision-input directory also retains duplicate copies for provenance. The original repositories may contain separate licenses for specific external or mechanical assets; upstream attribution should be retained when redistributing derived geometry. Original study contributions have **no blanket license declared** without a deliberately added top-level LICENSE.

Model geometry was gathered for research. No bench metrology, mechanical fatigue testing or tested motor thermal envelopes are implied.
