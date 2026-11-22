# Orientation study

The six arm coordinates, all in radians, are neck pitch, head pitch, neck yaw, terminal hinge, wrist flex and wrist roll. The tool is the SO-101 gripperframe site. Jaw opening is held fixed. The kinematic inputs are extracted from the Open Duck Playground Open Duck Mini v2 MJCF, the SO-ARM100 project's Simulation/SO101/so101_new_calib.xml, and the supplied direct-mount CAD datum. Nominal coordinates in the torso basis use +X forward, +Y left and +Z up.

The stock-fork family rotates the donor fork, terminal hinge and wrist as a group. The carrier orientation is Rz(azimuth) Ry(−elevation) Rx(clock) about the donor terminal-shaft midpoint. In the print coordinate system this point is (97.15, 0, 129.06) mm. A 90° azimuth and zero elevation puts the wrist-flex axis along +Y. At positive forward clock, the initially vertical distal geometry rotates toward +X. For the shaft-decoupled layout, only the wrist and distal geometry use this transformation. The terminal screw remains on the +X line through (78.95, 0, 129.06) mm. Translating its reference point along that line gives exactly the same screw. The Boolean column called two_part selects the decoupled joint geometry. A new case-mounted forearm on the original Duck carrier can realize this geometry with one custom part.

## Running the sweeps

The scripts require Python 3, NumPy, and SciPy. Place duck.xml, sole.stl and legs.py in a sibling directory named legs. Run from this folder:

```sh
python scan_arm.py
python scan_robot.py
python refine_clock.py
python checks.py
```

The scripts use fixed Sobol seeds and the same input points for every design being compared. The primary joint-space sweep spans the full upstream MJCF limits. The secondary central sweep uses the degree intervals [−10,50], [−35,35], [−75,75], [−27,27], [−70,70], [−115,115]. These postures have no collision, ground-clearance, balance, torque, cable or velocity filtering. The scripts in the separate study folder test common Cartesian targets.

- **arm_model.py** supplies batched FK and the geometric Jacobian in metre and radian units. The returned Jacobian uses translation divided by the length argument, default 0.15 metres.
- **scan_arm.py** screens 715 coupled-shaft orientations with 1,024 samples per design. Independent validation uses 16,384 configurations per joint range. The CSV quantiles are independent for each metric.
- **scan_robot.py** screens 715 coupled and 13 decoupled orientations, using 512 training points and 8,192 validation points. A left/right hip-yaw pair of +10°/−10° gives symmetric toe-out and four independent double-support leg rates. Left hip pitch −0.40, right hip pitch +0.40, bilateral knee +1.25 and bilateral ankle −0.85 rad give level soles. The single-support comparisons keep the same trunk posture with one foot constraint released. Single support assumes the robot could support its mass at that posture.
- **refine_clock.py** tests decoupled forward clock angles every 5° from 45° to 150° on 16,384 samples per angle for each sampling range. A clock of 90° positions the neutral forearm horizontally forward.
- **checks.py** checks the Jacobian with independent central differences. The torso check samples 256 upper joint vectors, and the whole-robot check perturbs each admissible leg and upper direction, honoring the left contact exactly and checking right contact to first order. It also writes double-support posture sensitivity for four leg configurations.

The dimensionless singular values correspond to output [v / L; ω] for Euclidean unit-norm actuator rates in rad/s. The leg-rate basis is orthonormal in actuator coordinates. The ratio κ is the largest singular value divided by the smallest at each configuration. A high κ indicates that tip motion in one direction requires much larger actuator rates than another. This geometric measure assumes equal actuator rate weights. The reported percentiles describe the sampled regions and carry no worst-case guarantee.

## Source models

- [Open Duck Playground robot MJCF](https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml)
- [SO-101 MJCF](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.xml)
