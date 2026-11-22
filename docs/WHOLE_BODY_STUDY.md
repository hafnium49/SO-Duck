> **Archive notice (2026-11-22):** This report documents a broader local study package, not all of which is currently in this public GitHub checkout. References below to legs/, collision/, search/scan_robot.py, study/pose_test.py, and raw CSVs describe original study inputs and outputs; only the upper-chain Jacobian tools and the summary were transferred in the initial public commit. The historical claims have not been independently reproduced from this minimal public snapshot. See [REPRODUCIBILITY.md](REPRODUCIBILITY.md).

# SO-Duck — bipedal manipulation research prototype

**Status:** Archived computational study, not an independent hardware validation. The mechanical fit, stability and pose-coverage figures below are simulated/preliminary; no complete hardware validation has been reported.

**Project:** Fuse the 10-DoF Open Duck Mini V2 lower body and 4-DoF neck with the last two SO-101 wrist axes and 1-DoF gripper using one custom case-mounted forearm. This gives 16 pose-changing DoFs plus 1 gripper DoF, excluding optional Duck antennas. The target is a lightweight mobile manipulator with forward/backward wrist bending.

**Read first:** [Architecture](docs/ARCHITECTURE.md) · [Validation limits](docs/VALIDATION.md) · [Provenance & upstream licenses](docs/PROVENANCE.md) · [Design decisions](docs/DECISIONS.md)

**Interactive viewer:** Open [`SO_Duck_pose_explorer.html`](SO_Duck_pose_explorer.html) locally in a browser. It presents recorded poses, not live physics dynamics.

**Code check:** `python scripts/verify_structure.py`

---

# SO-Duck whole-body grasp study

## Build choice

Keep the ten Duck leg servos and all four neck axes in their factory orientations. Use the stock Duck yaw bracket, terminal servo, and rear roll bearing. A single replacement forearm connects the moving terminal-servo case and the rear bearing to the SO-101 wrist motor holder. At joint zero the forearm points forward horizontally, the terminal rolls about the forward axis, the wrist bends about the left–right axis, and the gripper rolls about its length. Positive wrist flex lowers the gripper. The jaw retains a separate actuator. The head shell carries no arm load. The design has one newly designed rigid part, and two additional ordinary M3 bearing-retention screws and washers. A compatible all-stock assembly with these axes was not established.

The two bearings at the ends of the neck assembly support different functions. The driving horn of the original terminal servo stays attached to the native bracket. The servo case and opposite support bearing turn with the custom forearm. The CAD uses millimetres. The custom solid spans 244.45 × 52.08 × 66.91 mm. See [the assembly and fit notes](cad/Mount_fit.md).

## What the feet contribute

The whole robot has 16 actuators that change the grasp pose or the supported body posture, plus the jaw. The model treats the floating torso as an unactuated body. For each planted foot it enforces zero translational and angular velocity. With the two flat feet splayed 10° per side in the reference crouch, four independent combinations of leg rates can move the torso while preserving contact. Their actuator-space norm accounts for all ten leg motors. The six upper joints add independent rate coordinates. A single flat stance leg can move the torso in five instantaneous directions. The free swing leg changes mass distribution; taking a step changes the feasible world workspace.

At the reference leg posture, simply lifting the right foot with the horizontal, neutral arm leaves the modeled center of mass about 85 mm outside the left sole. A separately checked single-foot pose required about 41° of trunk roll and 30° of pitch. With the heavier native-case mass estimate it retained 5.47 mm of unloaded margin, falling to 3.24 mm with 50 g at the tool. The tested grasp strategy is to choose a stance and then manipulate in double support. The reported replants assume the biped can arrive at each new stance. They establish grasp endpoints. Gait generation, continuous collision-free trajectories, stability during transfer, and motor heating require separate validation.

## Jacobian comparison

The tool point is the closed-jaw SO-101 gripper reference site. Each posture maps unit-Euclidean-norm servo velocities in rad/s to the spatial tool velocity vector with its translation divided by 0.15 m. Thus linear and angular rows have comparable numerical units. A singular value gives the tool velocity gain in one principal direction. A small minimum means some motions need large servo rates. A large maximum means other motions amplify servo errors. The condition number is their ratio. Equal servo rate weights and a 150 mm translation per radian task weighting are study assumptions. The scalar gain distribution changes if the requested task weights change.

| Terminal axis, wrist flex and forearm mounting | 5th percentile minimum gain | 95th percentile maximum gain | 95th percentile condition number |
|---|---:|---:|---:|
| Both shafts transverse, upright stock SO forearm | 0.127 | 5.090 | 37.88 |
| Duck terminal roll and transverse wrist, upright forearm | 0.155 | 4.909 | 29.00 |
| Duck terminal roll and transverse wrist, horizontal forward forearm | 0.162 | 4.602 | 22.86 |

The comparison uses 8,192 identical held-out joint vectors per layout over the model limits at the same leg posture with both feet fixed. These percentiles describe sampled joint configurations prior to collision and balance screening. They are separate percentiles. Sensitivity at 100, 150 and 200 mm, single support, and additional leg postures is in the search tables. The coarse search compared 715 coupled-shaft orientations and 13 separately oriented terminal/wrist geometries. The latter can be manufactured as the single case-mounted part. Further forward clocking to 120° reduced the sampled 95th-percentile condition number to 21.68 on the independent fine sweep, with the neutral tool only 45 mm above the torso origin. Horizontal clocking left it 191 mm above the torso origin and gave better forward-grasp coverage than 120° on the test bank.

## Common world targets

Every configuration faced the same 64 forward grasp poses in a box 180–300 mm forward, ±65 mm lateral and 365–485 mm above the sole plane. The horizontal origin is the nominal upright torso projection. Target tool-frame orientations use intrinsic Z–Y–X yaw, pitch and roll in ±40°, ±30° and ±45°. A second bank tests 32 samples distributed over all three-dimensional rotations at the first 32 positions. These are finite samples. Failed IK searches do not prove infeasibility.

The nominal stance uses hip yaws +10° left and −10° right, zero hip rolls, hip pitch −0.40 rad left and +0.40 rad right, and bilateral knee +1.25 rad and ankle −0.85 rad. Both feet are fixed in the world throughout each endpoint solve. IK uses 24 starts, up to 100 iterations, 2° joint-limit reserve, tip tolerances 1.5 mm and 1°, and right-foot closure tolerances 0.2 mm and 0.15°. Left-foot closure is enforced by construction. Each foot replant rotates the stance to face the tool's horizontal approach and sets the target distance ahead of the reference torso to either 220 or 270 mm, keeping target elevation and full world orientation. The foot placement consequently varies by target.

| Foot placement for the horizontal forearm | Forward endpoints | All-orientation endpoints |
|---|---:|---:|
| Original two-foot stance | 45 / 64 | 9 / 32 |
| Replanted, 220 mm target distance | 43 / 64 | 21 / 32 |
| Replanted, 270 mm target distance | 59 / 64 | 26 / 32 |
| Any of these placements | **62 / 64** | **26 / 32** |
| Any placement with the illustrative gain bounds below | **61 / 64** | **26 / 32** |

All counts in this table require both pose tolerances, a 5 mm projected center-of-mass margin inside the modeled sole support polygon, and a clear endpoint in the tested 146 mesh-pair and floor checks. The gain-bounded row additionally requires a minimum singular value of at least 0.15, a maximum of at most 4.5 and a condition number of at most 25. Those are example engineering limits, selected to illustrate the trade-off. Among the 45 passing forward endpoints at the original stance, the 5th-percentile minimum gain was 0.191, the 95th-percentile maximum was 4.404, and the 95th-percentile condition number was 19.82. Every one of those endpoints had minimum gain at least 0.183 and maximum gain at most 4.590.

With the recommended arm on a fixed torso, IK found 9/64 forward and 1/32 all-orientation targets meeting pose and nominal balance criteria before collision checks. Allowing the leg joints to articulate with both feet still planted raised those counts to 49/64 and 9/32 before collision checks. The corresponding screened counts are reported above. Results for other candidate clocks are included prior to full collision checks. A 105° alternative gave 50/64 forward hits and 8/32 unrestricted-orientation hits at the nominal balance stage, compared with 49/64 and 9/32 at 90°. The 90° part is the horizontal compromise modeled in CAD and tested for collisions. There is no proof of a unique global optimum or complete orientation coverage at every point.

The IK saved one representative per target and placement, preferring a higher minimum singular value among nominally balanced solutions. Exploring other IK branches could recover additional feasible poses. The matrices and joint coordinates are geometric study outputs. Servo command signs, zero references, available current and control limits belong to the real hardware calibration.

## Static loads and clearance

The reference unloaded biped mass of approximately 2.019 kg uses stock trunk and leg masses, the two proximal Duck neck links, SO-101 lower arm, wrist, gripper and jaw inertias, and estimated yaw/terminal mounting masses. A separate sensitivity run adds 85 g at the custom link's modeled centroid and 18 g at the rear bearing. Of the 45 initially accepted forward poses, 43 still clear the 5 mm balance margin under that mass increase. The 50 g test mass at the tip changes the center of mass in a pose-dependent direction. It can counterbalance rearward lean.

Using the native moving-case mass model with the heavier link and bearing, the greatest calculated upper-joint gravity demand across those 45 grasps occurs at head pitch, the second neck joint. Its peak is 0.756 N·m empty and 0.929 N·m with 50 g at the tool point. Wrist-flex peaks are 0.116 and 0.193 N·m respectively. These are static actuator-axis torques and exclude walking acceleration, gripper forces, friction, joint elasticity and thermal duty limits. They do not establish a payload rating.

The collision check uses 19 rigid mesh groups with over 2.38 million source triangles, including both legs, trunk, neck and donor wrist. The local mating checks use the native CAD. Nominal endpoint checks omit cable harness, payload shape and intended joint interfaces. Additional limits are detailed in [the endpoint-screen notes](collision/README.md).

## Files and reproduction

- Open `SO_Duck_pose_explorer.html` in a browser for the articulated whole robot. The viewer draws the retained solutions in world coordinates, including changed foot placements. Its reported static margins use the nominal unloaded mass model. The lighter display meshes are separate from the collision meshes.
- The `cad` directory includes the one-piece STEP/STL, mount illustration, neutral neck/wrist GLB, and the parametric construction with its native donor solid. The stock gripper and hardware shown in the GLB are reference geometry.
- The `search` and `study` directories contain the reproducible orientation scans, target definitions, and per-target outputs. The technical parameter `two_part=True` in the original model denotes independent terminal and wrist axes. The supplied design achieves that geometry with one custom piece.
- The `collision` directory includes the original screening inputs, rebuildable C++ intersection helper, mesh-pair definitions and per-target results. The `legs` directory contains the contact model and static-load study. Native source licenses accompany the geometry.

Run with Python 3, NumPy and SciPy. VTK and a C++17 compiler are required only for rebuilding the triangle checks. CadQuery is required only to regenerate the manufacturing part. Example commands from the package root follow.

```sh
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

Robot definitions and geometry are adapted from the published models. [Open Duck Mini v2 hardware](https://github.com/apirrone/Open_Duck_Mini/tree/v2) · [Open Duck Playground joint model](https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml) · [TheRobotStudio SO-101 joint model](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.xml) · [Modern Robotics on manipulability](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-4-manipulability/).