# Study methods and definitions

This document describes the provenance and operational interpretation of the saved computations. It is a retrospective record of the source scripts and results, not a peer-reviewed validation of them.

## 1. Robot coordinates, joints, and assumptions

- Torso basis: **+X forward, +Y left, +Z up**; lengths in metres for dynamics/kinematics, millimetres for manufactured CAD.
- Duck leg order: left hip yaw, roll, pitch, knee, ankle; same five on the right. Joint signs follow the published MJCF.
- Arm pose-chain order: neck pitch, head pitch, neck yaw, terminal roll, SO-101 wrist flex, wrist roll; jaw opening excluded from the 6D pose Jacobian.
- Candidate geometry: terminal roll axis remains along the forward-facing segment, wrist-flex axis nominally transverse, one rigid forearm; the head shell is not a structural mounting element.
- Configuration named `two_part=True` in analysis code means *independently oriented terminal and wrist shafts*, **not** that the physical design uses two new printed parts. The proposed realization is one custom part.
- Upper-chain geometry combines published Duck and SO-101 MJCF frames with a hypothetical forearm datum. Model inertias for the candidate link and bearing are estimates.

## 2. Whole-body contact model

The torso is a floating rigid body, **not** a free commanded actuator. Double support imposes six instantaneous no-slip constraints per planted sole (translation and rotation). Solving these plus leg kinematics leaves four linearly independent leg-rate directions in the reference 10-actuator stance. Single support removes the other sole constraint; the left stance leg can contribute five independent torso motions and the swing leg changes mass distribution. The simulated results assume rigid flat feet and admissible static support conditions. Contact compliance, friction cones and gait transitions are not demonstrated.

The nominal crouch uses hip yaws +10°/−10°, hip rolls 0, left/right hip pitch −0.40/+0.40 rad, bilateral knee +1.25 rad, ankle −0.85 rad. Feet are held in world space during each endpoint IK solve.

## 3. Jacobian definition and survey

The spatial tool velocity is `(v_x / L, v_y / L, v_z / L, omega_x, omega_y, omega_z)` where `L = 0.15 m` in the reported principal study and servo rates are in rad/s, equally weighted. Thus the singular values are **dimensionless** for this particular task metric. They change if translation scale, rate weights, or target directions change. The samplewise condition number is largest/smallest singular value; different percentiles in a summary row are not mathematically linked to each other.

The model compares 715 coupled-shaft candidates and 13 decoupled orientations, including 8,192 common held-out vectors per layout in the primary whole-body survey. Supplemental results use 16,384 samples for refinement or upper-arm validation, 100/150/200 mm scaling sensitivity and different stance assumptions. See [`search/README.md`](../search/README.md) and exact scripts for sampling conventions and seeds. Joint-space surveys **do not** include collisions, joint torque, ground or balance constraints.

## 4. World-space grasp bank and numerical IK

- **Forward bank:** 64 positions and orientations, at x=180–300 mm forward, y=±65 mm, z=365–485 mm above the sole; intrinsic yaw/pitch/roll angles span approximately ±40°/±30°/±45°.
- **Orientation stress bank:** 32 broadly distributed 3D rotations at the first 32 positions.
- 24 initializations per target and up to 100 iterations; nominal tip tolerances 1.5 mm and 1°; right-foot closure tolerances 0.2 mm and 0.15°.
- Joint limit reserve: 2°. Left-foot closure is construction-enforced; right-foot closure checked by solver.
- Tested stance policies: original reference, and independent *endpoint replant* configurations with stand-off 220 or 270 mm from the target. These are alternate initial foot placements, **not solved stepping paths**.
- Center of mass must project at least 5 mm inside the modeled sole-support convex polygon.

Failure to find an IK solution within search limits is not a proof of unattainability.

## 5. Collision screening

The saved C++/VTK mesh tester considers 19 rigid mesh groups and 146 designated group pairs, drawing on 2,383,480 triangles of sourced STL geometry. It checks mesh-surface intersections and nonfoot floor penetration at nominal endpoint poses, excluding known adjacent mechanical interfaces. It does **not** prove positive clearance, prevent unseen internal penetration, account for harness/fasteners/payload envelope, or check a continuous reach trajectory. Rebuild commands and exact scope appear in [`collision/README.md`](../collision/README.md).

## 6. Static torques

Gravity loading is computed in an idealized rigid model with nominal and heavier case/link/bearing mass estimates and an optional 50 g load at the gripper point. No dynamic inertial torque, contact wrench, gear efficiency, motor thermal model or safety margin is included. A static torque below a modeled limit **cannot** be interpreted as a continuous-use servo rating.
