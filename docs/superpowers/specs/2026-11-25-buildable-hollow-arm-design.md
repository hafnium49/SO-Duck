# Buildable hollow arm for SO-Duck: design

Date: 2026-11-25. Status: draft for the owner's approval. Nothing here is built yet.

## Intent

Make the SO-Duck arm of the hollow three-part design something the owner can print and assemble. Today it is a computed study: the three custom parts exist only as open review meshes, and the shoulder servo cannot hold the arm.

The owner asked for two things in particular:

- Solve the servo load by changing the three custom parts, not by changing motors.
- Keep the custom parts in the style of the SO-101's own parts, which are hollow, channel-like shells. Build them from the SO-101 geometry.

## Background a builder needs

- The robot is an Open Duck Mini v2 biped. Its head and neck are replaced by a six-axis arm with an SO-101 gripper. Joint order from the body: shoulder yaw, shoulder pitch, elbow (named `forearm_pitch` in the model), forearm roll, wrist flex, wrist roll, then the gripper jaw. Seven arm servos, all Feetech STS3215, 7.4 V.
- Three parts are custom: the **neck socket mount** (holds the shoulder-yaw servo on the Duck's original four-screw neck socket), the **elbow-to-forearm-roll link** (fork on the elbow horns, span, band around the roll servo) and the **forearm-roll-to-wrist link** (from the roll servo's horn to the holder of the wrist-flex servo). Everything else in the arm is a stock SO-101 part or a servo. The viewer's other arm bodies match SO-101 parts by size: `shoulder_pitch_link` is `Rotation_Pitch_SO101`, `shoulder_collar` is `Motor_holder_SO101_Base`, `forearm_pitch_link` is `Upper_arm_SO101` and `wrist_roll_link` is `Wrist_Roll_Pitch_SO101`. The builder confirms each by overlaying the donor solid on the viewer mesh. If one does not match, stop and report: a fourth custom part is not in scope without the owner's say.
- `hollow/robot.xml` is a model reconstructed from the viewer. In it the forearm-roll joint sits 140 mm from the elbow axis, and the wrist-flex axis sits 80 mm to the side of the roll axis and 62 mm along it.
- `hollow/postures.py` and `coverage/resolve.py` replay and search postures for the 96 published grasp targets, with the leg-to-trunk gap rule (0.3 mm hip link, 1 mm thigh, shin and foot). Current result: 93 of 96.
- The SO-101 parts are published as STEP solids in TheRobotStudio/SO-ARM100 under Apache-2.0. Checked on commit `141aea7`: `Under_arm_SO101`, `Motor_holder_SO101_Wrist`, `Motor_holder_SO101_Base`, `Base_motor_holder_SO101`, `Wrist_Roll_Pitch_SO101`, `Upper_arm_SO101` and `Rotation_Pitch_SO101` each import in CadQuery 2.7 as one valid closed solid, and a cut and rejoin of each is exact.

## What the spikes measured

Gravity torque at the shoulder-pitch servo, with estimated masses. The STS3215 is listed at about 1.9 N m stall at 7.4 V.

| Arm | Empty | With 100 g at the tool |
| --- | ---: | ---: |
| Stock SO-101 on a table, worst pose | 0.86 N m | 1.27 N m |
| Hollow arm, postures as stored | 1.17 N m | 1.63 N m |
| Hollow arm, lowest-load posture per target | 1.14 N m | 1.57 N m |
| Forearm 40 mm shorter, wrist axis 40 mm closer to the roll axis | 0.91 N m | 1.29 N m |
| Forearm 55 mm shorter, wrist axis 50 mm closer | 0.83 N m | 1.16 N m |

The shortened rows moved joints only. No part was drawn and arm collisions were not checked. Moving the neck mount 40 mm forward helped little and lost targets.

Coverage of the 96 targets by the same joints-only method, with reach, joint ranges, the 5 mm support margin and the leg gap, and a deeper search for targets the first pass missed:

| Arm | Forward | Broad | Total |
| --- | ---: | ---: | ---: |
| Present lengths | 58/64 | 31/32 | 89/96 |
| Forearm 40 mm shorter, wrist axis 40 mm closer | 63/64 | 28/32 | 91/96 |
| Forearm 55 mm shorter, wrist axis 50 mm closer | 63/64 | 26/32 | 89/96 |

The method finds 89 for the present arm where the full search found 93, so differences of a few targets are noise. It is an upper bound for the shortened arms, because their parts have no meshes to collide yet.

## Goals

1. **Three printable parts.** Each is one valid closed solid, delivered as STEP and STL, built by a CadQuery script from the pinned SO-101 STEP solids plus new joining geometry.
2. **SO-101 style.** Where a part holds a servo or meets a horn, it uses the matching SO-101 feature cut from the donor solid: motor-holder band, horn cheek, channel section. New joining geometry is an open channel or shell with walls like the donor's, never a solid bar or tube.
3. **Shoulder load at the stock SO-101's level.** Over the reached targets, shoulder-pitch gravity torque is no more than the stock SO-101 arm's worst case, empty and with 100 g at the tool. This is the owner's "solve the servo issue", made measurable. It is my proposed criterion: the stock arm is the one load this servo is known to carry in practice. How both sides are computed is fixed under "Load calculation" below.
4. **Coverage reported honestly.** Rerun the 96-target search on the new geometry with every existing check and the leg gap, with arm collisions on the new full-resolution part meshes. Report the count. Target: at least 85 of 96 in the final stored postures, searched at the default budget and then with `--starts 2048` for targets still open. If that cannot be met together with goal 3, stop and bring the trade-off to the owner.
5. **A model that matches the parts.** `hollow/robot.xml` is generated from the same parameters as the CAD, with part masses from the solid volumes at a stated print setting and joint ranges measured by sweeping the real meshes.
6. **Assembly documents.** A fastener and parts list, a print guide (orientation, supports, material, settings) and an ordered assembly guide including servo order on the bus and how each horn is indexed at the zero pose.

## Non-goals

- A walking policy. The arm changes the robot's mass and takes over the neck motors, so walking needs separate training.
- Any hardware test. Fit, strength and thermal behaviour can only be proven by printing.
- Changes to the Duck's legs, trunk or roof, or to stock SO-101 parts.
- The six-part design in `coverage/`. It stays as it is.
- Motion between postures, grasp control, or a payload rating.

## Anti-goals

- Solid struts or tubes in place of SO-101-style shells.
- A part that needs a stock part drilled, cut or glued. The existing parking pad bonded to the roof is the one exception already in the design.
- Numbers that the repository cannot reproduce.
- Calling the robot "ready" beyond what was checked. Documents say "designed and checked in CAD and simulation, not yet printed".

## Constraints

- STS3215 at 7.4 V stays the servo for every joint.
- The mount uses the Duck's original four neck screws and nothing else on the trunk.
- Every part prints on a 180 x 180 mm bed in PLA or PETG without soluble supports. My assumption; the owner's printer is not known.
- Apache-2.0 notices for SO-ARM100 and Open Duck Mini stay with any derived geometry, and `docs/PROVENANCE.md` records the pinned commit.
- Repository rules hold: `scripts/study_inventory.py` regenerated, all workflows green, donors small enough to commit as they are (the seven STEP files total 3.9 MB).
- Tools already in `crosswise/requirements.txt`: CadQuery 2.7, MuJoCo 3.17, python-fcl, VTK.

## Approaches considered

1. **Rebuild all three parts from the SO-101 STEP solids.** Editable CAD, true SO-101 walls, can be shortened. Most work. Parts will resemble the review meshes without matching them. **Chosen by the owner.**
2. **Rebuild the forearm and wrist links; resample the existing mount mesh into a closed solid.** Less work, but the mount stays an uneditable mesh with screw holes only as accurate as a 0.1 mm grid.
3. **Patch the three review meshes.** Fastest. No editable CAD, and three repair tools failed to close the meshes in a spike (93 to 180 gaps per part).

## The design

### Layout

A new directory `hollow/cad/` holds the build. `donors/` has the pinned SO-101 STEP files with a checksum list. `params.py` is the single source of dimensions: joint positions, shortening, clearances, print settings. `build.py` produces `hollow/parts/<name>.step` and `.stl` and regenerates `hollow/robot.xml`. The existing scripts keep their roles: `postures.py` searches and replays, `inspect_viewer.py` checks the viewer.

### The three parts

- **Neck socket mount.** The SO-101 base motor-holder profile wraps the shoulder-yaw servo. An open channel upright joins it to a flange on the Duck's four neck screws. Yaw axis position stays where the model has it (1 mm forward of the trunk origin, 177 mm up), since moving it did not help.
- **Elbow-to-forearm-roll link.** Two cheeks on the elbow horns, a channel span taken from the SO-101 under-arm section, and a motor-holder band around the roll servo. The elbow-to-roll-joint distance drops from 140 mm by 40 to 55 mm.
- **Forearm-roll-to-wrist link.** A horn seat on the roll servo, a short channel bridge, and the SO-101 wrist motor holder for the wrist-flex servo. The side offset of the wrist axis drops from 80 mm by 40 to 50 mm, and the 62 mm offset along the roll axis by up to 12 mm.

### Load calculation

A new script, `hollow/loads.py`, computes every torque figure, so the repository can reproduce them. My calls:

- **Stock baseline.** `collision/inputs/so101.xml` with its link inertials, base fixed upright as on a table. The worst shoulder-lift gravity torque over 20,000 random poses inside its joint ranges, empty and with a 100 g point mass at the `gripperframe` site. The spike gave 0.86 and 1.27 N m; the script's own figures are the limit.
- **Hollow arm.** Stock links use the same SO-101 inertials and 55 g per servo. The three custom parts use their full solid volume at 1.24 g/cm³. That overstates a printed part, so it is the safe side. Lighter print settings do not count toward passing.
- **Which posture.** For each target, the stored posture is the passing candidate with the lowest shoulder-pitch torque with 100 g. Every stored posture still needs the 5 mm support margin, so margins will be smaller than today's and the README reports them as they come.

### How the lengths are chosen

Load comes first, because it is cheap to test without CAD. The builder moves joints only, as the spike did, and finds the smallest shortening that meets goal 3 with the masses above. That sets the required lengths. The CAD then has to pack them: at least 1.5 mm between any two parts that move relative to each other over the joint's range, room to insert and remove each servo and reach its screws, and room for the servo cable.

If the required lengths do not pack, stop and report the best lengths that do and the loads they give. Do not quietly ship a part that misses goal 3.

### Pickup, fold, overhead and parked postures

The viewer and CI also hold a floor pickup, an overhead pose, a parked pose and the fold between them. A shorter arm invalidates all four, so they are made again. My calls:

- **Pickup.** Same cube position. `postures.py --search-pickup` finds a new crouch and path. If the shorter arm cannot reach the cube, the cube moves toward the feet and the README says by how much.
- **Overhead.** Tool 200 mm forward on the centre line, as high as the arm reaches with every check passing. The height will be lower than the present 650.6 mm and is reported.
- **Parked.** Fixed fingertip on the rear pad, as now. The pad may move on the roof if the shorter arm needs it; the new position is documented for bonding.
- **Fold.** Joint-space interpolation from overhead through one waypoint to parked, in steps of at most 0.5 degrees per joint, with every check on every frame. This is the same method as `coverage/replay.py` uses and is not a general motion planner.

`postures.py` gains the code to write these clips into the viewer, and the frame counts expected by `inspect_viewer.py` follow the new clips.

### Checks, in the order they gate the work

1. Each part: one valid solid, watertight STL, wall thickness nowhere below 1.6 mm, screw holes at the diameters of the donor features.
2. Assembly at the zero pose: no part intersects a servo or a neighbouring part, on full-resolution meshes.
3. Joint sweeps on the real meshes give the usable range of each arm joint. Those ranges go into the model, replacing the ranges inferred from stored angles.
4. Posture search and replay with every existing check. Arm pairs use the new meshes, so the touching-face workaround for display meshes is removed where the new meshes make it unnecessary.
5. Torque report for goal 3, empty and with 100 g.
6. The viewer is regenerated from the new model and loaded in a browser on both render paths.

### What happens to the existing files

The three review meshes and `hollow/meshes/` stay until the owner has seen the new parts in the viewer and accepted them. After that they are removed from the tree and remain in history. My call, to be confirmed at that point.

## Left to the builder

- The exact shape of the joining geometry, within goal 2.
- Which donor features to cut, and where.
- The print setting used for mass (perimeters and infill), stated in `params.py`.
- The four neck screw positions: measure them from the Open Duck Mini v2 CAD at a pinned commit and confirm against the existing mount mesh.
- File and function structure inside `hollow/cad/`.

## Open points for the owner

- The printer's bed size and material, if different from the assumption above.
- Whether an SO-101 follower kit is on hand to supply the stock arm parts and servos. It only changes the parts list.
