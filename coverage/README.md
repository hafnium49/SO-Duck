# SO-Duck grasp coverage

This arm reaches 95 of the 96 position-and-orientation targets in the published planted-feet benchmark, with a required gap between the legs and the trunk. The model has six arm axes, one gripper actuator and the original ten leg actuators. The legs may change posture while both sole frames stay anchored to the floor.

| Benchmark | Previous crosswise arm | Revised arm |
|---|---:|---:|
| Forward orientations | 45/64 | 64/64 |
| Broad orientations | 12/32 | 31/32 |
| Both sets | 57/96 (59.4%) | 95/96 (99.0%) |

The crosswise counts predate the leg rule below and have not been recounted.

[Explore the whole robot in 3D](poses.html) by downloading and opening the HTML locally. It includes every target, floor pickup, the overhead reach and the seated parking pose. [Print geometry](mounts.step) · [whole-robot MuJoCo model](robot.xml) · [per-target joint angles](targets.csv) · [prior crosswise results](../crosswise/nominal_targets.csv).

## Mechanical changes

The original head and neck linkage gives way to a shoulder yaw and pitch pair, a crosswise elbow, a forearm roll axis and wrist flex and roll. Reassigning one of the existing upper motors to forearm roll lets the jaws turn about the length of the forearm after the elbow has positioned the wrist. The separated wrist pivots and reshaped links provide clearance for this motion. The stock SO-101 gripper and the rear parking pad are retained.

| Part | Mounting change |
|---|---|
| Torso roof and shoulder mount | The mount bolts to four new Ø3.3 mm through-holes on a 22 × 20 mm rectangle. Its center is at (−92, 0) mm in the torso frame. The six custom parts in the STEP assembly use a common neutral torso datum. The roof STL uses the donor mesh datum. |
| Shoulder and elbow | Shoulder pitch to elbow pivot spacing is 200 mm. The elbow bends about a crosswise axis and drives a 160 mm pivot-to-pivot forearm section. |
| Wrist and gripper | A roll axis precedes two displaced wrist pivots. Each new link connects a four-hole driven horn to a case-lug clamp. The revised links replace the earlier printed carrier and fixed SO-101 forearm fork. |
| Joint limits | Model ranges in chain order are ±150°, −105° to +125°, ±147°, ±155°, ±157°, and −155° to +160°. The target replay enforces a further 2° reserve on every arm and leg joint. Physical limits depend on horn indexing, harness routing and calibration. |

The individual STLs for the six mounts are in millimetres in their own link frames. The STEP assembly contains those same six custom parts. The complete assembly with cases, gripper, drilled roof, legs and trunk is in the MuJoCo model. Hardware fasteners, wiring, elastic deflection and any additional internal reinforcement are outside the model.

## What the score measures

The [original target generator](../study/pose_test.py) samples positions 180–300 mm forward, ±65 mm laterally, and 365–485 mm above the floor. The 64 forward targets use yaw, pitch and roll within ±40°, ±30° and ±45°. The 32 broad targets reuse the first 32 positions with sampled three-dimensional orientations. Each target specifies both position and orientation of the gripper reference frame. The 96 targets are individual endpoints.

A multistart whole-body inverse-kinematics search produced the postures in the CSV. Replay independently reconstructs them with the compiled MuJoCo model and compares them to the original target generator. It checks both sole frames, the joint limits, the projection of the unloaded center of mass inside the support polygon, floor clearance and intersections on the donor and custom triangle meshes. It checks moving arm neighbors too. Every leg link below the hip-yaw link is tested against the trunk and must keep a gap: 0.3 mm for the hip link and 1 mm for the thigh, shin and foot. The hip link's gap is smaller because the stock leg alone brings it within 0.02 mm of the trunk at 23° of hip roll. The exempt interfaces are the bolted deck mount, the native gripper-to-wrist-roll motor interface and the two donor jaw meshes. Across the eleven jaw angles sampled from 0° to 20°, the moving finger clears the other modeled parts and the balance margin stays above 5 mm.

| Independent replay measurement | Worst of the 95 passing endpoints (jaw at 0°) |
|---|---:|
| Gripper position / orientation error | 0.386 mm / 0.171° |
| Sole position / orientation error, either foot | 0.073 mm / 0.025° |
| Unloaded static support margin | 5.58 mm |
| Non-foot geometry above floor | 24.51 mm |
| Hip link to trunk | 0.30 mm |
| Thigh to trunk | 1.05 mm |

**Earlier postures and the leg rule.** The first set of postures passed all 96 targets, but that replay tested only the shins and feet against the trunk. In 33 of those postures a hip link or thigh cut the trunk shell, and 12 more came closer than the required gap. [`resolve.py`](resolve.py) keeps the 51 postures that pass and searches new ones for the rest with the solver of the original target generator. It found a passing posture for 44 of the 45. Target 64 has none: its row in the CSV still holds the earlier posture, and the replay expects exactly that target to fail. The search also showed that three arm joints meet their neighbouring link before the model range ends: shoulder pitch at −53°, wrist flex at −139° and +84°, and wrist roll at −121°.

## Floor pickup, parking and load

With the legs at their nominal stance, the model descends onto a 20 mm cube on the floor about 140 mm ahead of the toes. The jaw gap is 40.83 mm open and 20.00 mm at opposing contact. The replay checks 81 sampled configurations through approach, closure and a 30 mm vertical lift. It assumes the cube stays fixed relative to the gripper after contact. It also checks the overhead reference pose at 651 mm above the floor and 200 mm forward.

The fixed fingertip seats on the rear pad in the parked pose. The model is **578 mm tall parked**, up from 449 mm for the earlier crosswise arm. The replay checks the fold from overhead to the pad through one intermediate pose in 296 samples at intervals of at most 0.5° per joint. The fixed finger first contacts the pad at the final sample. At the seated pose the nominal pad indentation is 0.34 mm and the moving jaw clears the pad plane by 7.15 mm.

The nominal model mass is 2.16 kg. The largest unloaded gravity torque on an arm axis among the 95 passing poses is **1.51 N·m at shoulder pitch**. Link masses use CAD volume and an assumed material density of 1.24 g/cm³. The narrowest balance margin leaves only 0.58 mm above the benchmark cutoff. Continuous payload capacity, thermal duty, printed fits, physical pickup, and a motion planner connecting all 96 grasps remain unverified.

## Reproduce

Run from the repository root with Python 3.12 and a C++ compiler.

```sh
python -m pip install -r crosswise/requirements.txt
python scripts/restore_assets.py
python collision/build_bvh.py
OPENBLAS_NUM_THREADS=1 python coverage/replay.py
```

Run `OPENBLAS_NUM_THREADS=1 python coverage/resolve.py` to search postures for every target whose stored row fails the replay checks. It rewrites those rows of the CSV and leaves the others alone. Add `--starts 4096` to search harder, or `--dry-run` to report without writing.

Run `python coverage/build.py` to regenerate the six solids, drilled roof, print meshes and MuJoCo model. Run `python coverage/viewer.py` to regenerate the offline viewer. The MuJoCo keyframes include target 00, open and closed floor poses, the lifted pose, the rear-pad pose and the overhead pose.
