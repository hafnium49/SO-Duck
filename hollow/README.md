# Hollow neck mount and arm links

[Download the offline assembly viewer](./viewer.html) and open it locally. Select a part to inspect its interior section. The viewer includes the original four-hole Duck neck socket, the SO-101 donor geometry, six positioning axes and the gripper.

| Part | Structural change | GitHub mesh preview |
| --- | --- | --- |
| Neck socket mount | A closed band wraps the yaw motor. A hollow upright connects the band to the four original neck screws. | [Neck socket mount](review-meshes/neck-socket-mount.preview.stl) |
| Elbow-to-forearm-roll link | Two fork cheeks seat on the elbow horns. A hollow span joins the fork to a closed band around the roll servo. | [Elbow-to-forearm-roll link](review-meshes/elbow-to-forearm-roll.preview.stl) |
| Forearm-roll-to-wrist link | A hollow bridge and paired webs join the horn screws to the wraparound wrist motor holder. | [Forearm-roll-to-wrist link](review-meshes/forearm-roll-to-wrist.preview.stl) |

The individual review meshes use millimetres in the corresponding robot body frame. They preserve the viewer's 0.1 mm coordinate grid and all of its triangles. Rounding collapses some triangles. Use these files for visual review only. The fabrication STEP and original STL exports are pending.

The viewer stores 96 grasp endpoint poses, 81 floor-pickup frames and 188 folding frames. [`targets.csv`](targets.csv) and [`pickup.csv`](pickup.csv) give the joint angles of the endpoints and of the pickup. [`robot.xml`](robot.xml) is a model of this design reconstructed from the viewer.

## Coverage with a gap between the legs and the trunk

The three parts are unchanged since the first revision. Only stored postures changed.

| 96 planted-feet targets | Forward | Broad | Total |
| --- | ---: | ---: | ---: |
| First stored postures, as claimed | 64/64 | 32/32 | 96/96 |
| First stored postures, no leg link in the trunk | 38/64 | 7/32 | 45/96 |
| First stored postures, with the required gap | 30/64 | 6/32 | 36/96 |
| Postures in this revision, with the required gap | 62/64 | 31/32 | 93/96 |

The required gap is 0.3 mm from each hip link to the trunk and 1 mm from each thigh, shin and foot, as in the [six-part design](../coverage/README.md). The 36 first postures that pass are kept and 57 are new. The `source` column of `targets.csv` says which is which. The pickup has a new crouch on the same tool path.

Three targets have no passing posture. Their rows hold an earlier posture that fails the gap.

- **Targets 12 and 35.** Every balanced candidate brings the left hip link closer to the trunk than 0.3 mm. These targets need more hip roll toward the trunk than the gap allows. The stored rows have hip-link gaps of 0.26 mm and 0.01 mm.
- **Target 89.** No candidate reached the 5 mm support margin, with or without the gap. The centre of mass sits too close to the front edge of the soles. The stored row has a 4.62 mm margin and a thigh 0.003 mm from the trunk. Leaning the neck mount 10 mm forward gained less than 1 mm in a trial, and forearm and wrist links 15 % lighter gained about 1.5 mm. Neither change is made here.

## Reconstructed model

The model that produced the first postures is not in this branch. [`model.py`](model.py) rebuilds one from the viewer and wrote `robot.xml` and the meshes in `meshes/`.

- **Joints.** Each arm joint is a hinge fitted to the stored body transforms. The model reproduces the 96 first endpoints to 0.0000001 in every transform entry.
- **Joint ranges.** The largest and smallest angle stored for each joint, widened by the 2° reserve of the replay. The real ranges are unknown.
- **Meshes.** The arm uses the viewer's display meshes on a 0.1 mm grid, and the three review meshes for the hollow parts. The gripper and jaw use the full SO-101 meshes, whose frames they share. The trunk, legs and parking pad use the repository meshes of the six-part model, with the original roof.
- **Masses.** Estimates: the SO-101 link inertials, 55 g per servo and the enclosed volume of each review mesh at 1.24 g/cm³. The robot comes to 2.192 kg. Screws, horns, wiring and any payload are not counted.

**Still needed from the original design files:** the STEP and STL exports of the three parts, the real joint ranges, the real part masses and full-resolution arm meshes.

## Checks

The "Hollow parts inspection" job runs both scripts on each pull request.

```sh
python -m pip install -r crosswise/requirements.txt
python scripts/restore_assets.py
python collision/build_bvh.py
python hollow/inspect_viewer.py --screen --legs --replay
OPENBLAS_NUM_THREADS=1 python hollow/postures.py
```

[`postures.py`](postures.py) replays the CSV postures on the reconstructed model with the checks of the six-part replay. It fails unless exactly targets 12, 35 and 89 fail.

- **Targets.** Joint ranges with the 2° reserve, tool within 1.5 mm and 1°, soles within 0.2 mm and 0.15°, support margin of at least 5 mm, nothing below the floor, the leg gap, no mesh intersection, and all of that again at ten jaw openings up to 20°. The 93 passing endpoints are within 0.517 mm and 0.109° of their targets. The smallest support margin is 5.55 mm at target 60. The smallest gaps are 0.30 mm at a hip link and 1.19 mm at a thigh. The largest hip yaw is 22.0°.
- **Pickup.** The 81 frames hold one crouch. The support margin stays above 38.99 mm, the hip link gap is 0.32 mm and the thigh gap 6.44 mm. The tool stays on a vertical line.
- **Fold, overhead and parked postures.** They use the reference stance. The support margin stays above 9.58 mm. The fixed finger meets the parking pad only in the last frame.
- **Viewer.** The transforms stored in the viewer equal the model's for every endpoint and pickup frame.
- **Mesh pairs.** Meshes on the two sides of one arm joint touch on the display grid, and the triangle test of the replay counts touching faces as a hit. Those pairs get a test that ignores touching faces. Four of them overlap in every frame, a link seated on the servo horn before it, and are skipped.

`python hollow/postures.py --search` searches postures for every target whose row fails, with [`coverage/resolve.py`](../coverage/resolve.py). `--search-pickup` searches a pickup crouch and `--write-viewer` stores the CSV postures in the viewer.

[`inspect_viewer.py`](inspect_viewer.py) works from the viewer alone. Without options it needs only NumPy.

- **Files.** Frame counts, triangle indices, and each review mesh holding the same triangles as the viewer. It fails if either jaw overlaps the cube in the pickup clip.
- **Mesh screen.** `--screen` intersects the display meshes of every pair of separately moving bodies in all 368 stored frames. It fails if a hollow part meets any body other than the elbow servo its fork seats on, or if any other pair intersects in some frames only. The jaws on the cube, the finger on the parking pad and leg links joined by a joint are expected.
- **Legs.** `--legs` fails if a leg link intersects the trunk in any endpoint, pickup frame or folding frame, on the repository meshes.
- **Targets and margin.** `--replay` compares the stored tool poses and soles with the targets and anchors and estimates the support margin from the stored transforms. Its margins differ from the model's by up to 0.03 mm. It fails if an endpoint other than 89 drops below 5 mm, or if 89 drops below 4 mm.

These are reported and not checked: the heights, the joint-angle statistics and the defects of the review meshes.

- **Joint angles.** 43 of the 96 endpoints hold at least one arm joint at the largest or smallest stored angle. The stored angles span ±108.0° at shoulder yaw, −73.2° to 98.0° at shoulder pitch, −94.9° to 94.8° at the elbow, ±153.0° at forearm roll, −92.6° to 93.0° at wrist flex and −141.6° to 160.8° at wrist roll.
- **Pickup approach.** From frame 16 of the pickup clip the fixed jaw lowers along one cube face with less than 0.05 mm of clearance. The moving jaw stops 0.11 mm from the opposite face. A physical approach needs clearance on the fixed side.
- **Heights.** The parked robot is 524.8 mm tall and the overhead pose 661.8 mm. The lowest geometry other than the feet is 24.54 mm above the floor across the 96 endpoints and 2.47 mm during the pickup.

Not checked anywhere: joint torques, motion between the endpoints, and any payload in the gripper.
