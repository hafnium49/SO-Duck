# Hollow neck mount and arm links

[Download the offline assembly viewer](./viewer.html) and open it locally. Select a part to inspect its interior section. The viewer includes the original four-hole Duck neck socket, the SO-101 donor geometry, six positioning axes and the gripper.

| Part | Structural change | GitHub mesh preview |
| --- | --- | --- |
| Neck socket mount | A closed band wraps the yaw motor. A hollow upright connects the band to the four original neck screws. | [Neck socket mount](review-meshes/neck-socket-mount.preview.stl) |
| Elbow-to-forearm-roll link | Two fork cheeks seat on the elbow horns. A hollow span joins the fork to a closed band around the roll servo. | [Elbow-to-forearm-roll link](review-meshes/elbow-to-forearm-roll.preview.stl) |
| Forearm-roll-to-wrist link | A hollow bridge and paired webs join the horn screws to the wraparound wrist motor holder. | [Forearm-roll-to-wrist link](review-meshes/forearm-roll-to-wrist.preview.stl) |

The individual review meshes use millimetres in the corresponding robot body frame. They preserve the viewer's 0.1 mm coordinate grid and all of its triangles. Rounding collapses some triangles. Use these files for visual review only. The fabrication STEP and original STL exports are pending.

The viewer stores 96 grasp endpoint poses, 81 floor-pickup frames and 188 folding frames. [`targets.csv`](targets.csv) and [`pickup.csv`](pickup.csv) give the joint angles of the endpoints and of the pickup.

## Coverage with the legs clear of the trunk

The three parts are unchanged in this revision. Only stored poses changed.

| 96 planted-feet targets | Forward | Broad | Total |
| --- | ---: | ---: | ---: |
| First stored poses, as claimed | 64/64 | 32/32 | 96/96 |
| First stored poses, no leg part in the trunk | 38/64 | 7/32 | 45/96 |
| Poses in this revision | 64/64 | 31/32 | 95/96 |

In 51 of the first stored endpoints a hip link or thigh cut the trunk shell. Those 51 have new whole-body poses. The other 45 are kept as stored. The `source` column of `targets.csv` says which is which. The pickup has a new crouch on the same tool path, because the right hip link cut the trunk in every frame of the first one.

Target 89 does not pass. Its best pose with clear legs has a 4.59 mm support margin against the 5 mm requirement. The centre of mass sits too close to the front edge of the soles. Leaning the neck mount 10 mm forward gained less than 1 mm in a trial. Forearm and wrist links 15 % lighter gained about 1.5 mm. Neither change is made here.

**How the new poses were found.** The model that produced the first poses is not in this branch. The arm's joint frames were recovered from the body transforms stored in the viewer. They reproduce all 368 stored frames to 0.0001 mm. The search used the whole-body solver of [`study/pose_test.py`](../study/pose_test.py) with many starts and several hip-yaw ranges, kept the arm inside the angles of the first stored poses, and took for each target the clear pose with the largest support margin. The search scripts are not in this repository. The checks below do not depend on them.

**What still needs the original model.** The joint limits of the arm, the masses of the three parts and full-resolution arm meshes. Until then the support margins rest on estimated masses and the arm clearances on the viewer's display meshes.

## Checks

The "Hollow parts inspection" job runs every check on each pull request. They work from the body transforms and meshes stored in the viewer.

```sh
python -m pip install -r hollow/requirements.txt
python scripts/restore_assets.py
python hollow/inspect_viewer.py --screen --legs --replay
```

- **Files.** Without options the script needs only NumPy. It checks the frame counts, the triangle indices and that each review mesh holds the same triangles as the viewer.
- **Targets and soles.** `--replay` compares the stored tool pose of each endpoint with the [published target generator](../study/pose_test.py) and both soles with their anchors. The tool is within 0.522 mm and 0.129° of every target. The right sole is within 0.143 mm and 0.060° of its anchor. The limits are 1.5 mm and 1° for the tool, 0.2 mm and 0.15° for a sole.
- **Support margin.** `--replay` also estimates the centre of mass. It uses the trunk and leg inertials of `coverage/robot.xml`, the SO-101 link inertials, 55 g per servo and the volume of each review mesh at 1.24 g/cm³. The estimated robot mass is 2.192 kg. 95 of the 96 endpoints keep at least 5 mm of margin. The smallest that holds is 5.57 mm at target 60. The job fails if any endpoint other than 89 drops below 5 mm.
- **Legs and trunk.** `--legs` places the repository's trunk and leg meshes with the stored body transforms. No hip link, thigh, shin or foot meets the trunk in any endpoint, pickup frame or folding frame. The job fails if one does. `coverage/replay.py` tests only the shins and feet against the trunk. The same test on `coverage/targets.csv` finds a thigh in the trunk in 24 of the 96 endpoints of the six-part design, and any leg part in 33.
- **Hollow parts.** `--screen` intersects the viewer's display meshes for every pair of separately moving bodies in all 368 stored frames. The three hollow parts meet no such body except the elbow servo (`forearm_pitch_case`) that the fork cheeks seat on. The job fails if that changes. The display meshes sit on a 0.1 mm grid and overlap at every driven joint, so this is a screen and not a clearance measurement.
- **Joint angles.** 44 of the 96 endpoints hold at least one arm joint at the largest or smallest stored angle. The stored angles span ±108.0° at shoulder yaw, −73.2° to 98.0° at shoulder pitch, ±94.8° at the elbow, ±153.0° at forearm roll, −92.3° to 93.0° at wrist flex and −141.6° to 160.8° at wrist roll. The largest hip yaw is 21.3°. The joint ranges of the revised model are not published here.
- **Pickup approach.** From frame 16 of the pickup clip the fixed jaw lowers along one cube face with less than 0.05 mm of clearance and no overlap. The moving jaw stops 0.10 mm from the opposite face. A physical approach needs clearance on the fixed side.
- **Heights.** The parked robot is 524.8 mm tall and the overhead pose 661.8 mm. The lowest geometry other than the feet is 24.54 mm above the floor across the 96 endpoints and 2.47 mm during the pickup.
