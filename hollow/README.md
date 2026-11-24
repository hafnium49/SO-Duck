# Hollow neck mount and arm links

[Download the offline assembly viewer](./viewer.html) and open it locally. Select a part to inspect its interior section. The viewer includes the original four-hole Duck neck socket, the SO-101 donor geometry, six positioning axes and the gripper.

| Part | Structural change | GitHub mesh preview |
| --- | --- | --- |
| Neck socket mount | A closed band wraps the yaw motor. A hollow upright connects the band to the four original neck screws. | [Neck socket mount](review-meshes/neck-socket-mount.preview.stl) |
| Elbow-to-forearm-roll link | Two fork cheeks seat on the elbow horns. A hollow span joins the fork to a closed band around the roll servo. | [Elbow-to-forearm-roll link](review-meshes/elbow-to-forearm-roll.preview.stl) |
| Forearm-roll-to-wrist link | A hollow bridge and paired webs join the horn screws to the wraparound wrist motor holder. | [Forearm-roll-to-wrist link](review-meshes/forearm-roll-to-wrist.preview.stl) |

The individual review meshes use millimetres in the corresponding robot body frame. They preserve the viewer's 0.1 mm coordinate grid and all of its triangles. Rounding collapses some triangles. Use these files for visual review only. The fabrication STEP and original STL exports are pending.

The viewer stores 96 grasp endpoint poses, 81 floor-pickup frames and 188 folding frames. The stored poses support visual inspection. Independent replay requires the matching revision of the model and its validation inputs.

## Open checks

`python hollow/inspect_viewer.py` reads the viewer and the three review meshes. It needs NumPy. It checks the frame counts, the triangle indices, that each review mesh holds the same triangles as the viewer, and the sole closure. It is not a replay of the design. The figures below come from its output. The "Hollow parts inspection" job runs it with both options on every pull request.

```sh
python -m pip install -r hollow/requirements.txt
python scripts/restore_assets.py
python hollow/inspect_viewer.py --screen --legs
```

- **No replay yet.** The stored poses come from a model revision that is not in this branch. The "Grasp coverage replay" job still runs `coverage/robot.xml`, the earlier six-part design. That job passing says nothing about the hollow parts. Support margins, joint limits and target errors of the stored poses are unchecked.
- **Thighs and hip links intersect the trunk.** `--legs` places the repository's trunk and leg meshes with the body transforms stored in the viewer. A thigh cuts the trunk in 34 of the 96 endpoints: `knee_and_ankle_assembly` in 12 and `knee_and_ankle_assembly_3` in 22. Counting the hip links too, 51 of the 96 endpoints have a leg part in the trunk. Each of the 34 has a hip yawed 18.9° to 28.0°. The right hip link also cuts the trunk in all 81 pickup frames. The fold is clear. `coverage/replay.py` tests only the shins and feet against the trunk, so it does not see this. The same test on `coverage/targets.csv` finds a thigh in the trunk in 24 of the 96 endpoints of the six-part design. These endpoints need new solutions from a solver that tests the hip links and thighs against the trunk. The job reports them and does not fail on them.
- **Hollow parts.** `--screen` intersects the viewer's display meshes for every pair of separately moving bodies in all 368 stored frames. The three hollow parts meet no such body except the elbow servo (`forearm_pitch_case`) that the fork cheeks seat on. The job fails if that changes. The display meshes sit on a 0.1 mm grid and overlap at every driven joint, so this is a screen and not a clearance measurement.
- **Sole closure.** Against the stance of the pickup clip, the right sole of the 96 endpoints is off by at most 0.181 mm and 0.060° (target 08). The replay tolerance is 0.2 mm and 0.15°. The six-part design reported 0.052 mm and 0.016°.
- **Joint angles.** 36 of the 96 endpoints hold at least one arm joint at the largest or smallest stored angle. The stored angles span ±108.0° at shoulder yaw, −79.8° to 98.0° at shoulder pitch, ±94.8° at the elbow, ±153.0° at forearm roll, ±93.0° at wrist flex and −141.6° to 160.8° at wrist roll. The joint ranges of the revised model are not published here.
- **Pickup approach.** From frame 16 of the pickup clip the fixed jaw lowers along one cube face with less than 0.05 mm of clearance and no overlap. The moving jaw stops 0.10 mm from the opposite face. A physical approach needs clearance on the fixed side.
- **Heights.** The parked robot is 524.8 mm tall and the overhead pose 661.8 mm. The lowest geometry other than the feet is 24.54 mm above the floor across the 96 endpoints and 2.47 mm during the pickup.
