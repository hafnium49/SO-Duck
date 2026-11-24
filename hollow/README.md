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

Run `python hollow/inspect_viewer.py` to read the viewer and the three review meshes. It needs NumPy. It confirms the frame counts, the triangle indices and that each review mesh holds the same triangles as the viewer. It is not a replay of the design. The figures below come from its output.

- **No replay yet.** The stored poses come from a model revision that is not in this branch. The "Grasp coverage replay" job still runs `coverage/robot.xml`, the earlier six-part design. That job passing says nothing about the hollow parts. Support margins, joint limits and target errors of the stored poses are unchecked.
- **Sole closure.** Against the stance of the pickup clip, the right sole of the 96 endpoints is off by at most 0.181 mm and 0.060° (target 08). The replay tolerance is 0.2 mm and 0.15°. The six-part design reported 0.052 mm and 0.016°.
- **Joint angles.** 36 of the 96 endpoints hold at least one arm joint at the largest or smallest stored angle. The stored angles span ±108.0° at shoulder yaw, −79.8° to 98.0° at shoulder pitch, ±94.8° at the elbow, ±153.0° at forearm roll, ±93.0° at wrist flex and −141.6° to 160.8° at wrist roll. The joint ranges of the revised model are not published here.
- **Heights.** The parked robot is 524.8 mm tall and the overhead pose 661.8 mm. The lowest geometry other than the feet is 24.54 mm above the floor across the 96 endpoints and 2.47 mm during the pickup.
- **Mesh screen.** `python hollow/inspect_viewer.py --screen` also needs python-fcl. It intersects the viewer's display meshes for every pair of separately moving bodies in all 368 stored frames. The three hollow parts meet no such body except the elbow servo (`forearm_pitch_case`) that the fork cheeks seat on. The thigh meshes meet the trunk in 34 of the 96 endpoints: `knee_and_ankle_assembly` in 12 and `knee_and_ankle_assembly_3` in 22. They are clear in every other stored frame. `coverage/replay.py` does not test the thighs against the trunk, so the replay of this design should add those pairs. The display meshes sit on a 0.1 mm grid and overlap at every driven joint, so this result needs confirming on the collision meshes.
