# Hollow neck mount and arm links

[Download the offline assembly viewer](./viewer.html) and open it locally. Select a part to inspect its interior section. The viewer includes the original four-hole Duck neck socket, the SO-101 donor geometry, six positioning axes and the gripper.

| Part | Structural change | GitHub mesh preview |
| --- | --- | --- |
| Neck socket mount | A closed band wraps the yaw motor. A hollow upright connects the band to the four original neck screws. | [Neck socket mount](review-meshes/neck-socket-mount.preview.stl) |
| Elbow-to-forearm-roll link | Two fork cheeks seat on the elbow horns. A hollow span joins the fork to a closed band around the roll servo. | [Elbow-to-forearm-roll link](review-meshes/elbow-to-forearm-roll.preview.stl) |
| Forearm-roll-to-wrist link | A hollow bridge and paired webs join the horn screws to the wraparound wrist motor holder. | [Forearm-roll-to-wrist link](review-meshes/forearm-roll-to-wrist.preview.stl) |

The individual review meshes use millimetres in the corresponding robot body frame. They preserve the viewer's 0.1 mm coordinate grid and all of its triangles. Rounding collapses some triangles. Use these files for visual review only. The fabrication STEP and original STL exports are pending.

The viewer stores 96 grasp endpoint poses, 81 floor-pickup frames and 188 folding frames. The stored poses support visual inspection. Independent replay requires the matching revision of the model and its validation inputs.
