# Forearm CAD and interface

This page records the earlier longitudinal-roll design. The [current crosswise assembly](../crosswise/README.md) uses the stock SO-101 forearm.

The candidate is a **single case-mounted forearm** connecting the Duck's terminal neck roll actuator to the SO-101 wrist-flex joint. The Duck head shell is omitted from the load path; its driven-horn side and opposite idler-bearing support remain in the design. The wrist-flex axis is transverse to the robot's forward direction, so the gripper bends **forward/backward**.

- `Case_mounted_forearm.step`: editable STEP geometry from the study.
- `Case_mounted_forearm.stl`: 3D-printing mesh derivative.
- `Forward_neck_and_wrist.glb`: neutral assembly reference for visualization (not dynamic physics).
- `Forward_mount.png`: reference rendering.
- `under_arm_so101_v1.step`: upstream SO-101 forearm reference. Retained for provenance and fit comparison.
- `build_forearm.py`: generating script; depends on CadQuery.
- `Mount_fit.md`: orientation and nominal clearance; these are CAD-level, not test-bench measurements.

**Critical:** The exact moving-case screw lengths, thread engagement, passive bearing alignment, manufacturing anisotropy, stiffness and dynamic overhung loads have not been verified on physical parts. Rebuild/inspect the geometry before attempting hardware assembly.
