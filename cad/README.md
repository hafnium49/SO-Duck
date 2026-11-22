# Case-mounted forearm

The editable part is Case_mounted_forearm.step. It represents a **one-piece forearm** that couples the moving terminal-servo case and its opposite rear support bearing to the SO-101 distal wrist assembly. The thin Duck head shell is removed from the load path.

Coordinates in the proposed print frame: +X forward, +Y left, +Z up. Terminal neck roll is about forward +X, wrist flex about transverse +Y, and wrist roll about the gripper's length. The gripper's jaw is independently actuated.

The design originated as a CadQuery-generated STEP and was compared with source Duck and SO-101 mechanical reference models. **Its CAD checks are not manufacturing certification**: the carrier fit, servo case mounting bosses, rear bearing engagement, cable clearance, print anisotropy, real servo speeds and sustained torques have not been measured on a physical fused unit.

The upstream donor geometry comes from TheRobotStudio/SO-ARM100 and apirrone/Open_Duck_Mini. Upstream license texts and citations are recorded in ../LICENSES and ../docs/PROVENANCE.md.

A mesh export can be obtained from the STEP using FreeCAD or a compatible STEP-to-STL CAD exporter. Prefer checking critical mounting holes against actual hardware before fabrication.
