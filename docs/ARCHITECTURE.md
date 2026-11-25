# Proposed mechanical and control architecture

This page records the earlier longitudinal-roll design. The [current crosswise assembly](../crosswise/README.md) uses the stock SO-101 forearm.

```text
Open Duck Mini V2 hips, legs and feet (10 DoFs)
             │
Duck torso: battery, power, IMU, controller (floating, unactuated)
             │
Duck neck pitch → head pitch → head yaw → terminal roll (4 DoFs)
             │
case-mounted one-piece forward SO-Duck forearm (custom part)
             │
SO-101 wrist flex → SO-101 wrist roll (2 DoFs)
             │
SO-101 jaw gripper (1 DoF)
```

**Total: 17 active joints, of which 16 influence the tool's world pose, with constraints imposed by foot contact.** Optional Duck antenna actuators are not retained in this proposed manipulator configuration.

The driving horn and opposite idler support the native Duck terminal roll. The concept bolts the forearm to the *moving servo case*, not to a head shell. The wrist-flex axis runs left-right so the tool bends forward/backward in the sagittal plane. The wrist-roll axis and gripper retain the SO-101 donor geometry. An assembled physical fit, exact hardware screws, load paths and cable routes are still to be verified.

The controller is conceptually hierarchical: feet/stance planning and stabilizing locomotion first, then constrained whole-body IK for the gripper while both feet remain supported. The old Duck walking policy is **not validated** for added distal inertia and moving arm mass, and the study does **not** provide an integrated gait/grasp controller.

See [Mount fit](../cad/Mount_fit.md), [Methods](METHODS.md), and [Validation](VALIDATION.md).
