# SO-Duck architecture

**Status: research CAD and kinematic model only; no fully assembled, hardware-validated robot.**

## Actuation
- 10 Open Duck Mini V2 leg actuators (5 per leg).
- 4 original Duck neck/head actuators: neck pitch, head pitch, head yaw, head roll.
- 2 donor SO-101 wrist joints: wrist flex (sagittal pitch), wrist roll.
- 1 donor SO-101 gripper-opening actuator.
- **17 total, of which 16 change the TCP pose or base posture.** Optional Duck antenna servos are excluded.

## Proposed chain

Duck torso -> neck pitch -> head pitch -> head yaw -> terminal head roll -> case-mounted forearm -> SO-101 wrist flex -> SO-101 wrist roll -> 1-DoF gripper.

The terminal horn stays fixed relative to the native yaw-to-roll carrier; the terminal servo case moves with the one-piece forearm. The original Duck head shell does **not** carry the manipulation load. The native rear bearing supports the opposite end.

The stock forward/backward wrist flex needs a transverse shaft; terminal roll is longitudinal. This layout is a proposed compromise between joint independence, tip reach and minimizing custom parts.

## Whole-body study
The six upper chain joints alone do not guarantee arbitrary 6D pose at every tip position. Leg joints add constrained torso mobility during double support. Stepping expands reachable world-space goals, but is a distinct motion-planning task and is not an additional instantaneous Jacobian column.

See WHOLE_BODY_STUDY.md for the archived sample/IK experiment and VALIDATION.md for the interpretation limits.
