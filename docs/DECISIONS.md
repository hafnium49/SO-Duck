# Design decision log

This page records the earlier longitudinal-roll design. The [current crosswise assembly](../crosswise/README.md) uses the stock SO-101 forearm.

| Decision | Rationale | Unresolved consequence |
|---|---|---|
| Preserve all four Duck neck joints | Use pre-existing motors; gain yaw/pitch/roll repertoire | Torques and gear speed may be limiting |
| Add only SO-101 wrist flex and wrist roll, plus jaw | Achieve a nominal 6D tool-pose chain with minimal new servo hardware | No global reachability guarantee |
| Mount forearm directly to terminal servo case and opposite idler | Avoid flex from original head shell | Must measure load path, clearance, bearing alignment |
| One custom case-mounted forearm | Minimize custom printed part count | Custom design still untested for printability, flex, cracking |
| Terminal roll fore-aft, wrist-flex transverse | Separate roll/pitch axes; forward/backward wrist bend | Constraint-aware conditioning still varies by posture |
| Keep legs stock, leverage stance change | Reuse native lower-body hardware and control possibilities | Stepping transition with arm/payload is not demonstrated |
| Public research record with upstream notices | Support inspection and attribution | No blanket license for original materials yet |
