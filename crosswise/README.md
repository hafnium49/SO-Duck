# Crosswise forearm assembly

The yaw servo carries a single printed bracket. The bracket holds the existing terminal STS3215 case across the robot. Its driven horn and opposite idler carry the two cheeks of the **stock SO-101 lower arm**. The wrist motor holder, wrist flex, wrist roll and jaw keep their donor geometry and relative frames. The two pitch hinges are parallel at neutral yaw and 135 mm apart. The former longitudinal head-roll motion is lost. The distal wrist roll remains available.

## Fit and assembly

| Interface | As modeled |
|---|---|
| Yaw case to carrier | Four rear case fasteners in the native yaw position |
| Carrier to terminal case | Four case fasteners through 2.5 mm bores. The driven horn is at (97.15, −18.20, 129.06) mm in the Duck print datum, with positive pitch along +Y |
| Terminal horns to fork | Stock 36.4 mm cheek spacing. The donor horn pattern takes four M3 × 6 mm screws per cheek. Upright fork at model zero |
| Wrist and gripper | Stock lower-arm wrist holder and motors. The wrist-flex axis passes through (91.95, −18.20, 263.96) mm in the print datum |
| Rear parking contact | Bond the flat underside of the 10 mm diameter, 4 mm thick compliant pad at (−104, +23) mm on the existing rear roof, z = 107.891 mm **in the torso frame**. The footprint lies 5.0 mm inside the nearest roof edge or opening. The deck is undrilled |

The [assembly STEP](connector_assembly.step) contains all four located connector solids. The [carrier STEP](carrier.step) and [STL](carrier.stl) use mm in the common Duck print datum. The [pad STL](park_pad.stl) has its flat mounting face at local z = 0. The study's moving-case forearm, separate rear bearing and forward cradle are omitted from this assembly. The two material assumptions used for mass estimates are 1.24 g/cm³ for the carrier and 1.20 g/cm³ for the pad. The model assigns 55 g to each of the two retained cases. Electronics with unavailable outer meshes keep their donor body inertia.

### Actuation and routing

The existing terminal motor takes the geometric coordinate `forearm_pitch` in the former `head_roll` actuator slot. The ten leg motors, neck pitch, head pitch, yaw, two wrist motors and jaw retain their roles. Positive forearm pitch tips the upright fork toward the nose at neutral yaw. Set the neutral horn index using the located assembly. The MJCF uses radians and torque inputs. The pose CSVs use the units in their headers.

The **model travel is −40° to +96°**. A one-degree sweep of the entire lower arm against the carrier and both cases clears −41° through +99°. Samples at −42° and +100° interfere. The narrower model interval also respects the SO-101 donor elbow's ±1.69 rad range. These are geometric limits. Hardware zero, direction, case-screw fit, cable bend radius and slack across yaw, forearm pitch and wrist roll need physical measurement. The bus crosses the pitch joint between the fixed carrier and the moving wrist holder. The collision model contains rigid parts and omits the harness.

## Verified poses and paths

All heights below use the floor plane through the planted soles. Static margins use the horizontal projection of the unloaded MuJoCo subtree COM and the nominal two-foot support polygon.

| Case | Geometric result |
|---|---|
| Empty arm folds toward the rear | 302 configurations from upright to seated, with at most 0.5° per joint between samples. Robot height drops from 658.6 to 448.6 mm. The folded arm extends 34 mm beyond the front of the torso shell. Minimum unloaded support margin is 9.9 mm |
| Final contact | The fixed fingertip indents the modeled uncompressed pad by 0.34 mm. The moving finger stays 4.04 mm above the pad plane. The fork spans over the body and the parked wrist faces down. This is a powered parked pose with a compliant contact. There is no latch or measured load-sharing force |
| Neutral to floor approach | 186 sampled configurations to the open-jaw approach. Reversing the fold returns to this neutral pose |
| 20 mm cube pickup | 81 approach, closure and lift configurations. Open clearance across the cube is 40.83 mm. The jaw angle at side contact is +3.33°. Minimum nonfoot floor clearance is 2.32 mm. The 30 mm lift assumes no slip after contact |
| Overhead reach | At x = 200 mm on the torso centerline, TCP height is 487.35 mm above the torso origin and 650.60 mm above the floor. The earlier roll design reached 474.89 mm above the torso origin at the same location, a **12.46 mm** difference for these poses |

The model reaches Jacobian rank six away from neutral. At the upright zero pose its tool Jacobian has rank five. In the unloaded free-arm model, gravity alone requires about 0.57 N·m at the first neck pitch joint at cube pickup and 0.36 N·m at the crosswise joint in the folded pose before taking credit for pad support. These static estimates exclude acceleration, servo friction, grip forces and pad reaction. No physical load capacity or power-off retention has been demonstrated.

### Nominal grasp bank

The same 96 world targets, foot anchors, 24 IK starts, 100 iterations and 2° joint-limit reserve were used for the two designs. The crosswise results re-solve the joint angles, confirm tool and foot error against the shipped MJCF, and check mesh intersections, the floor and a 5 mm static margin. One candidate per target is screened after the IK search. Overlapping counts below use matching target IDs. Counts do not establish a workspace boundary.

| Target bank | Earlier roll design | Crosswise | Passed by both |
|---|---:|---:|---:|
| Forward grasps | 45/64 | 45/64 | 39 |
| Broad orientations | 9/32 | 12/32 | 7 |

The [per-target results](nominal_targets.csv) include joint positions, pose and foot errors, margin and interfering pairs. The earlier replant and single-support results apply to the earlier design.

## Reproduce

From the repository root with Python 3.12 and a Linux C++17 compiler:

```sh
python -m pip install -r crosswise/requirements.txt
python scripts/restore_assets.py
python collision/build_bvh.py
python crosswise/verify.py
python crosswise/targets.py /tmp/so-duck-crosswise-targets.csv
```

Load [robot.xml](robot.xml) in MuJoCo after restoring the assets. Select the `ready`, `folded`, `pickup_open`, `pickup_contact`, `lifted` or `high_reach` keyframe. The [offline viewer](../SO_Duck_pose_explorer.html) includes its own geometry and opens without dependencies. Its meshes are simplified for display. The collision replay uses the full native STL triangles, including both jaw surfaces.

`build_model.py` rebuilds the MJCF and printable pad using the connector assembly, donor XML and pose definitions. `export_explorer.py` rebuilds the viewer from that model. The Duck and SO-101 geometry sources and license notices are in [Provenance](../docs/PROVENANCE.md).
