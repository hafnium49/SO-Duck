# O90 endpoint collision screen

The three CSVs give one result per representative grasp solution for each stance policy. Every check uses the same +90° forward-clocked orthogonal forearm STL (SHA-256 80d7e16707ae3ff8cacb3941a8ed0778c8005d5d39fb6fced6dd6cad2007a5b8). IDs start at 0. The bank labeled front has 64 targets. The bank labeled all_directions has 32. A valid row for the nominal model requires pose_and_feet_ok=1, static_margin_5mm_ok=1 and hit_count=0. Stock moving joints use the joint angles in the corresponding poses CSV, including all ten leg coordinates. The jaw remains at its zero coordinate.

| Balanced grasp endpoints passing this screen | Front | All directions |
|---|---:|---:|
| Original two-foot stance | 45/64 | 9/32 |
| Replanted with 220 mm stand-off | 43/64 | 21/32 |
| Replanted with 270 mm stand-off | 59/64 | 26/32 |
| At least one of the above | 62/64 | 26/32 |

Numbers refer to the finite bank and the solver's selected representative per target. Different inverse-kinematic solutions can have different contact results. A replant is an endpoint assumption. The transitions between the original and replanted stance require a separate gait study.

## What was tested

The program uses 2,383,480 native STL triangles grouped into 19 rigid assemblies. It checks triangle-surface intersections under 146 relative-transform pairs and checks the lowest vertices of all nonfoot groups against the floor, with a 0.1 mm floor tolerance. Trunk and leg external shells, neck sheets, brackets, available servo/horn pieces, the actual replacement link, the distal SO motor, wrist-flex link, wrist holder, gripper and closed jaw are included.

The first 73 pairs compare hybrid arm members against the rigid Duck trunk, neck and ten leg groups and selected non-mating pairs within the arm. The other 73 compare structural members of the two legs against one another, non-neighboring links along each leg, neck against leg groups, knees and ankles against the trunk, head-pitch carrier against trunk and yaw against neck-pitch sheets. The pair list is generated explicitly in screen.py. Within these pairs a hit means at least one triangle-surface intersection. Intentional neighboring hinge surfaces and upstream-listed exceptions are excluded from the pairs. Coplanar contact can register as an intersection. No separate positive feature clearance, fully enclosed solid, small unmodeled fastener, harness or payload test is provided. Original ankle/foot rigid groups are excluded from the floor test because the articulated models maintain sole contact.

The saved column hybrid_arm_intersecting_pairs isolates the first 73 pair checks. The intersecting_pairs column includes all 146 pair checks and floor hits. The ground column gives the lowest point on any nonfoot exterior group. contact_details.csv names the individual intersecting rigid mesh parts on nominally balanced endpoints. These are endpoint checks of a nominal rigid assembly. A collision-free path and a hardware clearance allowance require additional testing.

## Reproduction

The inputs folder contains the source rigid mesh geometry and link placements. Place the sibling search and legs folders from the analysis bundle beside this folder. On Linux, install NumPy, SciPy and the VTK Python wheel with a C++17 compiler. Then run the following in this directory:

```sh
python build_bvh.py
python screen.py ../study/poses_O90.csv ../study/poses_Replant22.csv ../study/poses_Replant27.csv
```

The C++ helper prunes triangle pairs with bounding-volume hierarchies and delegates narrow-phase triangle intersection to VTK. Only the source should be needed when rebuilding the native helper. Default SI inputs are converted to millimetres for the triangle test. Source geometry credit and license information appears in inputs. Duck assembly inputs are from Open_Duck_Mini v2 and Open_Duck_Playground, and the donor arm inputs are from TheRobotStudio/SO-ARM100.
