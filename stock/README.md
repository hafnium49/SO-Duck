# One connector for the original SO-101 and Duck neck pitch

The complete SO-101 follower mounts on the moving sheets of the first Open Duck Mini V2 neck joint. The load path runs from the SO-101 base through the printed deck and its two integral cheeks, into the original Duck sheets and lower neck pivot. The first servo tilts the whole assembly. The SO-101 still provides shoulder pan, shoulder lift, elbow flex, wrist flex, wrist roll and jaw opening. Together with the ten Duck leg actuators, the robot has 17 actuators.

[Editable connector (STEP)](connector.step) · [Print mesh (STL, millimetres)](connector.stl) · [Whole-robot MuJoCo model](robot.xml) · [Offline 3D viewer](poses.html)

## Assembly

| Qty | Part | Mounting detail |
|---:|---|---|
| 1 | Printed connector | One solid with a 5 mm deck, two 5.5 mm cheeks and 2 mm root fillets. Solid CAD mass is 36.45 g at 1.24 g/cm³. |
| 4 each | M2 × 12 socket-head screws and M2 nuts | Install from the outside of the Duck sheets. Heads sit in the existing counterbores, with the nuts inside the cheeks. |
| 4 each | M3 × 30 socket-head screws, 12 mm OD × 1 mm washers and M3 nuts | Insert upward through the deck and original SO-101 base. Seat each wide washer and nut on the upper face of the base flange. |

1. Retain the Duck neck pitch motor, its lower driven and support attachments, and both original neck sheets. Remove the upper head joints and their attachments from the top of the sheets. The original head and its three axes leave the moving assembly.
2. Fit the integral cheeks between the sheets. The 31.7 mm external width gives 0.15 mm nominal clearance per side. The left and right screw centerlines are respectively 37.00 and 33.25 mm above the lower pivot. Both fore–aft hole spacings are 20.50 mm. Fit the four M2 screws before installing the SO-101.
3. Seat the original SO-101 base ribs on the deck with its stock zero configuration pointing forward. Use the four existing diamond-shaped flange openings for the M3 screws. In the stock base coordinate system the hole centers are at (−3.398, ±36.276) and (+40.420, ±28.460) mm. The wide washers bridge the openings. All arm links, the base feet, motor mounts and gripper retain their stock geometry.
4. The installed neck pitch setting is **−20° to +30°**. The original upper limit is +65°. The complete SO-101 base first intersects the Duck trunk at +31° in the half-degree sweep. The model applies the installed limit. A hardware controller needs the same limit.

The circular screw gauges clear both donor interfaces. The computed washer bearing areas are 88.4–90.9 mm² each on the donor mesh. The nominal rib/deck contact area is 1,170 mm². These are digital fit measurements. Printed tolerances, cable routing, neck-bearing loads, fastener preload, fatigue and motor thermal performance still need hardware validation. There is no supported payload rating.

## Whole-body checks

| Test | Verified result |
|---|---|
| Original 64 forward-facing grasp targets | 52 of 64 (81.25%) |
| Original 32 broadly oriented grasp targets | 7 of 32 (21.88%) |
| Both groups combined | **59 of 96 (61.46%)** |
| Overhead | Gripper reference point 650.6 mm above the floor at 200.0 mm forward, nominal standing pose |
| Floor pickup | Both jaws surround a 20 mm cube centered 182.1 mm forward. Gap goes from 40.83 to 20.00 mm. The 81 sampled frames include a 30 mm vertical lift under an assumed no-slip grasp. Both soles remain planted in a crouch. |
| Folded arm | One clear, balanced posture, 495.1 mm tall versus 574.3 mm in the stock-zero standing pose. The fingertips are airborne. A path to that pose and load-bearing parking were not verified. |
| Static neck load | Peak unloaded gravity torque across the 59 stored grasp poses is 0.742 N·m, using donor and estimated masses. |

Every successful target has a reproducible joint vector in [targets.csv](targets.csv). Each requires the specified position **and** orientation with both feet planted, 2° joint reserve on each pose axis, at least 5 mm static support margin and a mesh collision screen. Maximum recorded residuals were 0.088 mm and 0.026°, with a minimum static margin of 7.54 mm. A blank row means the numerical search did not find an accepted solution. The counts measure pose coverage on this finite bank. Physical grasp success, payload and continuous trajectories between these endpoints are separate questions.

## Reproduction

From the repository root on Linux with Python 3.12 and a C++17 compiler:

```sh
python -m pip install -r stock/requirements.txt
python scripts/restore_assets.py
python collision/build_bvh.py
python stock/verify.py
```

To regenerate the CAD and assembled MJCF, run `python stock/build.py`. To regenerate the self-contained viewer, run `python stock/viewer.py`. The viewer includes the whole robot, the isolated and exploded connector, all verified grasp endpoints and the floor pickup replay. Read [geometry provenance](../docs/PROVENANCE.md) for pinned donor revisions.
