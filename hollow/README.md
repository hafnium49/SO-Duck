# Hollow neck mount and arm links

Three printed parts put a six-axis SO-101 arm on the neck socket of an Open Duck Mini V2. They are drawn in CadQuery from the public SO-101 solids and have print files in [`parts/`](parts/). They are not yet printed. [BUILD.md](BUILD.md) lists the parts and screws and gives the printing and assembly steps.

[Download the offline assembly viewer](./viewer.html) and open it locally. Select a part to see it alone or cut open.

| Part | What it is | Print files |
| --- | --- | --- |
| Neck socket mount, 31.3 g | A plug fills the Duck's neck socket and takes its four screws. A channel upright leans back to the SO-101 base motor holder, which holds the shoulder yaw servo. | [STL](parts/neck_socket_mount.stl) · [STEP](parts/neck_socket_mount.step) |
| Elbow-to-forearm-roll link, 38.5 g | The two round ends of the SO-101 under arm sit on the elbow servo. Two plates run outside the upper arm to the full SO-101 seat that holds the forearm roll servo. | [STL](parts/elbow_to_forearm_roll.stl) · [STEP](parts/elbow_to_forearm_roll.step) |
| Forearm-roll-to-wrist link, 24.6 g | One round end of the SO-101 under arm sits on the roll servo's horn. A three-sided channel carries the full SO-101 seat that holds the wrist flex servo. | [STL](parts/forearm_roll_to_wrist.stl) · [STEP](parts/forearm_roll_to_wrist.step) |

Masses are as printed with the SO-101's own settings (see [BUILD.md](BUILD.md)). The files use millimetres in the frame of the model body that carries the part. [`robot.xml`](robot.xml) is the model built from them.

## What changed, and why

The first hollow design had only open review meshes, which cannot be printed, and its arm loaded the shoulder pitch servo with up to 1.17 N m empty and 1.63 N m with 100 g in the gripper. The stock SO-101 arm, which uses the same servo, never asks more than 0.864 N m empty and 1.267 N m with 100 g of it.

- **Printable solids.** [`cad/parts.py`](cad/parts.py) draws each part as one closed solid with walls of 1.6 mm or more over 99 % of its surface. Where a part holds a servo or sits on a horn it reuses the SO-101 feature that does the same in the stock arm.
- **A shorter arm.** The forearm is 40 mm shorter. The wrist flex axis sits 40 mm closer to the roll axis and 12 mm further back along it. [`cad/lengths.py`](cad/lengths.py) chose the smallest shortening that kept enough targets within the load limit.
- **Full stock servo seats.** Each link holds its servo in the stock SO-101 seat: the end of the under arm and the wrist motor holder, printed as one band, with all four screws.
- **Weighed like the stock parts.** Every printed part is weighed as printed: 0.9 mm of solid wall on each surface and 15 % infill inside. That rule gives the six stock SO-101 bodies the plastic masses that the SO-101 model gives them, within 4 g each. The custom parts were first weighed as solid plastic, about twice as heavy per cm³ as the stock parts they are compared with.
- **The neck servo leaves.** The mount's plug sits where the Duck's neck servo sat and uses that servo's four screws.

The review meshes of the first design are still in [`review-meshes/`](review-meshes/) and `meshes/` for comparison. Nothing uses them except the earlier reconstruction in [`model.py`](model.py).

## Shoulder load and coverage

The load is the gravity torque on the shoulder pitch servo in a held posture. The limit is the worst that the stock SO-101 arm asks of the same servo over its whole joint range: **0.864 N m empty and 1.267 N m with 100 g** at the tool point. [`loads.py`](loads.py) searches that worst case and computes both.

| 96 planted-feet targets | Forward | Broad | Total |
| --- | ---: | ---: | ---: |
| Reached with every check passed | 64/64 | 27/32 | 91/96 |
| Of those, within the load limit | 58/64 | 27/32 | 85/96 |

**85 of the 96 targets are reached within the load limit.** The worst of the 85 asks 0.855 N m empty and 1.143 N m with 100 g. For each target [`targets.csv`](targets.csv) holds the passing posture with the lowest loaded shoulder torque that the search found.

Six targets are reached but stay over the limit:

| Target | Empty | With 100 g |
| ---: | ---: | ---: |
| 5 | 0.929 N m | 1.308 N m |
| 18 | 0.898 N m | 1.233 N m |
| 21 | 0.869 N m | 1.219 N m |
| 50 | 0.907 N m | 1.274 N m |
| 53 | 0.906 N m | 1.254 N m |
| 60 | 0.879 N m | 1.197 N m |

Five targets have no passing posture. Their rows in `targets.csv` hold a posture of the earlier design that no longer reaches the target.

- **Target 74.** The shorter arm does not reach it inside the joint ranges.
- **Targets 64 and 89.** Every posture that reaches them lacks the 5 mm support margin or goes below the floor.
- **Target 93.** No posture with the support margin keeps the gap between the legs and the trunk.
- **Target 78.** Hundreds of postures have the margin and the gap. In each one two parts of the robot intersect.

The first hollow design reached 93 targets without a load limit. This one gives up two targets to reach and six to the limit.

## Other postures

- **Pickup.** One crouch holds for the whole clip while the tool goes down a vertical line over a 20 mm cube, closes and lifts 30 mm. The support margin stays above 32.8 mm.
- **Overhead.** The tool point reaches **800 mm** above the floor on the centre line, 200 mm ahead of the trunk. That is the highest whole millimetre at which a posture passes at the reference stance.
- **Parked.** The fixed fingertip rests 0.08 mm into the roof pad and the moving jaw stays 3.28 mm above it. The parked robot is 520 mm tall.
- **Fold.** 229 frames lead from overhead to parked through one waypoint, at most half a degree per joint and frame. The support margin stays above 5.4 mm. The pad is touched only in the last frame.

The overhead, parked and waypoint postures are in [`overhead.csv`](overhead.csv), [`park.csv`](park.csv) and [`fold_waypoint.csv`](fold_waypoint.csv), the pickup in [`pickup.csv`](pickup.csv).

## The model

[`cad/build_parts.py`](cad/build_parts.py) writes the print files and `robot.xml`.

- **Joints.** The frames of the first design with the chosen lengths applied ([`cad/params.py`](cad/params.py)).
- **Joint ranges.** Shoulder yaw ±108°, shoulder pitch −81° to 100°, elbow ±95°, forearm roll ±155°, wrist flex ±95°, wrist roll −143° to 162°. The three joints next to a printed part stay inside the sweep that its part test covers. Inside these ranges some stock meshes still cross near the ends; such postures are refused one by one. See the mesh pairs under Checks.
- **Meshes.** The three parts from `parts/`. The stock SO-101 links and the servos from the SO-101 solids, placed by a fit to the first design's display meshes within 0.11 mm. They are written to `parts/stock/` by `build_parts.py --stock` and are not committed.
- **Masses.** Each printed part as printed (`params.printed_grams`, PLA at 1.24 g/cm³), with its centre at the centre of its solid; 55 g per servo; and the SO-101 link inertials. The robot comes to 2.112 kg. Screws, horns, wiring and any payload are not counted.
- **Trunk.** The meshes of the six-part model with the original roof and without the neck servo. Its mass and centre of mass are the Duck's with that servo's 55 g taken out.

## Checks

The "Hollow parts inspection" job runs these on each pull request.

```sh
python -m pip install -r crosswise/requirements.txt
python scripts/restore_assets.py
python collision/build_bvh.py
python hollow/cad/build_parts.py --stock
python hollow/cad/build_parts.py --check
OPENBLAS_NUM_THREADS=1 python -m pytest hollow/tests -q
python hollow/inspect_viewer.py --screen --legs --replay
OPENBLAS_NUM_THREADS=1 python hollow/postures.py
```

[`tests/`](tests/) checks the parts in CAD.

- **Each part.** One valid closed solid, walls of at least 1.6 mm over 99 % of the surface, and a fit on a 180 mm bed.
- **Servos.** Each servo sits in its seat no deeper than the stock SO-101 holder sits on its own servo, 0.45 mm in the servo mesh. It can be slid in or lifted out along a straight line. The printed part leaves both of its plugs open. All four screw passages of each seat are open, with a wall under each screw head, and the seat stops the servo in every direction but out along its long axis.
- **Horns.** Each round end touches the face of its horn. A 2.9 mm bore is open at each of its four horn screws, and a 4 mm driver reaches the screws of the wrist link.
- **Neck.** The four recorded neck screws are holes in the trunk mesh, and the plug has a pilot hole at each. The mount touches no trunk mesh. The two shelf screws meet holes in the servo mesh.
- **Moving clearance.** Each part keeps 1.5 mm to the parts that move against it: the mount over ±108° of yaw, the forearm link over ±95° of the elbow, the wrist link over ±155° of roll. Only the 13 mm around each horn is left out.
- **Build.** The committed STEP and STL files are what `parts.py` draws, surface against surface within 0.05 mm. The model's masses are the parts as printed, the print rule reproduces the stock SO-101 masses, and the zero pose has no intersection.

[`postures.py`](postures.py) replays the CSV postures on the model with the checks of the [six-part replay](../coverage/README.md). It fails unless exactly targets 64, 74, 78, 89 and 93 fail and exactly targets 5, 18, 21, 50, 53 and 60 are over the load limit.

- **Targets.** Joint ranges with a 2° reserve, tool within 1.5 mm and 1°, soles within 0.2 mm and 0.15°, support margin of at least 5 mm, nothing below the floor, a gap of 0.3 mm from each hip link and 1 mm from each thigh, shin and foot to the trunk, no mesh intersection, and all of that again at ten jaw openings up to 20°. The 91 passing endpoints are within 0.558 mm and 0.309° of their targets. The smallest support margin is 5.37 mm at target 90. The smallest gaps are 0.30 mm at a hip link and 1.01 mm at a thigh.
- **Clips.** The pickup, overhead, fold and parked postures with the same checks at fixed legs.
- **Viewer.** The transforms stored in the viewer equal the model's in every frame of every clip.
- **Mesh pairs.** Every pair of meshes on the two sides of a joint is tested, and touching counts as a hit, with these exceptions. A servo and the link on its horn touch at the horn: those five pairs are tested without the 13 mm round the joint axis. The stock gripper on its stock wrist link touches all round its joint and gets a test that ignores touching faces. Two pairs inside the stock gripper are not tested: its servo against its body, and its body against the moving jaw.

`python hollow/postures.py --search` searches every target whose row fails or is over the limit. `--search-pickup` searches the pickup crouch, `--make-clips` the overhead and parked postures and the fold. [`viewer_build.py`](viewer_build.py) puts the parts and all clips into the viewer; it needs shapely.

[`inspect_viewer.py`](inspect_viewer.py) works from the viewer alone. Without options it needs only NumPy.

- **Files.** Frame counts, triangle indices, and each part mesh holding the same triangles as its print file on the viewer's 0.1 mm grid. It fails if either jaw overlaps the cube in the pickup clip.
- **Mesh screen.** `--screen` intersects the display meshes of every pair of separately moving bodies in 405 stored frames. The five targets without a passing posture are left out. It fails if a printed part meets a body other than the servo it sits on, or if any other pair intersects in some frames only.
- **Legs.** `--legs` fails if a leg link intersects the trunk in any endpoint, pickup frame or folding frame.
- **Targets and margin.** `--replay` compares the stored tool poses and soles with the targets and anchors and computes the support margin with the masses of `robot.xml`.

## Limits

- **Not printed.** No part has been printed, fitted on a servo or loaded. Hole sizes, the spring of the round ends over the elbow servo and the screw lengths in BUILD.md come from the CAD alone.
- **Print settings.** The masses, and with them the load and the count, assume the parts are printed like the stock SO-101 parts: two walls, 15 % infill. Printed heavier they load the shoulder more. With three walls and 20 % infill, the stored postures keep 83 targets within the limit instead of 85.
- **Static loads only.** The limit compares gravity torque in held postures. Motion, friction, grip force and heat are left out, and so are the other five arm servos. The largest gravity torque elsewhere is 0.653 N m at the elbow with 100 g.
- **Stock meshes that cross.** Three pairs of stock meshes cross inside the joint ranges: the shoulder yaw servo and the shoulder stage beyond −105° and +104.5° of yaw, the shoulder pitch servo and the upper arm beyond 98.75° of pitch, and the gripper and the wrist link between −117.5° and −115° of wrist roll. The crossings are under 1 mm deep at the first two. No stored posture lies in any of them. The stock arm has the same parts in the same places, so they may be artefacts of the meshes. That is not known.
- **Yaw servo plugs.** The shoulder stage turns under the yaw servo, across the space its two plugs need, as in the stock arm's base. How the cables leave there is not worked out.
- **Pickup approach.** From frame 16 of the pickup clip the fixed jaw lowers along one cube face with less than 0.05 mm of clearance. A physical approach needs clearance on that side.
- **Joint angles.** 48 of the 96 endpoints hold at least one arm joint at the largest or smallest angle stored for it, 40 of them the elbow at ±93°.
- **Cables.** Each seat leaves both plugs of its servo open. Cable lengths and routes are not designed.
- **Not checked anywhere:** motion between the endpoints, the walking policy with the arm on the robot, and hardware of any kind.
