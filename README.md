# SO-Duck

SO-Duck uses a **crosswise forearm connector** on an Open Duck Mini V2 biped. The last Duck neck motor now pitches the stock SO-101 forearm forward and back. The robot has 17 actuators: ten leg joints, four neck/forearm joints, two wrist joints and the gripper. The original head shell is outside the arm load path.

![Folded SO-Duck, floor grasp and high reach](crosswise/poses.png)

| Open the current design | Contents |
|---|---|
| [Connector assembly (STEP)](crosswise/connector_assembly.step) | Yaw servo, printed carrier, crosswise terminal servo and stock forearm in their mounting positions |
| [Carrier STEP](crosswise/carrier.step) · [carrier STL](crosswise/carrier.stl) · [rear pad STL](crosswise/park_pad.stl) | Fabrication geometry in mm |
| [Whole-robot pose explorer](SO_Duck_pose_explorer.html) | Download and open locally to orbit the assembled robot through the fold, floor pickup and upper reach |
| [MuJoCo model](crosswise/robot.xml) · [assembly and reproduction guide](crosswise/README.md) | Complete joint chain, actuator bindings, parking-pad placement, six keyframes and replay checks |

With both feet planted, the folded model is **449 mm high**, down from **659 mm** upright. The fixed finger seats on a small pad bonded to the rear roof. At the floor, the jaws can enclose a 20 mm cube 140 mm ahead of the toes and follow a 30 mm lift. At a separate overhead pose, the gripper reference point reaches **651 mm above the floor** at 200 mm forward of the torso origin. These results come from CAD and kinematic replay. Printed fit, harness clearance, motor duty, grip friction and walking with the arm parked remain unverified.

## Earlier whole-body study

The original longitudinal-roll design, orientation search, changing-stance experiments and published data remain available through the [study index](docs/STUDY_INDEX.md) and [original pose explorer](study/legacy_pose_explorer.html). The [new target results](crosswise/nominal_targets.csv) use the crosswise model and its revised joint limits. [Upstream geometry and licenses](docs/PROVENANCE.md) cover Open Duck Mini V2, Duck Playground and TheRobotStudio SO-101.
