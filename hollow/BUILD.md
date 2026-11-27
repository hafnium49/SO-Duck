# Building the hollow arm

**The three parts are not yet printed.** Every fit in this guide comes from the CAD and the checks in [`tests/`](tests/). Nothing here has been confirmed on a printer or on the robot. Print one part and try it on a servo before printing the rest.

## Parts and fasteners

### Printed parts

| File in [`parts/`](parts/) | Mass, printed solid | Size |
| --- | ---: | --- |
| `neck_socket_mount` | 50.7 g | 68 × 48 × 127 mm |
| `elbow_to_forearm_roll` | 40.7 g | 117 × 77 × 60 mm |
| `forearm_roll_to_wrist` | 21.8 g | 36 × 46 × 57 mm |

Each part has a STEP and an STL file in millimetres. All three fit a 180 mm bed.

The parking pad on the roof is the one of the earlier designs, [`crosswise/park_pad.stl`](../crosswise/park_pad.stl). It is needed only to park the gripper.

### Stock SO-101 parts

Print these from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), follower arm, or take them from a follower kit.

| SO-101 file | Where it goes |
| --- | --- |
| `Rotation_Pitch_SO101` | On the shoulder yaw servo. Carries the shoulder pitch servo. |
| `Motor_holder_SO101_Base` | Closes `Rotation_Pitch_SO101` around the shoulder pitch servo. |
| `Upper_arm_SO101` | Shoulder pitch to elbow. Carries the elbow servo. |
| `Wrist_Roll_Pitch_SO101` | On the wrist flex servo. Carries the wrist roll servo. |
| `Wrist_Roll_Follower_SO101` | The gripper body with the fixed jaw. Carries the gripper servo. |
| `Moving_Jaw_SO101` | The moving jaw. |

Not used: `Base_SO101`, `Base_motor_holder_SO101`, `Under_arm_SO101`, `Motor_holder_SO101_Wrist` and the mounting plate. The three printed parts take their place.

### Servos

Seven Feetech STS3215, each with its horn. A follower kit holds six, so one more is needed. The Duck's neck servo leaves the robot: the mount fills its socket.

| Joint | The servo sits in | Its horn drives |
| --- | --- | --- |
| Shoulder yaw | `neck_socket_mount` | `Rotation_Pitch_SO101` |
| Shoulder pitch | `Rotation_Pitch_SO101` | `Upper_arm_SO101` |
| Elbow | `Upper_arm_SO101` | `elbow_to_forearm_roll` |
| Forearm roll | `elbow_to_forearm_roll` | `forearm_roll_to_wrist` |
| Wrist flex | `forearm_roll_to_wrist` | `Wrist_Roll_Pitch_SO101` |
| Wrist roll | `Wrist_Roll_Pitch_SO101` | `Wrist_Roll_Follower_SO101` |
| Gripper | `Wrist_Roll_Follower_SO101` | `Moving_Jaw_SO101` |

### Screws for the three printed parts

Sizes follow the holes. Lengths are worked out from the wall each screw passes through and have not been tried.

| Screws | Where | Traced to |
| --- | --- | --- |
| 4 × M2 × 10, thread-forming | Through the two side walls of the Duck's neck socket into the tabs of the mount's plug. Two per wall. | The four 2.4 mm holes of `trunk_top`, which held the neck servo. Wall 6.5 mm, tab 4.9 mm with 1.8 mm pilot holes. The plug has both rows of holes on both sides; use the two that line up with each wall. |
| 2 × M2 × 6 | From below through the shelf of the mount into the back of the shoulder yaw servo. | The servo's two threaded holes near its tail, 20.3 mm behind its centre and 10.3 mm either side. Shelf 2.9 mm. |
| 2 × M2 × 6 | Through the seat of `elbow_to_forearm_roll` into the forearm roll servo, one on each wide face. | The stock passages of the SO-101 under arm: 2 mm of wall, then a 4 mm recess for the head. |
| 2 × M2 × 6 | Through the seat of `forearm_roll_to_wrist` into the wrist flex servo, one on each wide face. | The same stock passages, mirrored. |
| 8 × M3 × 6 | `elbow_to_forearm_roll` onto the elbow servo: four into the horn, four into the idler on the opposite face. | The 3.0 mm holes with 5.4 mm recesses in the two round ends of the SO-101 under arm, centred on the joint axis. |
| 4 × M3 × 6 | `forearm_roll_to_wrist` onto the horn of the forearm roll servo. | The same holes of the same round end. Four channels through the seat above let a 4 mm driver reach them. |

Every horn also takes its centre screw, as supplied with the servo. Every other joint is a stock SO-101 joint. Fasten those as the SO-101 assembly guide says.

Each of the two seats holds its servo on four sides and with two screws. The stock arm adds a clip-on holder with two more screws. That holder was left off to bring the shoulder load within the limit. See the limits in [README.md](README.md).

## Printing

Material: PLA or PETG. The model's masses assume PLA at 1.24 g/cm³ printed solid. Use a 0.4 mm nozzle, 0.2 mm layers, four perimeters and 100 % infill. The walls are 1.6 mm or more, so little is left to infill.

The orientations below put the layers along the direction in which each part bends. The overhang areas come from `python hollow/cad/build_parts.py --orientations`, which lists all six ways to lay each part down. "Up" names the axis of the part's file that points away from the bed.

| Part | Orientation | Supports |
| --- | --- | --- |
| `neck_socket_mount` | +y up, on its side, 48 mm tall | 53.7 cm² of overhang: under the upper side wall of the upright, inside the servo holder and inside the plug. Add a brim: 4.9 cm² touch the bed. |
| `elbow_to_forearm_roll` | +y up, the back of the servo seat on the bed, 77 mm tall | 9.3 cm² of overhang, inside the seat and under the round ends. |
| `forearm_roll_to_wrist` | +x up, the closed face of the channel on the bed, 36 mm tall | 14.5 cm² of overhang, under the seat and the round end. Add a brim: 2.2 cm² touch the bed. |

The mount could also stand upright with 16.5 cm² of overhang, but then the layers lie across the upright, which carries the whole arm. Keep supports out of the screw holes, or clear them with a 2 mm or 3 mm drill.

## Assembly

Set the ID of every servo and move it to its middle position, 2048, before it goes into a part. Use seven IDs that the leg servos do not use. This guide numbers them from the trunk outward:

| ID | Joint |
| ---: | --- |
| 31 | Shoulder yaw |
| 32 | Shoulder pitch |
| 33 | Elbow |
| 34 | Forearm roll |
| 35 | Wrist flex |
| 36 | Wrist roll |
| 37 | Gripper |

The bus runs in the same order, from the trunk to the gripper. Each of the three printed parts leaves both plugs of its servo open. Cable lengths and routes are not worked out.

1. **Trunk.** Take out the Duck's neck servo with its two horns. Keep its four screws only if they are thread-forming M2 × 10.
2. **Mount and shoulder yaw servo.** With the mount on the bench, lower the servo into the holder from above, horn up and tail toward the upright. Fasten it from below with two M2 × 6. The driver reaches the shelf only while the mount is off the robot.
3. **Mount onto the trunk.** Push the plug into the neck socket and fasten it through the socket walls with four M2 × 10. The body shell may have to come off to reach them, as when the neck servo was fitted.
4. **Shoulder and upper arm.** Build `Rotation_Pitch_SO101`, the shoulder pitch servo, `Motor_holder_SO101_Base`, `Upper_arm_SO101` and the elbow servo as in the SO-101 guide, and fit `Rotation_Pitch_SO101` on the horn of the shoulder yaw servo.
5. **Forearm roll servo.** Slide it into the seat of `elbow_to_forearm_roll` tail first, horn away from the elbow. Fasten it with two M2 × 6, one from the elbow side and one from the horn side.
6. **Elbow.** Spring the two round ends of `elbow_to_forearm_roll` over the horn and the idler of the elbow servo. Fasten four M3 × 6 on each side.
7. **Wrist link.** Before the wrist flex servo goes in, put the round end of `forearm_roll_to_wrist` on the horn of the forearm roll servo and fasten four M3 × 6 through the channels in the seat.
8. **Wrist flex servo.** Slide it into the seat tail first and fasten it with two M2 × 6.
9. **Wrist and gripper.** Build `Wrist_Roll_Pitch_SO101`, the wrist roll servo, `Wrist_Roll_Follower_SO101`, the gripper servo and `Moving_Jaw_SO101` as in the SO-101 guide, and fit `Wrist_Roll_Pitch_SO101` on the horn and idler of the wrist flex servo.
10. **Parking pad.** Glue it on the roof 104 mm behind the trunk's origin and 23 mm to the left.

### Indexing the horns

The output shaft has 25 teeth, so a horn can sit 14.4° off. With all seven servos at 2048 the arm must stand in its zero pose:

- the upper arm and the forearm stand upright above the mount, the elbow 344 mm above the trunk's origin and the forearm roll axis vertical,
- the wrist and the gripper lie level and point to the rear, the tool point 166 mm behind the trunk's origin and 502 mm above it, on the centre line,
- the jaws are closed.

Fit each part as close to that pose as the teeth allow, then check it. The table gives how far the named point is from its place when that joint alone is one tooth off. It comes from `python hollow/horn_check.py`.

| Joint | Check with all servos at 2048 | One tooth off moves it |
| --- | --- | ---: |
| Shoulder yaw | The tool point is on the centre line of the trunk. | 42 mm |
| Shoulder pitch | The tool point is 502 mm above the trunk's origin. | 84 mm |
| Elbow | The forearm roll axis is vertical; the tool point is 166 mm behind the origin. | 69 mm |
| Forearm roll | The gripper points straight to the rear, the tool point on the centre line. | 50 mm |
| Wrist flex | The gripper is level. | 40 mm |
| Wrist roll | Open the jaw by 90°: its tip must be straight to one side of the gripper, level with it. | 26 mm |
| Gripper | The jaws are closed, tip on tip. | 21 mm |

Work from the trunk outward. An error at an earlier joint moves every later check.

Before powering the arm on the robot, set the joint limits in the controller to the ranges of [`cad/params.py`](cad/params.py), `RANGES_DEG`, and move each joint slowly through its range by hand.
