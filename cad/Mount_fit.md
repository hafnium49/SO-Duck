# Case-mounted forearm for the ostrich

This single custom link keeps the Open Duck Mini v2 neck pitch, head pitch, head yaw and terminal roll mechanism. It replaces the Duck moving `head_roll_mount` and the SO-101 `Under_arm_SO101`. It bolts to four bosses on the moving case of the terminal servo and carries the original rear roll bearing. The Duck head shell is omitted. The stock SO-101 wrist motor holder, wrist flex and roll mechanism, and gripper attach to the front of the link. The STEP and STL use millimeters.

## Fit and assembly

1. Retain the Duck `head_yaw_to_roll` bracket and its driven-horn attachment, the two original neck servos and the stock opposite roll bearing. Keep the original shaft, yaw servo and roll servo in their Duck locations. The terminal servo case rotates relative to its anchored output horn.
2. Work on the loose yaw-to-roll module, with the rear bearing removed. In the print frame, slide the empty forearm forward over the stock spigot to 2.2 mm beyond its installed X position. Lift the original terminal servo into the case saddle from below at this offset. Slide the servo and forearm together 2.2 mm rearward until the horn seats against the stock bracket. Fit the original horn fasteners. The local relief in the rear bearing shoulder provides the temporary axial travel.
3. Secure the servo using its four asymmetric case bosses. The clearances are 2.5 mm with 4.5 mm counterbores and about 2.4 mm of material below the heads. Check head size, thread engagement and allowable screw depth on the donor servo. Two forward holes provide straight driver access to the idler-side screws. Slide the original 20 × 32 × 7 mm rear roll bearing in from the outboard side and onto its spigot. The prototype has a 32.2 mm pocket, an inboard shoulder and two 2.5 mm pilot holes to tap M3. Two additional M3 × 6 screws and 7 mm OD washers retain the outer race. The model leaves about 0.5 mm axial clearance at the outboard face. Check fit and end float on the hardware. Give the cable room for full roll and yaw motion.
4. Mount the original SO-101 wrist motor holder and the stock distal assembly using their donor fasteners. The wrist interface is taken directly from the SO-101 native CAD. Mounting hardware, washers and harnesses are omitted from the assembly GLB.

## Axes and sign

Use Duck print axes with +X forward, +Y left and +Z up. At neutral, the new forearm points forward, terminal roll uses +X, wrist flex uses +Y, and wrist roll uses −X in the study. Positive wrist flex lowers the forward-pointing gripper. At this reference posture the two roll axes are parallel and spatially offset. Bending the wrist changes the angle between them. At neutral the wrist-flex shaft goes through (232.05, −18.20, 134.26) mm and the terminal roll line is y = 0, z = 129.06 mm. The study’s terminal angle has the opposite sign to the original Duck MJCF `head_roll` coordinate. Confirm servo direction at low travel using measured motion before mapping controller commands.

## CAD checks

| Check | Result |
|---|---|
| STEP re-import | One valid solid |
| Custom link against native terminal servo, flex servo, stock wrist holder and original bearing envelope | Zero overlapping volume at their mounted positions |
| Link to yaw servo, terminal roll −30°, 0°, +30° | Minimum 9.415 mm at sampled endpoints |
| Link to native `head_yaw_to_roll` bracket | No mesh contact at 61 terminal angles from −30° to +30° in 1° steps |
| Link to `head_pitch_to_yaw` bracket | No mesh contact at 19,581 yaw/roll combinations in 1° steps across yaw ±160° and roll ±30° |
| Assembly motion with bearing removed | No surface interference along 0.1 mm axial steps to +2.2 mm, 0.1 mm vertical steps bringing the servo up into the saddle, 0.01 mm return steps before the horn reaches its intended mating surface, or 0.1 mm steps inserting the bearing from the rear |
| Four straight driver paths | 4.3 mm diameter cylinders extending at least 55 mm from the four case screw seats had zero solid overlap with link, yaw case and terminal case |
| Distal wrist/gripper against new forearm and terminal case | No mesh contact on 39 flex angles, −95° to +95° at 5° spacing, and 34 roll angles spanning the donor range, approximately −157.2° to +162.8°. The moving jaw was also checked at −10°, 0°, 30°, 60°, 90°, 100° |
| Distal wrist/gripper against yaw case and stock yaw carrier | Same flex, wrist roll and jaw grid at terminal −30°, 0° and +30° |

The rigid CAD volume is 106.89 cm³. The reference SO forearm is about 93 cm³. Weigh the printed link for the balance model. A fully solid part at an assumed density of 1.24 g/cm³ would weigh approximately 133 g. Infill, walls and material change that value. The uniform-solid centroid is (124.00, −0.10, 140.60) mm in the Duck print frame. The part spans 244.45 × 52.08 × 66.91 mm in the assembly axes. Bed size and orientation matter for a single-piece print.

These rigid-body checks cover the listed pairs and poses. Loads, bearing capacity, fastener pull-out, beam stiffness, physical fit, thermal limits, powered motion and continuous cable clearance still require hardware testing. The accompanying whole-robot study reports its own pose and support assumptions.

## Source models

The wrist-end geometry derives from TheRobotStudio’s SO-101 forearm. The Duck terminal case and rear bearing datums follow the v2 assembly. The design modification integrates the moving case saddle, rear bearing housing and forward wrist mounting features into one printed body. [SO-101 source CAD](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STEP/SO101) · [Open Duck Mini v2 source](https://github.com/apirrone/Open_Duck_Mini/tree/v2) · [Open Duck Playground](https://github.com/apirrone/Open_Duck_Playground).
