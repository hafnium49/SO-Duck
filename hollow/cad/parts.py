"""The three custom parts of the hollow arm, built from SO-101 solids. Millimetres.

Each part is given in the frame of the model body that carries it. Where a part holds a
servo it reuses the SO-101 feature that holds the same servo in the stock arm, placed
by the same servo-to-holder relation, so the fit is the stock one.
"""

import xml.etree.ElementTree as ET
from functools import lru_cache

import cadquery as cq
import numpy as np
from scipy.spatial.transform import Rotation

import params
import stock

# The STS3215 horn's four tapped holes about its axis.
HORN_HOLES_MM = [(-4.95, -4.95), (-4.95, 4.95), (4.95, -4.95), (4.95, 4.95)]

# --- elbow-to-forearm-roll link, in the frame of forearm_pitch_linkage -----------------
# The frame's origin is on the elbow axis, z along it. The elbow servo's horn is at z = 0
# and its idler near z = 36.4. The forearm runs along +x.
CHEEK_RADIUS_MM = 12.4  # the SO-101 cheek is a 12 mm disc
# The screw passages run through the whole thickness of each cheek, so that a screwdriver
# reaches the horn from outside: centre and length along z, for the horn and the idler side.
ELBOW_BORES_MM = ((-6.0, 11.8), (42.2, 11.4))
CHEEK_Z_MM = ((-12.5, 2.0), (34.5, 48.0))  # the two cheeks; above 48 the donor has only ribs
# In the SO-101 under arm the horn's hole pattern sits 0.46 mm off the joint axis.
DONOR_HORN_CENTRE_MM = (0.0, -0.46)
# The upper arm passes between the plates when the elbow folds. Outside this band of z
# it is absent near the elbow, so the plates run below and above it.
PLATE_LOW_MM = (-12.0, -8.8)
PLATE_HIGH_MM = (45.0, 48.0)
# Over the elbow's swing the upper arm leaves free, at plate height, a corridor of this
# half-width along the forearm, and everything beyond this x at any height.
CORRIDOR_HALF_MM = 9.0
CORRIDOR_END_MM = 50.0
# The donor's own web leaves each cheek along the arm. This much of it is kept, at plate
# height, as the root of each plate: it carries the screw passages and the donor's fillets.
ROOT_END_MM = 18.0
ROOT_HALF_MM = 10.5
PLATE_START_MM = 14.0  # the plates begin here, inside the root and beyond the screw passages
PLATE_BORDER_MM = 7.0  # width of the frame left around each plate's window
STRIP_MM = 4.0  # thickness of the strips joining the plates to the pocket
HORN_PLANE_GAP_MM = 1.5  # the link ends this far before the roll joint's plane


def _frame(position, wxyz):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(np.roll(np.asarray(wxyz, dtype=float), -1)).as_matrix()
    out[:3, 3] = position
    return out


def _box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def _invalid(shape):
    """Bounding boxes of the faces that make a shape invalid, for an error message."""
    from OCP.BRepCheck import BRepCheck_Analyzer

    out = []
    for face in shape.Faces():
        if not BRepCheck_Analyzer(face.wrapped).IsValid():
            box = face.BoundingBox()
            out.append([round(v, 1) for v in (box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax)])
    return out


def _join(name, first, *more):
    """Fuse pieces one at a time into one valid solid, or say where it went wrong."""
    shape = first
    for index, piece in enumerate(more, start=1):
        shape = shape.fuse(piece).clean()
        if not shape.isValid():
            raise ValueError(f"{name}: invalid after piece {index}, at faces {_invalid(shape)[:4]}")
    solids = shape.Solids()
    if len(solids) != 1 or not solids[0].isValid():
        raise ValueError(f"{name}: {len(solids)} solids")
    return solids[0]


@lru_cache(maxsize=None)
def so101(body, mesh):
    """Where the stock SO-101 puts a part in one of its bodies: 4x4, millimetres."""
    root = ET.parse(stock.ROOT / "collision" / "inputs" / "so101.xml")
    geom = root.find(f".//body[@name='{body}']/geom[@mesh='{mesh}'][@class='visual']")
    frame = _frame(1000 * np.fromstring(geom.get("pos"), sep=" "), np.fromstring(geom.get("quat"), sep=" "))
    frame.setflags(write=False)
    return frame


def roll_servo_frame():
    """The forearm-roll servo in the forearm frame, moved back by the chosen shortening."""
    frame = stock.placement("forearm_roll_case").copy()
    frame[0, 3] -= 1000 * params.FOREARM_SHORTEN
    return frame


def _wrist_pocket(servo_frame):
    """The pocket that holds the SO-101 wrist servo, placed around a servo at servo_frame.

    It is the end of the under arm, which seats the servo's tail, and the wrist motor
    holder that clips onto it and closes it into a band. The under arm is kept as far
    toward the elbow as the holder reaches, so the two meet as they do in the stock arm.
    """
    under, holder, servo = (so101("lower_arm", m) for m in ("under_arm_so101_v1", "motor_holder_so101_wrist_v1", "sts3215_03a_v1"))
    onto = servo_frame @ np.linalg.inv(servo)
    clip = stock.moved(params.donor("Motor_holder_SO101_Wrist"), holder)  # in the lower arm's frame
    reach = clip.BoundingBox().xmax  # the elbow is toward +x
    seat = stock.moved(params.donor("Under_arm_SO101"), under).intersect(_box(-200, reach, -100, 100, -100, 100))
    return stock.moved(_join("servo pocket", seat, clip), onto)


def _horn_cheek():
    """The horn-side round end of the SO-101 under arm, hole pattern on the z axis.

    As in the stock arm: the horn's face at z = 0, the cheek below it, a lip around the horn above.
    """
    under = so101("lower_arm", "under_arm_so101_v1")
    x, y = DONOR_HORN_CENTRE_MM
    centred = stock.moved(params.donor("Under_arm_SO101"), _frame([-x, -y, 0], [1, 0, 0, 0]) @ under)
    z0, z1 = CHEEK_Z_MM[0]
    return _join("horn cheek", centred.intersect(cq.Solid.makeCylinder(CHEEK_RADIUS_MM, z1 - z0, cq.Vector(0, 0, z0))))


def _elbow_roots():
    """The donor's material at the elbow: its two round ends, each with the start of its web.

    Returned for the horn side and the idler side, hole patterns on the elbow axis.
    """
    under = so101("lower_arm", "under_arm_so101_v1")
    x, y = DONOR_HORN_CENTRE_MM
    # Half a turn puts the side the SO-101 arm leaves from toward +x, and the shift puts
    # the hole pattern on the axis.
    turned = stock.moved(params.donor("Under_arm_SO101"), _frame([0, 0, 0], [0, 0, 0, 1]) @ _frame([-x, -y, 0], [1, 0, 0, 0]) @ under)
    roots = []
    for (z0, z1), plate in zip(CHEEK_Z_MM, (PLATE_LOW_MM, PLATE_HIGH_MM)):
        disc = cq.Solid.makeCylinder(CHEEK_RADIUS_MM, z1 - z0, cq.Vector(0, 0, z0))
        # at plate height only: between the plates the upper arm passes
        web = _box(0, ROOT_END_MM, -ROOT_HALF_MM, ROOT_HALF_MM, max(plate[0], z0), min(plate[1], z1))
        roots.append(_join("elbow root", turned.intersect(disc.fuse(web))))
    return roots


def _plate(z, x0, x1, y_back):
    """A plate from its root along the corridor, then fanning out to the pocket's back."""
    z0, z1 = z
    fan = [
        (CORRIDOR_END_MM, CORRIDOR_HALF_MM),
        (x1, y_back + 12.0),
        (x1, y_back),
        (x0, y_back),
        (CORRIDOR_END_MM, -CORRIDOR_HALF_MM),
    ]
    outline = [(PLATE_START_MM, CORRIDOR_HALF_MM)] + fan + [(PLATE_START_MM, -CORRIDOR_HALF_MM)]
    plate = cq.Workplane("XY").workplane(offset=z0).polyline(outline).close().extrude(z1 - z0).val()
    window = cq.Workplane("XY").workplane(offset=z0 - 1).polyline(fan).close().offset2D(-PLATE_BORDER_MM)
    return plate.cut(window.extrude(z1 - z0 + 2).val())


def forearm_link():
    """Elbow-to-forearm-roll link: the SO-101 ends on the elbow horns, two plates, a pocket for the roll servo."""
    pocket = _wrist_pocket(roll_servo_frame())
    box = pocket.BoundingBox()
    x0 = box.xmin
    x1 = min(box.xmax, 1000 * (params.FRAME["forearm_roll"][0][0] - params.FOREARM_SHORTEN) - HORN_PLANE_GAP_MM)
    y_back = box.ymin
    root_low, root_high = _elbow_roots()
    # Strips along the pocket's back join each plate to the holder between them.
    strip_low = _box(x0, x1, y_back, y_back + STRIP_MM, PLATE_LOW_MM[0], box.zmin + 0.2)
    strip_high = _box(x0, x1, y_back, y_back + STRIP_MM, box.zmax - 0.2, PLATE_HIGH_MM[1])
    low = _join("lower arm", root_low, _plate(PLATE_LOW_MM, x0, x1, y_back), strip_low)
    high = _join("upper arm", root_high, _plate(PLATE_HIGH_MM, x0, x1, y_back), strip_high)
    return _join("forearm link", pocket, low, high)


# --- forearm-roll-to-wrist link, in the frame of forearm_roll_linkage ------------------
# The origin is on the roll axis, z along it, away from the roll servo. The servo's horn
# ends at z = 1.5. The wrist-flex servo lies across the axis with its tail over it.
ROLL_HORN_FACE_MM = 1.5
ROLL_SEAT_BORE_MM = (7.5, 11.8)  # centre and length along z of the screw passages in the seat
# The forearm link's pocket reaches 8.2 mm past the roll joint's plane, 16.3 mm from the
# axis. The seat turns inside it; the bridge starts above it.
BRIDGE_Z_MM = (10.0, 36.5)
BRIDGE_X_MM = (-12.5, 4.2)
BRIDGE_HALF_MM = 11.0
BRIDGE_WALL_MM = 3.0
# The four seat screws are driven from beyond the pocket, before the wrist servo goes in.
DRIVER_RADIUS_MM = 2.25
ROLL_DRIVER_PATH_MM = (43.0, 58.0)  # centre and length along z of the driver's path above the seat


def wrist_servo_frame():
    """The wrist-flex servo in the roll body's frame, moved by the chosen shortening."""
    frame = stock.placement("wrist_flex_case").copy()
    frame[0, 3] += 1000 * params.WRIST_SIDE_SHORTEN
    frame[2, 3] -= 1000 * params.WRIST_AXIAL_SHORTEN
    return frame


def wrist_link():
    """Forearm-roll-to-wrist link: an SO-101 cheek on the roll horn, a channel, the SO-101 pocket for the wrist servo."""
    # The cheek turned over: its horn face on the roll horn, the cheek beyond it.
    seat = stock.moved(_horn_cheek(), _frame([0, 0, ROLL_HORN_FACE_MM], [0, 1, 0, 0]))
    pocket = _wrist_pocket(wrist_servo_frame())
    (x0, x1), (z0, z1), half, wall = BRIDGE_X_MM, BRIDGE_Z_MM, BRIDGE_HALF_MM, BRIDGE_WALL_MM
    # A channel closed on three sides; the screws' paths run inside it and past its open side.
    channel = _box(x0, x1, -half, half, z0, z1).cut(_box(x0 + wall, x1 + 1, -half + wall, half - wall, z0 - 1, z1 + 1))
    link = _join("wrist link", pocket, channel, seat)
    top = ROLL_DRIVER_PATH_MM[0] + ROLL_DRIVER_PATH_MM[1] / 2 + 10
    for x, y in HORN_HOLES_MM:
        path = cq.Solid.makeCylinder(DRIVER_RADIUS_MM, top - BRIDGE_Z_MM[0] - 4, cq.Vector(x, y, BRIDGE_Z_MM[0] + 4))
        link = _join("wrist link", link.cut(path))
    return link
