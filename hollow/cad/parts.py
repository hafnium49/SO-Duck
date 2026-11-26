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
STRIP_MM = 3.0  # thickness of the strips joining the plates to the seat
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


# The servo's body ends this far behind its centre, and its two wide faces are this far
# either side of it. The seat's own screw passages meet two of the servo's tapped M2 holes:
# (x, y) in the servo's frame, and the face they are on (-1 back, +1 horn side).
SERVO_TAIL_MM = 22.7
SERVO_FACE_MM = 15.9
SEAT_BACK_MM = 4.0  # of the under arm kept behind the tail; less leaves walls under 1 mm
SEAT_SCREWS_MM = (((-20.30, -10.27), -1), ((-16.58, -10.25), 1))


def _servo_seat(servo_frame, mirrored=False):
    """The end of the SO-101 under arm that seats the wrist servo, placed around a servo at servo_frame.

    It holds the servo on four sides and takes two of its screws. The stock arm closes
    the open side with a separate clip-on holder and two more screws; that holder is
    left off, which halves the weight at the end of the link. mirrored turns the open
    side to the servo's other flank; the servo is the same on both.
    """
    under, servo = so101("lower_arm", "under_arm_so101_v1"), so101("lower_arm", "sts3215_03a_v1")
    whole = stock.moved(params.donor("Under_arm_SO101"), np.linalg.inv(servo) @ under)  # in the servo's frame
    seat = _join("servo seat", whole.intersect(_box(-SERVO_TAIL_MM - SEAT_BACK_MM, 200, -100, 100, -100, 100)))
    if mirrored:
        seat = seat.mirror("XZ")
    return stock.moved(seat, servo_frame)


def seat_screws(servo_frame, mirrored=False):
    """Centre and outward direction of the seat's two screw passages, where servo_frame puts them."""
    out = []
    for (x, y), side in SEAT_SCREWS_MM:
        centre = np.array([x, -y if mirrored else y, side * (SERVO_FACE_MM + 3.0), 1.0])
        out.append((tuple((servo_frame @ centre)[:3]), tuple(servo_frame[:3, 2] * side)))
    return out


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
    """Elbow-to-forearm-roll link: the SO-101 ends on the elbow horns, two plates, the SO-101 seat for the roll servo."""
    pocket = _servo_seat(roll_servo_frame())
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
    """Forearm-roll-to-wrist link: an SO-101 cheek on the roll horn, a channel, the SO-101 seat for the wrist servo."""
    # The cheek turned over: its horn face on the roll horn, the cheek beyond it.
    seat = stock.moved(_horn_cheek(), _frame([0, 0, ROLL_HORN_FACE_MM], [0, 1, 0, 0]))
    # The seat's closed flank is toward the roll servo, on top of the channel.
    pocket = _servo_seat(wrist_servo_frame(), mirrored=True)
    (x0, x1), (z0, z1), half, wall = BRIDGE_X_MM, BRIDGE_Z_MM, BRIDGE_HALF_MM, BRIDGE_WALL_MM
    # A channel closed on three sides; the screws' paths run inside it and past its open side.
    channel = _box(x0, x1, -half, half, z0, z1).cut(_box(x0 + wall, x1 + 1, -half + wall, half - wall, z0 - 1, z1 + 1))
    link = _join("wrist link", pocket, channel, seat)
    top = ROLL_DRIVER_PATH_MM[0] + ROLL_DRIVER_PATH_MM[1] / 2 + 10
    for x, y in HORN_HOLES_MM:
        path = cq.Solid.makeCylinder(DRIVER_RADIUS_MM, top - BRIDGE_Z_MM[0] - 4, cq.Vector(x, y, BRIDGE_Z_MM[0] + 4))
        link = _join("wrist link", link.cut(path))
    return link


# --- neck socket mount, in the trunk frame ---------------------------------------------
# The socket in the trunk top that held the Duck's neck servo: floor at z = 54.9, side
# walls 16.1 mm either side of the middle, up to z = 72.5, a back wall at x = -11.4.
PLUG_X_MM = (-11.2, 15.5)
PLUG_HALF_MM = 15.9
PLUG_Z_MM = (55.0, 73.0)
PLUG_TAB_MM = 4.9  # the tabs that take the neck screws
PLUG_END_MM = 3.0
NECK_PILOT_RADIUS_MM = 0.9  # an M2 screw cuts its own thread
NECK_SCREW_ROWS_MM = (57.27, 61.02)  # either row may be the one a wall uses
# The shoulder stage hooks under the yaw servo and sweeps z 131 to 144 out to 51 mm from
# the axis, except behind x = -35. The upright leans back under the trunk's cover line
# and is behind x = -36 before it reaches that height. Side profile: (x, z).
UPRIGHT_FRONT_MM = [(15.5, 71.0), (-36.0, 127.0), (-36.0, 149.0)]
UPRIGHT_BACK_MM = [(-11.2, 71.0), (-11.2, 88.0), (-52.0, 128.0), (-52.0, 149.0)]
UPRIGHT_HALF_MM = 15.9  # as wide as the plug; the servo screws' driver paths run inside the side walls
UPRIGHT_WALL_MM = 3.0
UPRIGHT_WEB_MM = 4.0  # along x; the web is the channel's front, its back is open
# Behind this the stock holder is a 1 mm cover over the SO-101 base's electronics. It is left off.
HOLDER_BACK_MM = -52.0
DECK_MM = ((-51.7, -36.0), (-22.0, 22.1), (147.5, 150.5))  # joins the holder's side walls to the upright
# The STS3215 has two tapped M2 holes on its back face near the tail. A shelf under the
# tail carries the servo's weight and takes two screws from below.
YAW_SERVO_HOLES_MM = [(-20.30, 10.27), (-20.30, -10.27)]  # in the servo's frame
YAW_SERVO_BACK_FACE_MM = -15.9
SHELF_MM = 2.9
SHELF_X_MM = (-36.5, -29.2)  # the servo's back is flat only this far from the tail; then its cover stands proud
SHELF_HALF_MM = 13.5
SERVO_SCREW_RADIUS_MM = 1.2


def _yaw_servo_screws():
    """Where the two shelf screws enter the yaw servo: trunk frame, on its back face."""
    frame = stock.placement("shoulder_yaw_case")
    return [tuple(float(v) for v in (frame @ [x, y, YAW_SERVO_BACK_FACE_MM, 1.0])[:3]) for x, y in YAW_SERVO_HOLES_MM]


YAW_SERVO_SCREWS_MM = _yaw_servo_screws()


def _base_holder():
    """The SO-101 base motor holder, placed around the yaw servo as it is in the stock base."""
    holder, servo = so101("base", "base_motor_holder_so101_v1"), so101("base", "sts3215_03a_v1")
    placed = stock.moved(params.donor("Base_motor_holder_SO101"), stock.placement("shoulder_yaw_case") @ np.linalg.inv(servo) @ holder)
    return _join("base holder", placed.intersect(_box(HOLDER_BACK_MM, 100, -100, 100, 0, 300)))


def _extrude_xz(points, half):
    """A side profile given as (x, z) points, extruded to both sides of y = 0."""
    return cq.Workplane("XZ").polyline(points).close().extrude(half, both=True).val()


def neck_mount():
    """Neck socket mount: a plug in the trunk's neck socket, a channel upright, the SO-101 base holder."""
    (x0, x1), (z0, z1) = PLUG_X_MM, PLUG_Z_MM
    plug = _box(x0, x1, -PLUG_HALF_MM, PLUG_HALF_MM, z0, z1).cut(
        _box(x0 + PLUG_END_MM, x1 - PLUG_END_MM, -PLUG_HALF_MM + PLUG_TAB_MM, PLUG_HALF_MM - PLUG_TAB_MM, z0 - 1, z1 + 1)
    )
    profile = UPRIGHT_FRONT_MM + UPRIGHT_BACK_MM[::-1]
    hollow = [(x - UPRIGHT_WEB_MM, z + shift) for (x, z), shift in zip(profile, [-1, 0, 1, 1, 0, 0, -1])]
    upright = _extrude_xz(profile, UPRIGHT_HALF_MM).cut(_extrude_xz(hollow, UPRIGHT_HALF_MM - UPRIGHT_WALL_MM))
    (dx0, dx1), (dy0, dy1), (dz0, dz1) = DECK_MM
    deck = _box(dx0, dx1, dy0, dy1, dz0, dz1)
    middle = stock.placement("shoulder_yaw_case")[1, 3]
    top = stock.placement("shoulder_yaw_case")[2, 3] + YAW_SERVO_BACK_FACE_MM - 0.1
    shelf = _box(SHELF_X_MM[0], SHELF_X_MM[1], middle - SHELF_HALF_MM, middle + SHELF_HALF_MM, top - SHELF_MM, top)
    mount = _join("neck mount", _base_holder(), deck, upright, plug, shelf)
    for x, y, z in YAW_SERVO_SCREWS_MM:
        mount = mount.cut(cq.Solid.makeCylinder(SERVO_SCREW_RADIUS_MM, SHELF_MM + 2, cq.Vector(x, y, top - SHELF_MM - 1)))
        mount = mount.cut(cq.Solid.makeCylinder(DRIVER_RADIUS_MM, 70, cq.Vector(x, y, top - SHELF_MM - 70)))
    for x in sorted({centre[0] for centre in params.NECK_SCREWS_MM}):
        for z in NECK_SCREW_ROWS_MM:
            for sign in (1, -1):
                start = cq.Vector(x, sign * (PLUG_HALF_MM - PLUG_TAB_MM - 0.5), z)
                mount = mount.cut(cq.Solid.makeCylinder(NECK_PILOT_RADIUS_MM, PLUG_TAB_MM + 1.0, start, cq.Vector(0, sign, 0)))
    return _join("neck mount", mount)
