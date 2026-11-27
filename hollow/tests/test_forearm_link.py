import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import checks
import params
import parts
import stock

ELBOW_SWING_DEG = range(-95, 96, 5)


@pytest.fixture(scope="module")
def link():
    return parts.forearm_link()


@pytest.fixture(scope="module")
def mesh(link):
    return checks.tessellate(link)


def parent_at(angle_deg):
    """Where the upper arm's parts are, in the forearm frame, with the elbow at an angle."""
    position, quaternion = params.FRAME["forearm_pitch"]
    frame = np.eye(4)
    frame[:3, :3] = Rotation.from_quat(np.roll(quaternion, -1)).as_matrix()
    frame[:3, 3] = 1000 * np.asarray(position)
    turn = np.eye(4)
    turn[:3, :3] = Rotation.from_euler("z", -angle_deg, degrees=True).as_matrix()
    return turn @ np.linalg.inv(frame)


def test_one_printable_solid(link, mesh):
    assert link.isValid() and len(link.Solids()) == 1 and mesh.is_watertight
    assert checks.min_wall_mm(link) >= params.MIN_WALL_MM
    assert checks.fits_bed(link)


def test_roll_servo_sits_at_the_chosen_length(mesh):
    frame = parts.roll_servo_frame()
    shortened = stock.placement("forearm_roll_case")[:3, 3] - [1000 * params.FOREARM_SHORTEN, 0, 0]
    assert np.allclose(frame[:3, 3], shortened)
    assert checks.servo_penetration_mm(mesh, "forearm_roll_case", placement=frame) <= checks.SERVO_ALLOWANCE_MM
    seated = stock.servo_mesh().copy()
    seated.apply_transform(frame)
    assert checks.clearance_mm(mesh, seated) < 0.3  # the pocket holds the servo, it does not float near it


def test_roll_servo_can_be_inserted(mesh):
    for out in range(4, 44, 4):  # pulled out along the servo's long axis, away from the pocket
        depth = checks.servo_penetration_mm(mesh, "forearm_roll_case", shift=(out, 0, 0), placement=parts.roll_servo_frame())
        assert depth <= checks.SERVO_ALLOWANCE_MM, (out, depth)


def test_roll_servo_is_screwed_to_its_seat(link):
    assert len(parts.seat_screws(parts.roll_servo_frame())) == 4
    for centre, axis in parts.seat_screws(parts.roll_servo_frame()):
        assert checks.hole_is_open(link, centre, axis, radius=0.9, depth=5.0), centre  # the stock 2 mm passage
        assert not checks.hole_is_open(link, centre, axis, radius=1.4, depth=5.0), centre  # with a wall under the screw's head


def test_seat_holds_the_roll_servo_on_all_sides(mesh):
    # pushed 2 mm toward the tail, either flank or either face, the servo meets the seat
    for push in ((-2, 0, 0), (0, 2, 0), (0, -2, 0), (0, 0, 2), (0, 0, -2)):
        assert checks.servo_penetration_mm(mesh, "forearm_roll_case", shift=push, placement=parts.roll_servo_frame()) > 1.5, push


def test_connectors_are_open(link):
    assert checks.connector_blocked_mm3(link, "forearm_roll_case", placement=parts.roll_servo_frame()) < 1.0


@pytest.mark.parametrize("z, length", parts.ELBOW_BORES_MM)
@pytest.mark.parametrize("x, y", parts.HORN_HOLES_MM)
def test_a_screwdriver_reaches_the_elbow_horn_screws(link, x, y, z, length):
    assert checks.hole_is_open(link, (x, y, z), (0, 0, 1), radius=1.45, depth=length)  # M3 screws


def test_round_ends_sit_on_the_elbow_servo(mesh):
    placement = parent_at(0.0) @ stock.placement("forearm_pitch_case")
    assert checks.servo_penetration_mm(mesh, "forearm_pitch_case", placement=placement) <= 0.1
    assert checks.clearance_mm(mesh, stock.mesh("forearm_pitch_case"), move_b=parent_at(0.0)) < 0.05  # they touch


def test_elbow_swing_keeps_clearance(mesh):
    upper_arm, elbow_servo = stock.mesh("forearm_pitch_link"), stock.mesh("forearm_pitch_case")
    # the cheeks sit on the elbow servo's horn and idler; skip that seat
    seat = lambda points: np.hypot(points[:, 0], points[:, 1]) < parts.CHEEK_RADIUS_MM + 1.0
    for angle in ELBOW_SWING_DEG:
        move = parent_at(angle)
        assert checks.clearance_mm(mesh, upper_arm, move_b=move) >= params.MOVING_CLEARANCE_MM, angle
        assert checks.clearance_mm(mesh, elbow_servo, skip=seat, move_b=move) >= params.MOVING_CLEARANCE_MM, angle


def test_roll_servo_clears_the_upper_arm(mesh):
    seated = stock.servo_mesh().copy()
    seated.apply_transform(parts.roll_servo_frame())
    upper_arm = stock.mesh("forearm_pitch_link")
    for angle in ELBOW_SWING_DEG:
        assert checks.clearance_mm(seated, upper_arm, move_b=parent_at(angle)) >= params.MOVING_CLEARANCE_MM, angle
