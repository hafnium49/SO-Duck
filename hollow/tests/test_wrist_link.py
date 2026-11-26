import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import checks
import params
import parts
import stock

ROLL_TURN_DEG = range(-155, 156, 5)


@pytest.fixture(scope="module")
def link():
    return parts.wrist_link()


@pytest.fixture(scope="module")
def mesh(link):
    return checks.tessellate(link)


def test_one_printable_solid(link, mesh):
    assert link.isValid() and len(link.Solids()) == 1 and mesh.is_watertight
    assert checks.min_wall_mm(link) >= params.MIN_WALL_MM
    assert checks.fits_bed(link)


def test_wrist_servo_sits_at_the_chosen_offset(mesh):
    frame = parts.wrist_servo_frame()
    moved = stock.placement("wrist_flex_case")[:3, 3] + [1000 * params.WRIST_SIDE_SHORTEN, 0, -1000 * params.WRIST_AXIAL_SHORTEN]
    assert np.allclose(frame[:3, 3], moved)
    assert checks.servo_penetration_mm(mesh, "wrist_flex_case", placement=frame) <= checks.SERVO_ALLOWANCE_MM
    seated = stock.servo_mesh().copy()
    seated.apply_transform(frame)
    assert checks.clearance_mm(mesh, seated) < 0.3


def test_wrist_servo_can_be_inserted(mesh):
    for out in range(4, 44, 4):
        depth = checks.servo_penetration_mm(mesh, "wrist_flex_case", shift=(out, 0, 0), placement=parts.wrist_servo_frame())
        assert depth <= checks.SERVO_ALLOWANCE_MM, (out, depth)


def test_connectors_are_open(link):
    assert checks.connector_blocked_mm3(link, "wrist_flex_case", placement=parts.wrist_servo_frame()) < 1.0


@pytest.mark.parametrize("x, y", parts.HORN_HOLES_MM)
def test_seat_takes_the_roll_horn_screws(link, x, y):
    z, length = parts.ROLL_SEAT_BORE_MM
    assert checks.hole_is_open(link, (x, y, z), (0, 0, 1), radius=1.0, depth=length)  # M2 screws


@pytest.mark.parametrize("x, y", parts.HORN_HOLES_MM)
def test_a_screwdriver_reaches_the_roll_horn_screws(link, x, y):
    # with the wrist servo not yet in its pocket, straight down the roll axis' direction
    z, length = parts.ROLL_DRIVER_PATH_MM
    assert checks.hole_is_open(link, (x, y, z), (0, 0, 1), radius=2.0, depth=length)  # a 4 mm driver shaft


def test_roll_turn_keeps_clearance(mesh):
    position, quaternion = params.joint_frames(params.FOREARM_SHORTEN, params.WRIST_SIDE_SHORTEN, params.WRIST_AXIAL_SHORTEN)["forearm_roll"]
    into = np.eye(4)
    into[:3, :3] = Rotation.from_quat(np.roll(quaternion, -1)).as_matrix()
    into[:3, 3] = 1000 * np.asarray(position)
    into = np.linalg.inv(into)  # from the forearm frame into the roll body's frame
    forearm = checks.tessellate(parts.forearm_link())
    forearm.apply_transform(into)
    servo = stock.servo_mesh().copy()
    servo.apply_transform(into @ parts.roll_servo_frame())
    # the seat sits on the roll servo's horn; skip that contact
    seat = lambda points: np.hypot(points[:, 0], points[:, 1]) < 13.0
    for angle in ROLL_TURN_DEG:
        turn = np.eye(4)
        turn[:3, :3] = Rotation.from_euler("z", -angle, degrees=True).as_matrix()
        assert checks.clearance_mm(mesh, forearm, move_b=turn) >= params.MOVING_CLEARANCE_MM, angle
        assert checks.clearance_mm(mesh, servo, skip=seat, move_b=turn) >= params.MOVING_CLEARANCE_MM, angle
