import cadquery as cq
import numpy as np
import pytest

import checks
import params
import stock


def box(x, y, z, dx, dy, dz):
    return cq.Solid.makeBox(dx, dy, dz, cq.Vector(x, y, z))


def test_wall_of_an_open_box_is_two_millimetres():
    shell = box(0, 0, 0, 20, 20, 20).cut(box(2, 2, 2, 16, 16, 18))
    assert abs(checks.min_wall_mm(shell) - 2.0) < 0.1


def test_wall_of_a_thinner_box_reads_thinner():
    shell = box(0, 0, 0, 20, 20, 20).cut(box(1, 1, 1, 18, 18, 19))
    assert abs(checks.min_wall_mm(shell) - 1.0) < 0.1


def test_clearance_between_two_boxes():
    a = checks.tessellate(box(0, 0, 0, 10, 10, 10))
    assert abs(checks.clearance_mm(a, checks.tessellate(box(13, 0, 0, 10, 10, 10))) - 3.0) < 0.05
    assert checks.clearance_mm(a, checks.tessellate(box(8, 0, 0, 10, 10, 10))) == 0.0


def test_clearance_ignores_skipped_regions():
    a = checks.tessellate(box(0, 0, 0, 10, 10, 10).fuse(box(10, 4, 4, 5, 2, 2)))  # a peg toward the other box
    b = checks.tessellate(box(16, 0, 0, 10, 10, 10))
    assert abs(checks.clearance_mm(a, b) - 1.0) < 0.05
    on_peg = lambda points: (points[:, 0] > 10.01) & (points[:, 0] < 15.5)
    assert abs(checks.clearance_mm(a, b, skip=on_peg) - 6.0) < 0.05


def test_clearance_refuses_a_skip_that_removes_a_whole_mesh():
    a = checks.tessellate(box(0, 0, 0, 10, 10, 10))
    b = checks.tessellate(box(13, 0, 0, 10, 10, 10))
    with pytest.raises(ValueError):
        checks.clearance_mm(a, b, skip=lambda points: points[:, 0] > 12)


def test_clearance_follows_a_moved_second_mesh():
    a = checks.tessellate(box(0, 0, 0, 10, 10, 10))
    b = checks.tessellate(box(13, 0, 0, 10, 10, 10))
    move = np.eye(4)
    move[0, 3] = 4.0
    assert abs(checks.clearance_mm(a, b, move_b=move) - 7.0) < 0.05
    assert abs(checks.clearance_mm(a, b) - 3.0) < 0.05


def test_hole_is_open_only_where_the_plate_is_drilled():
    plate = box(-10, -10, 0, 20, 20, 3).cut(cq.Solid.makeCylinder(1.6, 5, cq.Vector(0, 0, -1)))
    assert checks.hole_is_open(plate, (0, 0, 1.5), (0, 0, 1))
    assert not checks.hole_is_open(plate, (5, 0, 1.5), (0, 0, 1))


def test_a_point_in_empty_space_is_not_a_hole():
    plate = box(-10, -10, 0, 20, 20, 3)
    assert not checks.hole_is_open(plate, (30, 0, 1.5), (0, 0, 1))


def so101_wrist_pocket():
    """The under arm and the wrist motor holder of the stock SO-101, in the frame of the servo they hold."""
    from scipy.spatial.transform import Rotation

    def frame(position, wxyz):
        out = np.eye(4)
        out[:3, :3] = Rotation.from_quat(np.roll(wxyz, -1)).as_matrix()
        out[:3, 3] = position
        return out

    servo = frame([-122.4, 5.2, 18.7], [0, 0, 1, 0])  # collision/inputs/so101.xml, body lower_arm
    under = checks.tessellate(params.donor("Under_arm_SO101"))
    under.apply_transform(np.linalg.inv(servo) @ frame([-64.8499, -32, 18.2], [0, 1, 0, 0]))
    holder = checks.tessellate(params.donor("Motor_holder_SO101_Wrist"))
    holder.apply_transform(np.linalg.inv(servo) @ frame([-64.8499, -32, 18.0], [0, -1, 0, 0]))
    return under, holder


def test_the_stock_pocket_admits_its_servo():
    for part in so101_wrist_pocket():
        for out in (0, 4, 8, 12, 16, 20, 30, 40):  # seated, then pulled out along the servo's long axis
            depth = checks.servo_penetration_mm(part, "wrist_flex_case", shift=(out, 0, 0), placement=np.eye(4))
            assert depth <= checks.SERVO_ALLOWANCE_MM, (out, depth)


def test_the_stock_holder_is_snug():
    _, holder = so101_wrist_pocket()
    assert 0.3 < checks.servo_penetration_mm(holder, "wrist_flex_case", placement=np.eye(4)) <= 0.45


def test_a_block_in_the_servo_is_found():
    block = checks.tessellate(box(-30, -10, -5, 10, 5, 5))  # reaches 2.7 mm into the servo's tail
    assert abs(checks.servo_penetration_mm(block, "wrist_flex_case", placement=np.eye(4)) - 2.7) < 0.3
    assert checks.servo_penetration_mm(block, "wrist_flex_case", shift=(4, 0, 0), placement=np.eye(4)) == 0.0


def test_a_thin_blade_in_the_servo_is_found():
    blade = checks.tessellate(box(-30, 0, -5, 12, 0.4, 3))  # thinner than the spacing of surface samples
    assert checks.servo_penetration_mm(blade, "wrist_flex_case", placement=np.eye(4)) > 1.0


def test_connector_space_is_measured():
    low, high = stock.SERVO_CONNECTORS[0]
    plug = box(*low, *(np.subtract(high, low)))  # fills the first connector's space, in the servo frame
    assert abs(checks.connector_blocked_mm3(plug, "wrist_flex_case", placement=np.eye(4)) - 5 * 9 * 12) < 1.0
    assert checks.connector_blocked_mm3(box(40, 0, 0, 5, 5, 5), "wrist_flex_case", placement=np.eye(4)) < 1e-6


def test_bed_check_uses_the_two_smallest_extents():
    assert checks.fits_bed(box(0, 0, 0, 170, 60, 250))
    assert not checks.fits_bed(box(0, 0, 0, 190, 60, 250))
