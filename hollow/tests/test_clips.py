import numpy as np
import pytest

import params
import postures

import model


@pytest.fixture(scope="module")
def pairs(robot):
    return model.tested_pairs(robot)


def test_overhead_is_on_the_centre_line(robot, pairs):
    rows = postures.at_reference(postures.read_arm("overhead"))
    frames = postures.fixed_legs(robot, pairs, rows, "Overhead")
    point = 1000 * postures.tool(frames)[0, :3, 3]
    assert np.abs(point[:2] - params.OVERHEAD_XY_MM).max() < 0.3
    assert abs(point[2] - postures.overhead_height_mm(robot)) < 1e-6
    assert abs(point[2] - round(point[2])) < 0.01  # found on whole millimetres


def test_overhead_is_the_highest_that_passes(robot, pairs):
    height = round(postures.overhead_height_mm(robot))
    assert len(postures.overhead_postures(robot, pairs, height))
    assert not len(postures.overhead_postures(robot, pairs, height + 2))


def test_parked_fingertip_rests_on_the_pad(robot, pairs):
    rows = postures.at_reference(postures.read_arm("park"))
    frames = postures.fixed_legs(robot, pairs, rows, "Parked", pad_contact=[0])
    sink, jaw = postures.pad_rest(robot, frames)
    assert 0 < sink < 1 and jaw > 0
    mm, deg = postures.offsets(postures.tool(frames), postures.frame(params.PARK_TOOL_MM, params.PARK_TOOL_QUAT)[None])
    assert mm.max() < 0.01 and deg.max() < 0.01
    with pytest.raises(AssertionError):  # without the allowance the pad contact is a collision
        postures.fixed_legs(robot, pairs, rows, "Parked")


def test_fold_is_clear_and_fine_grained(robot, pairs):
    path = postures.fold_path()
    assert np.abs(np.diff(np.rad2deg(path[:, 10:16]), axis=0)).max() <= 0.5 + 1e-9
    assert np.allclose(path[0, 10:16], postures.read_arm("overhead")[0]) and np.allclose(path[-1, 10:16], postures.read_arm("park")[0])
    postures.fixed_legs(robot, pairs, path, "Fold", pad_contact=[len(path) - 1])


def test_pickup_holds_one_crouch_on_a_vertical_line(robot, pairs):
    q = postures.replay_pickup(robot, pairs)
    assert q.shape == (81, 17)
    assert np.ptp(q[:, :10], axis=0).max() < 1e-9
