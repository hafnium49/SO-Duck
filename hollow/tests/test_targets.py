import numpy as np

import loads
import params
import postures
import resolve

import model

GATE = 85  # targets that must be reached within the load limit


def checked(robot):
    q, source = postures.read_targets()
    positions, rotations, _ = postures.target_frames()
    ok, _ = resolve.passes(robot, model.tested_pairs(robot), q, np.arange(96), positions, rotations)
    return q, source, ok


def test_coverage_gate(robot):
    q, source, ok = checked(robot)
    assert sorted(np.flatnonzero(~ok)) == sorted(postures.UNREACHED)
    assert not set(postures.UNREACHED) & set(postures.OVER_LIMIT)
    assert ok.sum() - len(postures.OVER_LIMIT) >= GATE
    assert set(source) == {"re-solved"}


def test_stored_postures_meet_the_load_limit(robot, passing_q):
    empty, loaded = loads.stock_baseline()
    assert len(passing_q) == 96 - len(postures.UNREACHED) - len(postures.OVER_LIMIT)
    assert np.abs(loads.arm_torques(robot, passing_q)[:, loads.SHOULDER]).max() <= empty
    assert np.abs(loads.arm_torques(robot, passing_q, params.PAYLOAD_KG)[:, loads.SHOULDER]).max() <= loaded


def test_over_limit_targets_are_over_the_limit(robot):
    q, _ = postures.read_targets()
    for target in postures.OVER_LIMIT:
        assert not postures.within_limit(robot, q[target])[0], target
