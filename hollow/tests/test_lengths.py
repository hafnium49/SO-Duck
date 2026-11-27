import csv

import numpy as np
import pytest

import lengths
import loads
import params
import resolve


def test_smallest_passing_prefers_less_shortening():
    rows = [
        dict(forearm=0.040, wrist_side=0.040, shoulder_empty=0.91, shoulder_loaded=1.29, covered=91),
        dict(forearm=0.045, wrist_side=0.045, shoulder_empty=0.85, shoulder_loaded=1.20, covered=90),
        dict(forearm=0.055, wrist_side=0.050, shoulder_empty=0.82, shoulder_loaded=1.15, covered=89),
    ]
    assert lengths.smallest_passing(rows, (0.86, 1.27))["forearm"] == 0.045
    assert lengths.smallest_passing(rows[:1], (0.86, 1.27)) is None


def test_smallest_passing_needs_the_coverage_gate():
    rows = [
        dict(forearm=0.045, wrist_side=0.045, shoulder_empty=0.85, shoulder_loaded=1.20, covered=84),
        dict(forearm=0.050, wrist_side=0.045, shoulder_empty=0.84, shoulder_loaded=1.19, covered=85),
    ]
    assert lengths.smallest_passing(rows, (0.86, 1.27))["forearm"] == 0.050


def test_ties_go_to_the_shorter_forearm_cut():
    rows = [
        dict(forearm=0.050, wrist_side=0.040, shoulder_empty=0.8, shoulder_loaded=1.2, covered=90),
        dict(forearm=0.045, wrist_side=0.045, shoulder_empty=0.8, shoulder_loaded=1.2, covered=90),
    ]
    assert lengths.smallest_passing(rows, (0.86, 1.27))["forearm"] == 0.045


@pytest.mark.parametrize("rank, expected", [(lambda q: q[:, 0], 0.0), (None, 1.0)])
def test_rank_orders_the_candidates(monkeypatch, rank, expected):
    # Two candidates for target 0, both passing. Their support margins favour the first.
    def passes(robot, pairs, q, targets, positions, rotations, full=True):
        return np.ones(len(q), bool), np.where(q[:, 0] == 1.0, 0.02, 0.01)

    monkeypatch.setattr(resolve, "passes", passes)
    found = {0: np.array([[1.0] + [0.0] * 15, [0.0] * 16])}
    chosen = resolve.choose(None, [], found, None, None, rank=rank)
    assert chosen[0][0] == expected


def test_chosen_lengths_are_recorded_and_pass(root):
    with (root / "hollow/cad/length_study.csv").open() as stream:
        row = next(r for r in csv.DictReader(stream) if r["chosen"] == "1")
    assert float(row["forearm"]) == params.FOREARM_SHORTEN
    assert params.FOREARM_RANGE[0] <= params.FOREARM_SHORTEN <= params.FOREARM_RANGE[1]
    assert float(row["wrist_side"]) == params.WRIST_SIDE_SHORTEN
    assert params.WRIST_SIDE_RANGE[0] <= params.WRIST_SIDE_SHORTEN <= params.WRIST_SIDE_RANGE[1]
    assert float(row["wrist_axial"]) == params.WRIST_AXIAL_SHORTEN <= params.WRIST_AXIAL_MAX
    empty, loaded = loads.stock_baseline()
    assert float(row["shoulder_empty"]) <= empty and float(row["shoulder_loaded"]) <= loaded
    assert int(row["covered"]) >= 85


def test_written_model_has_the_requested_frames_and_masses(root, tmp_path):
    import mujoco

    import assemble

    frames = params.joint_frames(0.040, 0.040, 0.012)
    masses = assemble.estimated_masses(frames)
    path = root / "hollow" / "_test_model.xml"
    try:
        assemble.write_model(path, frames, masses, assemble.display_meshes(), assemble.stored_ranges())
        model = mujoco.MjModel.from_xml_path(str(path))
    finally:
        path.unlink(missing_ok=True)
    assert np.allclose(model.body("forearm_roll_linkage").pos, frames["forearm_roll"][0])
    assert np.allclose(model.body("wrist_flex_linkage").pos, frames["wrist_flex"][0])
    assert np.isclose(model.body("forearm_pitch_linkage").mass, masses["forearm_pitch_linkage"][0])
    base = assemble.estimated_masses(params.joint_frames(0, 0, 0))
    # the roll servo and half the link move back with the joint, so the centre moves back
    assert masses["forearm_pitch_linkage"][1][0] < base["forearm_pitch_linkage"][1][0] - 0.020
    assert np.isclose(sum(m for m, _ in masses.values()), sum(m for m, _ in base.values()))
