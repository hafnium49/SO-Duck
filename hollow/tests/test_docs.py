import re

import cadquery as cq
import numpy as np
import pytest

import assemble
import build_parts
import horn_check
import loads
import params
import postures

JOINTS7 = ["shoulder yaw", "shoulder pitch", "elbow", "forearm roll", "wrist flex", "wrist roll", "gripper"]
PARTS = ("neck_socket_mount", "elbow_to_forearm_roll", "forearm_roll_to_wrist")
# The orientation that BUILD.md recommends for each part.
ORIENTATION = {"neck_socket_mount": "+y", "elbow_to_forearm_roll": "+y", "forearm_roll_to_wrist": "+x"}


@pytest.fixture(scope="module")
def build(root):
    return (root / "hollow" / "BUILD.md").read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def readme(root):
    return (root / "hollow" / "README.md").read_text(encoding="utf-8")


def test_build_guide_covers_every_joint_and_part(build):
    text = build.lower()
    assert all(text.count(joint) >= 2 for joint in JOINTS7)  # once in the servo list, once in horn indexing
    assert all(name in text for name in PARTS)
    assert "14.4" in text and "not yet printed" in text


def test_build_guide_masses_are_the_solids(build, readme):
    for name in PARTS:
        solid = cq.importers.importStep(str(build_parts.OUT / f"{name}.step")).val()
        grams = f"{params.printed_grams(solid.Volume(), solid.Area()):.1f} g"
        assert re.search(rf"`{name}` \| {re.escape(grams)} \|", build), (name, grams)
        assert grams in readme, (name, grams)


def test_build_guide_tooth_checks_are_computed(build, robot):
    for label, _, distance in horn_check.table(robot):
        rows = [line for line in build.splitlines() if line.lower().startswith(f"| {label} |") and line.endswith(" mm |")]
        assert len(rows) == 1 and rows[0].endswith(f"| {distance:.0f} mm |"), (label, distance)
    origins, tool = horn_check.zero_pose(robot)
    assert f"the elbow {origins['forearm_pitch'][2]:.0f} mm above" in build
    assert f"the tool point {-tool[0]:.0f} mm behind the trunk's origin and {tool[2]:.0f} mm above it" in build


def test_build_guide_orientations_are_computed(build):
    for name, up in ORIENTATION.items():
        overhang, bed, height = build_parts.orientations(name)[up]
        row = next(line for line in build.splitlines() if line.startswith(f"| `{name}` | {up} up"))
        assert f"{height:.0f} mm tall" in row and f"{overhang:.1f} cm² of overhang" in row, (name, row)


def test_readme_figures_match_the_programs(readme, robot):
    q, _ = postures.read_targets()
    reached = 96 - len(postures.UNREACHED)
    covered = reached - len(postures.OVER_LIMIT)
    assert f"**{covered} of the 96 targets are reached within the load limit.**" in readme
    assert f"| {reached}/96 |" in readme and f"| {covered}/96 |" in readme
    empty, loaded = loads.stock_baseline()
    assert f"**{empty:.3f} N m empty and {loaded:.3f} N m with 100 g**" in readme
    rows = np.delete(q, list(postures.UNREACHED) + list(postures.OVER_LIMIT), axis=0)
    worst = postures.shoulder_torque(robot, rows).max(), postures.shoulder_torque(robot, rows, params.PAYLOAD_KG).max()
    assert f"The worst of the {covered} asks {worst[0]:.3f} N m empty and {worst[1]:.3f} N m with 100 g." in readme
    for target in postures.OVER_LIMIT:
        a = postures.shoulder_torque(robot, q[target])[0]
        b = postures.shoulder_torque(robot, q[target], params.PAYLOAD_KG)[0]
        assert f"| {target} | {a:.3f} N m | {b:.3f} N m |" in readme, target
    assert f"**{postures.overhead_height_mm(robot):.0f} mm** above the floor" in readme
    assert f"{len(postures.fold_path())} frames lead from overhead to parked" in readme
    none = ", ".join(str(t) for t in postures.UNREACHED[:-1]) + f" and {postures.UNREACHED[-1]}"
    over = ", ".join(str(t) for t in postures.OVER_LIMIT[:-1]) + f" and {postures.OVER_LIMIT[-1]}"
    assert f"exactly targets {none} fail and exactly targets {over} are over the load limit" in readme


def test_readme_ranges_are_the_model_ranges(readme):
    low, high = params.RANGES_DEG["shoulder_pitch"]
    assert f"shoulder pitch −{-low:.0f}° to {high:.0f}°" in readme
    low, high = params.RANGES_DEG["wrist_roll"]
    assert f"wrist roll −{-low:.0f}° to {high:.0f}°" in readme
    for joint, label in (("shoulder_yaw", "Shoulder yaw"), ("forearm_pitch", "elbow"), ("forearm_roll", "forearm roll"), ("wrist_flex", "wrist flex")):
        assert f"{label} ±{params.RANGES_DEG[joint][1]:.0f}°" in readme, joint


def test_no_readiness_claim(root, build, readme):
    top = (root / "README.md").read_text(encoding="utf-8")
    for text in (build, readme, top):
        assert "ready to print" not in text.lower() and "ready to build" not in text.lower()
    assert "not yet printed" in readme.lower() and "not yet printed" in top.lower()
    assert set(assemble.CUSTOM.values()) == set(PARTS)
