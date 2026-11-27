import numpy as np
import pytest
import trimesh

import postures
import viewer_build
from inspect_viewer import EXPECTED_FRAMES, HERE, PART_MESHES, decode_mesh, load_viewer, read_stl


@pytest.fixture(scope="module")
def data():
    return load_viewer()


def group(data, color, section=False):
    if section:
        return next(g for g in data["meshes"] if g.get("sectionOf") == color and g["color"] == color)
    return next(g for g in data["meshes"] if g["color"] == color and not g.get("sectionOf"))


@pytest.mark.parametrize("color", PART_MESHES)
def test_viewer_meshes_follow_the_parts(data, color):
    triangles = read_stl(HERE / "parts" / PART_MESHES[color])
    part = trimesh.Trimesh(triangles.reshape(-1, 3), np.arange(3 * len(triangles)).reshape(-1, 3), process=True)
    for section in (False, True):
        vertices, faces = decode_mesh(group(data, color, section))
        points = vertices[np.unique(faces)] / 10.0
        distance = trimesh.proximity.closest_point(part, points[:: max(1, len(points) // 4000)])[1]
        assert distance.max() < 0.15, (color, section)


@pytest.mark.parametrize("color", PART_MESHES)
def test_cutaways_are_cut_at_their_planes(data, color):
    axis, place = viewer_build.CUTS[color]
    vertices, _ = decode_mesh(group(data, color, section=True))
    assert vertices[:, axis].min() == round(10 * place)  # nothing behind the plane, and the cut reaches it
    cap = next(g for g in data["meshes"] if g.get("sectionOf") == color and g["color"] == "section")
    flat, faces = decode_mesh(cap)
    assert len(faces) > 20 and np.all(flat[:, axis] == round(10 * place))


def test_viewer_meshes_fit_the_index_type(data):
    for item in data["meshes"]:
        vertices, faces = decode_mesh(item)
        assert len(vertices) <= 65535 and faces.max() < len(vertices) and faces.min() >= 0


def test_moved_servos_sit_in_their_seats(data, robot):
    for body, name in (("forearm_roll_case", "forearm_roll_link"), ("wrist_flex_case", "wrist_flex_link")):
        vertices, _ = decode_mesh(next(g for g in data["meshes"] if data["bodies"][g["b"]] == body))
        built = robot.mesh[body].reshape(-1, 3) * 1000
        assert np.abs(vertices.min(0) / 10 - built.min(0)).max() < 0.3 and np.abs(vertices.max(0) / 10 - built.max(0)).max() < 0.3, body


def test_viewer_shows_the_csv_postures(data, robot):
    postures.replay_viewer(robot)
    for name, frames in EXPECTED_FRAMES.items():
        assert next(c for c in data["poses"] if c["name"] == name)["n"] == frames


def test_viewer_script_is_untouched_but_for_five_texts(robot):
    html = (HERE / "viewer.html").read_text(encoding="utf-8")
    assert viewer_build.shell_sha256(html) == viewer_build.SHELL_SHA256
    assert viewer_build.split_page(html)[1] == viewer_build.texts(robot)
