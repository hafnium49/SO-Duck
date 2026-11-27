"""Put the built parts and the stored postures into hollow/viewer.html.

    python hollow/viewer_build.py

Run from the repository root after hollow/cad/build_parts.py. Needs shapely besides
crosswise/requirements.txt, for the faces of the cutaways.

The page's script and every mesh of the trunk, the legs, the stock arm links, the gripper
and the unmoved servos stay as they are. This script replaces
- the three part meshes, from hollow/parts/*.stl on the viewer's 0.1 mm grid,
- their cutaways, cut at the same planes as before,
- the two servos that moved with the shorter links,
- every clip, from the CSV files (postures.viewer_clips),
- five texts of the page that describe the parts or quote figures.
"""

import base64
import gzip
import hashlib
import json
import re

import numpy as np

import postures
from inspect_viewer import HERE, PART_MESHES, decode_mesh, grid_triangles, load_viewer, read_stl

# Viewer colour of each part, the axis its cutaway plane is normal to, and the plane's place in mm.
# The half on the positive side is shown.
CUTS = {"mount": (1, 0.5), "forearm": (2, 18.6), "wrist": (1, 0.4)}
# Servos that moved in their body's frame, and by how much, in units of the 0.1 mm grid.
MOVED_SERVOS = {
    "forearm_roll_case": (-round(1e4 * postures.params.FOREARM_SHORTEN), 0, 0),
    "wrist_flex_case": (round(1e4 * postures.params.WRIST_SIDE_SHORTEN), 0, -round(1e4 * postures.params.WRIST_AXIAL_SHORTEN)),
}
# SHA-256 of the page with its data block and the five texts taken out. It is that of the
# page before the parts were rebuilt: the script itself has not changed.
SHELL_SHA256 = "a2ab1ead04c2780c67ca52d40a9e00b3a3ae17a98b692c231c81bdbc033f8a56"


def texts(robot):
    count = 96 - len(postures.UNREACHED)
    none = ", ".join(str(t) for t in postures.UNREACHED[:-1]) + f" and {postures.UNREACHED[-1]}"
    return {
        "mount": "A plug fills the Duck neck socket and takes its four screws. A channel upright leans back to the SO-101 base holder, which wraps the yaw motor.",
        "forearm": "The round ends of the SO-101 under arm seat on the elbow horns. Two plates run outside the upper arm to the SO-101 seat that holds the roll motor.",
        "wrist": "An SO-101 cheek sits on the roll horn. A three-sided channel carries the SO-101 seat that holds the flex motor.",
        "fold": f"The gripper starts {postures.overhead_height_mm(robot):.1f} mm above the floor and finishes on the rear pad. The fixed fingertip seats on the rear pad.",
        "targets": (
            "Each target specifies gripper position and orientation. With both feet planted and a required gap between the legs and the trunk, "
            f"{count} of the 96 stored endpoints pass the checks in hollow/README.md, {count - len(postures.OVER_LIMIT)} of them within the shoulder load of the stock arm. "
            f"Targets {none} have no passing posture."
        ),
    }


def split_page(html):
    """The page with its data block and the five texts replaced by marks, and the texts found."""
    found = {}

    def take(match):
        found[match.group(1)] = match.group(3)
        return f"{match.group(1)}:{match.group(2)}@{match.group(1)}@{match.group(2)}"

    shell = re.sub(r'(<script id="payload"[^>]*>).*?(</script>)', r"\1\2", html, count=1, flags=re.S)
    start, end = shell.index("const partInfo="), shell.index("const state=")
    middle = shell[start:end]
    for key in ("mount", "forearm", "wrist", "fold", "targets"):
        middle, count = re.subn(rf"\b({key}):(['\"])(.*?)\2", take, middle, count=1)
        assert count == 1, key
    return shell[:start] + middle + shell[end:], found


def shell_sha256(html):
    return hashlib.sha256(split_page(html)[0].encode()).hexdigest()


def save(data, html, new_texts=None):
    """Write the page with the data and, if given, the five texts."""
    blob = base64.b64encode(gzip.compress(json.dumps(data, separators=(",", ":")).encode(), 9, mtime=0)).decode()
    html, count = re.subn(r'(<script id="payload"[^>]*>).*?(</script>)', lambda m: m.group(1) + blob + m.group(2), html, count=1, flags=re.S)
    assert count == 1
    if new_texts:
        shell, old = split_page(html)
        for key, text in new_texts.items():
            assert "'" not in text and '"' not in text and "\\" not in text, key
            mark = f"@{key}@"
            assert shell.count(mark) == 1, key
            shell = shell.replace(mark, text)
        html = re.sub(r'(<script id="payload"[^>]*>)(</script>)', lambda m: m.group(1) + blob + m.group(2), shell, count=1)
    (HERE / "viewer.html").write_text(html, encoding="utf-8", newline="\n")


def store(group, vertices, faces):
    """Put a mesh on the 0.1 mm grid into a group of the data, keeping its other entries."""
    assert len(vertices) <= 65535 and np.abs(vertices).max() < 32768
    group["v"] = base64.b64encode(vertices.astype("<i2").tobytes()).decode()
    group["f"] = base64.b64encode(faces.astype("<u2").tobytes()).decode()
    group["c"] = [round(float(v), 5) for v in (vertices.min(0) + vertices.max(0)) / 2 * 1e-4]


def indexed(triangles):
    """Vertices and faces of a triangle soup on the integer grid."""
    vertices, index = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    return vertices, index.reshape(-1, 3)


def cutaway(triangles_mm, axis, place):
    """The half of a part on the positive side of a plane, and the cut face, as triangles in mm."""
    import shapely
    import trimesh

    mesh = trimesh.Trimesh(*indexed_float(triangles_mm), process=True)
    normal, origin = np.eye(3)[axis], np.eye(3)[axis] * place
    half = trimesh.intersections.slice_mesh_plane(mesh, normal, origin, cap=False)
    flat, to_space = mesh.section(plane_origin=origin, plane_normal=normal).to_2D()
    face = []
    for polygon in flat.polygons_full:
        for triangle in shapely.constrained_delaunay_triangles(polygon).geoms:
            corners = np.array(triangle.exterior.coords)[:3]
            corners = (to_space @ np.c_[corners, np.zeros(3), np.ones(3)].T).T[:, :3]
            if np.cross(corners[1] - corners[0], corners[2] - corners[0]) @ normal > 0:
                corners = corners[::-1]  # the face looks toward the half that was taken away
            face.append(corners)
    return half.triangles, np.array(face)


def indexed_float(triangles):
    vertices, index = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    return vertices, index.reshape(-1, 3)


def main():
    html = (HERE / "viewer.html").read_text(encoding="utf-8")
    assert shell_sha256(html) == SHELL_SHA256, "the page's script differs from the one this build was written for"
    data = load_viewer()
    bodies = data["bodies"]
    for color, filename in PART_MESHES.items():
        triangles = read_stl(HERE / "parts" / filename)
        whole = next(g for g in data["meshes"] if g["color"] == color and not g.get("sectionOf"))
        store(whole, *indexed(grid_triangles(triangles)))
        half, face = cutaway(triangles, *CUTS[color])
        shell = next(g for g in data["meshes"] if g.get("sectionOf") == color and g["color"] == color)
        cap = next(g for g in data["meshes"] if g.get("sectionOf") == color and g["color"] == "section")
        store(shell, *indexed(grid_triangles(half)))
        store(cap, *indexed(grid_triangles(face)))
        print(f"{filename}: {len(decode_mesh(whole)[1])} triangles, cutaway {len(decode_mesh(shell)[1])} and a cut face of {len(decode_mesh(cap)[1])}")
    for body, shift in MOVED_SERVOS.items():
        group = next(g for g in data["meshes"] if bodies[g["b"]] == body)
        vertices, faces = decode_mesh(group)
        if group.get("moved") != list(shift):  # move once
            store(group, vertices + shift, faces)
            group["moved"] = list(shift)
    robot = postures.robot_model()
    postures.viewer_clips(data, robot)
    save(data, html, texts(robot))
    print("Wrote viewer.html")


if __name__ == "__main__":
    main()
