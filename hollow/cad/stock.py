"""Where each stock SO-101 part and each servo sits in the hollow-arm model.

    python hollow/cad/stock.py    # fit every part and rewrite placements.json

Each stock link is an SO-101 donor solid, and each servo is the STS3215 mesh of the
repository. Their places in the model's body frames are found by fitting them to the
display meshes the viewer stores (hollow/meshes), which are coarse copies of the same
shapes. Lengths are millimetres.
"""

import itertools
import json
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

import params

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
STOCK = {
    "shoulder_pitch_link": "Rotation_Pitch_SO101",
    "shoulder_collar": "Motor_holder_SO101_Base",
    "forearm_pitch_link": "Upper_arm_SO101",
    "wrist_roll_link": "Wrist_Roll_Pitch_SO101",
}
SERVOS = (
    "shoulder_yaw_case",
    "shoulder_pitch_case",
    "forearm_pitch_case",
    "forearm_roll_case",
    "wrist_flex_case",
    "wrist_roll_case",
)
# In the servo mesh's own frame, millimetres. The horn turns about an axis through this
# point along this direction; the idler is on the same axis on the opposite face.
SERVO_AXIS = ((12.5, 0.0, 0.0), (0.0, 0.0, 1.0))
# Space a cable plug needs under the idler-side cover, as (low corner, high corner).
SERVO_CONNECTORS = (
    ((-4.0, -9.0, -29.4), (1.0, 0.0, -17.4)),
    ((-4.0, 0.0, -29.4), (1.0, 9.0, -17.4)),
)


def tessellate(solid, tolerance=0.02, angular=0.1):
    """A CadQuery solid as a triangle mesh in the same units."""
    from OCP.BRepTools import BRepTools

    BRepTools.Clean_s(solid.wrapped)  # otherwise a triangulation made earlier, finer or coarser, is reused
    vertices, faces = solid.tessellate(tolerance, angular)
    mesh = trimesh.Trimesh([v.toTuple() for v in vertices], faces, process=True)
    # The SO-101 solids come out with a few vertices doubled a ten-thousandth of a
    # millimetre apart, which leaves hairline cracks. Joining them closes the mesh.
    mesh.merge_vertices(digits_vertex=4)
    mesh.update_faces(mesh.nondegenerate_faces())
    return mesh


@lru_cache(maxsize=None)
def servo_mesh(horn=True):
    """The STS3215 in its own frame, millimetres."""
    name = "sts3215_03a_v1.stl" if horn else "sts3215_03a_no_horn_v1.stl"
    mesh = trimesh.load(ROOT / "collision" / "inputs" / "arm_meshes" / name)
    mesh.apply_scale(1000.0)
    return mesh


@lru_cache(maxsize=None)
def source(name):
    """The unplaced shape of a stock link or servo."""
    if name in STOCK:
        return tessellate(params.donor(STOCK[name]))
    return servo_mesh(horn=name != "wrist_roll_case")


def _axis_rotations():
    for order in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            rotation = np.zeros((3, 3))
            rotation[range(3), order] = signs
            if np.linalg.det(rotation) > 0:
                yield rotation


def _step(points, matched, rotation, shift):
    """One rigid least-squares update moving points (already placed) onto their matches."""
    a, b = points.mean(0), matched.mean(0)
    u, _, vt = np.linalg.svd((points - a).T @ (matched - b))
    fix = np.diag([1, 1, np.sign(np.linalg.det(vt.T @ u.T))])
    turn = vt.T @ fix @ u.T
    return turn @ rotation, turn @ (shift - a) + b


@lru_cache(maxsize=None)
def register(name):
    """Fit a stock part to its display mesh.

    Returns the 4x4 transform from the part's own millimetres to the body frame, the RMS
    distance of the display vertices from the placed part, and the share within 0.25 mm.
    """
    part = source(name)
    display = trimesh.load(ROOT / "hollow" / "meshes" / f"{name}.stl")
    vertices = np.unique(np.asarray(display.vertices), axis=0)
    points = np.asarray(trimesh.sample.sample_surface(display, 1500, seed=0)[0])
    cloud = np.asarray(trimesh.sample.sample_surface(part, 150000, seed=0)[0])
    tree = cKDTree(cloud)
    # Starting rotations, display frame to part frame: the axis-aligned ones, and the same
    # set between the two shapes' principal axes for parts that sit at an angle.
    axes_display = np.linalg.eigh(np.cov(points.T))[1]
    axes_part = np.linalg.eigh(np.cov(cloud[::100].T))[1]
    guesses = list(_axis_rotations())
    guesses += [axes_part @ turn @ axes_display.T for turn in _axis_rotations()]
    guesses = [g if np.linalg.det(g) > 0 else -g for g in guesses]

    def refine(rotation, shift, rounds):
        for _ in range(rounds):
            moved = points @ rotation.T + shift
            distance, index = tree.query(moved, workers=-1)
            keep = distance <= np.quantile(distance, 0.9)
            rotation, shift = _step(moved[keep], cloud[index[keep]], rotation, shift)
        return tree.query(points @ rotation.T + shift, workers=-1)[0].mean(), rotation, shift

    # A few rounds from every start, then only the six best carry on.
    starts = [refine(g, cloud.mean(0) - g @ points.mean(0), 6) for g in guesses]
    starts = [refine(rotation, shift, 40) for _, rotation, shift in sorted(starts, key=lambda item: item[0])[:6]]
    _, rotation, shift = min(starts, key=lambda item: item[0])
    for _ in range(8):  # finish on the exact surface, with the display vertices
        moved = vertices @ rotation.T + shift
        nearest, _, _ = trimesh.proximity.closest_point(part, moved)
        rotation, shift = _step(moved, nearest, rotation, shift)
    distance = trimesh.proximity.closest_point(part, vertices @ rotation.T + shift)[1]
    transform = np.eye(4)
    transform[:3, :3] = rotation.T  # invert: part frame to display (body) frame
    transform[:3, 3] = -rotation.T @ shift
    return transform, float(np.sqrt(np.mean(distance**2))), float(np.mean(distance < 0.25))


def placement(name):
    """The stored transform from a part's own millimetres to its body frame."""
    stored = json.loads((HERE / "placements.json").read_text())
    return np.array(stored[name]["transform"])


def mesh(name):
    """A stock link or servo as a mesh in its body frame, millimetres."""
    placed = source(name).copy()
    placed.apply_transform(placement(name))
    return placed


def solid(name):
    """A stock link as a solid in its body frame, millimetres."""
    return moved(params.donor(STOCK[name]), placement(name))


def moved(shape, transform):
    """A CadQuery shape moved by a 4x4 rigid transform."""
    import cadquery as cq
    from scipy.spatial.transform import Rotation

    turn = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    angle = np.linalg.norm(turn)
    axis = turn / angle if angle > 1e-12 else np.array([0.0, 0.0, 1.0])
    return shape.moved(cq.Location(cq.Vector(*transform[:3, 3]), cq.Vector(*axis), np.degrees(angle)))


def main():
    stored = {}
    for name in (*STOCK, *SERVOS):
        transform, rms, share = register(name)
        stored[name] = {"transform": transform.round(9).tolist(), "rms_mm": round(rms, 4), "share_within_0.25_mm": round(share, 4)}
        print(f"{name}: RMS {rms:.3f} mm, {100 * share:.1f}% of display vertices within 0.25 mm")
    (HERE / "placements.json").write_text(json.dumps(stored, indent=1) + "\n")


if __name__ == "__main__":
    main()
