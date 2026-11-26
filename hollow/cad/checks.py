"""Geometry checks shared by the part tests. Lengths are millimetres."""

from functools import lru_cache

import cadquery as cq
import numpy as np
import trimesh

import params
import stock
from stock import tessellate  # noqa: F401  (re-exported)

# The stock SO-101 holder sits 0.40 mm inside the servo mesh along one edge of the cover.
# A holder may go that deep before it counts as cutting into the servo.
SERVO_ALLOWANCE_MM = 0.45
RAY_BATCH = 400  # points per ray query, to bound trimesh's memory


def min_wall_mm(solid, samples=20000):
    """Wall thickness that 99% of the surface exceeds.

    From sample points on the surface a ray runs inward along the normal. Its length to
    the next surface is the thickness of the material there.
    """
    mesh = tessellate(solid)
    points, faces = trimesh.sample.sample_surface(mesh, samples, seed=0)
    inward = -mesh.face_normals[faces]
    origins = np.asarray(points) + 1e-3 * inward
    depths = []
    for start in range(0, samples, 10 * RAY_BATCH):
        batch = slice(start, start + 10 * RAY_BATCH)
        hits, rays, _ = mesh.ray.intersects_location(origins[batch], inward[batch], multiple_hits=False)
        depths.append(np.linalg.norm(hits - origins[batch][rays], axis=1))
    return float(np.percentile(np.concatenate(depths), 1))


# Collision objects are built once per mesh and kept: a sweep asks about the same two
# meshes many times.
_OBJECTS = {}


def _collision_object(mesh, skip):
    import fcl

    key = (id(mesh), id(skip))
    if key in _OBJECTS:
        return _OBJECTS[key][0]

    vertices = np.asarray(mesh.vertices, dtype=float)
    faces = np.asarray(mesh.faces, dtype=np.int64)
    if skip is not None:
        faces = faces[~skip(vertices[faces].mean(axis=1))]
        if not len(faces):
            raise ValueError("skip removes every triangle of a mesh")
        used, faces = np.unique(faces, return_inverse=True)  # the wrapper needs every vertex used
        vertices, faces = vertices[used], faces.reshape(-1, 3)
    shape = fcl.BVHModel()
    shape.beginModel(len(vertices), len(faces))
    shape.addSubModel(np.ascontiguousarray(vertices), np.ascontiguousarray(faces, dtype=np.int64))
    shape.endModel()
    _OBJECTS[key] = (fcl.CollisionObject(shape, fcl.Transform()), shape, mesh, skip)
    return _OBJECTS[key][0]


def clearance_mm(a, b, skip=None, move_b=None):
    """Smallest distance between two meshes, or 0 if they intersect.

    skip(points) marks triangles, by their centres in the mesh's own frame, that take no
    part: the faces where two parts are meant to touch. move_b is a 4x4 transform applied
    to the second mesh, for sweeping it through a joint's range.
    """
    import fcl

    first, second = _collision_object(a, skip), _collision_object(b, skip)
    move = np.eye(4) if move_b is None else np.asarray(move_b, dtype=float)
    second.setTransform(fcl.Transform(np.ascontiguousarray(move[:3, :3]), np.ascontiguousarray(move[:3, 3])))
    if fcl.collide(first, second, fcl.CollisionRequest(), fcl.CollisionResult()):
        return 0.0
    return float(fcl.distance(first, second, fcl.DistanceRequest(), fcl.DistanceResult()))


def _servo_frame(servo, shift, placement):
    frame = np.array(stock.placement(servo) if placement is None else placement, dtype=float)
    move = np.eye(4)
    move[:3, 3] = shift
    return frame @ move


@lru_cache(maxsize=None)
def _servo_points(horn):
    servo = stock.servo_mesh(horn)
    return np.asarray(trimesh.sample.sample_surface(servo, 6000, seed=0)[0])


def _depth_inside(mesh, points):
    """How far the deepest of the points lies inside the mesh; 0 if none is inside."""
    low, high = mesh.bounds
    points = points[np.all((points > low) & (points < high), axis=1)]
    if not len(points):
        return 0.0
    # In pieces: trimesh gathers every candidate triangle of every ray at once, which
    # for a large part takes gigabytes.
    inside = np.concatenate([mesh.contains(points[start : start + RAY_BATCH]) for start in range(0, len(points), RAY_BATCH)])
    points = points[inside]
    if not len(points):
        return 0.0
    return float(trimesh.proximity.closest_point(mesh, points)[1].max())


def servo_penetration_mm(part, servo, shift=(0.0, 0.0, 0.0), placement=None):
    """How deep a part's mesh and a servo cut into each other, in millimetres.

    The servo sits at its stored place in the body frame, or at placement, and is moved
    by shift along its own axes: that is how an insertion path is tested. Both ways are
    measured: the servo's surface inside the part, and the part's vertices inside the servo.
    """
    horn = servo != "wrist_roll_case"
    frame = _servo_frame(servo, shift, placement)
    surface = trimesh.transform_points(_servo_points(horn), frame)
    corners = trimesh.transform_points(np.asarray(part.vertices), np.linalg.inv(frame))
    return max(_depth_inside(part, surface), _depth_inside(stock.servo_mesh(horn), corners))


def connector_blocked_mm3(part, servo, placement=None):
    """Volume of a servo's two cable-plug spaces that the part (a solid) fills."""
    frame = _servo_frame(servo, (0.0, 0.0, 0.0), placement)
    blocked = 0.0
    for low, high in stock.SERVO_CONNECTORS:
        space = cq.Solid.makeBox(*np.subtract(high, low), cq.Vector(*low))
        blocked += sum(solid.Volume() for solid in part.intersect(stock.moved(space, frame)).Solids())
    return float(blocked)


def hole_is_open(part, centre, axis, radius=1.4, depth=8.0):
    """Whether a bore of the given radius runs through the part at centre along axis.

    The bore itself must be empty, and the part must have material around it.
    """
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    start = cq.Vector(*(np.asarray(centre, dtype=float) - axis * depth / 2))
    direction = cq.Vector(*axis)
    bore = cq.Solid.makeCylinder(radius, depth, start, direction)
    collar = cq.Solid.makeCylinder(radius + 2.0, depth, start, direction).cut(cq.Solid.makeCylinder(radius + 0.4, depth, start, direction))
    volume = lambda shape: sum(solid.Volume() for solid in shape.Solids())
    return volume(part.intersect(bore)) < 0.2 and volume(part.intersect(collar)) > 5.0


def fits_bed(solid):
    """Whether the part lies on the bed in some axis-aligned orientation."""
    box = solid.BoundingBox()
    return sorted([box.xlen, box.ylen, box.zlen])[1] <= params.BED_MM
