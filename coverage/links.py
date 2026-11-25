"""Horn and case interfaces in the native STS3215 driven-horn datum (mm)."""

import cadquery as cq
import numpy as np
from routing import route
from OCP.gp import gp_Trsf


def transform(s, rotation, translation):
    t = gp_Trsf()
    t.SetValues(*np.column_stack([rotation.as_matrix(), translation]).ravel())
    return s.moved(cq.Location(t))


def box(x, y, z, dx, dy, dz):
    return cq.Solid.makeBox(dx, dy, dz, cq.Vector(x, y, z))


def cylinder(r, z, h, x=0, y=0):
    return cq.Solid.makeCylinder(r, h, cq.Vector(x, y, z))


def horn():
    # Hole coordinates are the four native horn tapped-hole centers.
    flange = cylinder(11.5, -6, 5.94)
    boss = cylinder(7.6, -15, 10)
    s = flange.fuse(boss)
    for x in (-4.95, 4.95):
        for y in (-4.95, 4.95):
            s = s.cut(cylinder(1.6, -16, 17, x, y))
    return s.cut(cylinder(3, -16, 17))


def clamp():
    # Dual-sided capture on the rear pair of lugs of each cover.
    a = box(25, -14, -3.4, 15, 28, 5.03)
    b = box(30.6, -14, 35.78, 9.4, 28, 4.95)
    wall = box(35.45, -14, -3.4, 4.55, 28, 44.13)
    s = a.fuse(b, wall)
    for x, z in ((29.0, -3.5), (32.8, 35.65)):
        for y in (-10.25, 10.25):
            s = s.cut(cylinder(1.25, z, 5.5, x, y))
    return s


def beam(a, b):
    d = b - a
    return cq.Solid.makeCylinder(
        7.4,
        np.linalg.norm(d) + 0.2,
        cq.Vector(*(a - 0.1 * d / np.linalg.norm(d))),
        cq.Vector(*d),
    )


def link(start, axis, end, housing, base=False):
    # First pass through each leg outside the associated servo, then bridge the two.
    a = start + axis.apply([0, 0, -13])
    b = end + housing.apply([38.5, 0, 18.7])
    a1 = start + axis.apply([0, 0, -24])
    b1 = end + housing.apply([49, 0, 18.7])
    if base:
        a = start
        a1 = a + [0, 0, 10]
        basepart = box(a[0] - 16, a[1] - 15, a[2], 32, 30, 3.5)
        for x in [-11, 11]:
            for y in [-10, 10]:
                basepart = basepart.cut(cylinder(1.65, a[2] - 1, 6, a[0] + x, a[1] + y))
    else:
        basepart = transform(horn(), axis, start)
    path = route(a1, b1, start, axis, end, housing, base)
    points = np.vstack([a, path, b])
    pieces = [basepart, transform(clamp(), housing, end)]
    for left, right in zip(points, points[1:]):
        pieces.append(beam(left, right))
    for p in points[1:-1]:
        pieces.append(cq.Solid.makeSphere(7.5, cq.Vector(*p), angleDegrees1=-90))
    shape = pieces[0]
    for piece in pieces[1:]:
        shape = shape.fuse(piece, tol=1e-6)
    for left, right in zip(points, points[1:]):
        middle = right - left
        span = np.linalg.norm(middle)
        if span > 35:
            shape = shape.cut(
                cq.Solid.makeCylinder(
                    5.0,
                    span - 18,
                    cq.Vector(*(left + 9 * middle / span)),
                    cq.Vector(*middle),
                )
            )
    shape = shape.clean()
    if not shape.isValid() or len(shape.Solids()) != 1:
        raise ValueError(
            f"Mount {end}: solids {len(shape.Solids())}, volume {shape.Volume()}, valid {shape.isValid()}"
        )
    return shape
