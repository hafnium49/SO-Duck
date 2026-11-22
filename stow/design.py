"""Centered stow variant. CAD coordinates are the Duck trunk frame in millimetres."""
from pathlib import Path
import math

import cadquery as cq
import numpy as np

OUT = Path(__file__).resolve().parent
ROOF = 107.8909
HOLE_X = (-14.0, 18.0)
RAIL_Y = 29.5
INDEX_DEG = -360 / 25
PARK = np.array([60., -74., 0., 0., 90., 0.])  # effective joint angles, degrees
# Underside of the original wrist fork at PARK (confirmed against the installed STEP).
PAD_Z82 = 115.29848
PAD_SLOPE = math.tan(math.radians(PARK[0] + PARK[1]))
PAD_THICKNESS = 1.5  # vertical, including bond line
WORK_ELBOW = (-45., 45.)


def elbow_motor(effective):
    """Degrees relative to the pre-indexing motor datum, with donor signs applied."""
    return effective - INDEX_DEG


def prism_xz(profile, y0, y1):
    vertices = [cq.Vector(x, y0, z) for x, z in profile]
    wire = cq.Wire.makePolygon(vertices, close=True)
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(0, y1 - y0, 0))


def pad_height(x):
    return PAD_Z82 + (x - 82.) * PAD_SLOPE


def ramp(x0, x1, y0, y1, bottom, top):
    return prism_xz([(x0, bottom(x0)), (x1, bottom(x1)),
                     (x1, top(x1)), (x0, top(x0))], y0, y1)


def bore(x, y, z, diameter, height):
    return cq.Solid.makeCylinder(diameter / 2, height, cq.Vector(x, y, z))


def parts():
    # Bridge the two side rails beyond the nose so the neck opening stays clear.
    # Keep the center of the bridge below the fork outside the two pad lands.
    rail_profile = [(-20, ROOF), (23, ROOF), (26, ROOF+1), (47.5, ROOF+1), (55, 100.0), (85, 100.0),
                    (85, 111.4), (73, 115.8), (-20, 115.8)]
    rails, backers, pads = [], [], []
    for side in (-1, 1):
        y = side * RAIL_Y
        rail = prism_xz(rail_profile, y - 3.7, y + 3.7)
        # The roof has a 45° inner fillet at its outboard edge.
        yz = [(-3.7, -6), (3.7, -6), (3.7, -4.3), (2.4, -3), (-3.7, -3)]
        contour = [cq.Vector(-20, y + side*dy, ROOF+dz) for dy,dz in yz]
        backer = cq.Solid.extrudeLinear(cq.Wire.makePolygon(contour, close=True),
                                        [], cq.Vector(43, 0, 0))
        for x in HOLE_X:
            rail = rail.cut(bore(x, y, ROOF - 1, 3.3, 11))
            # Recess the screw head under the rail top. Four millimetres of rail remain under the head.
            rail = rail.cut(bore(x, y, ROOF + 4.0, 6.2, 6))
            backer = backer.cut(bore(x, y, ROOF - 7, 2.5, 5))  # M3 tap drill
        rails.append(rail)
        backers.append(backer)
        y0,y1 = sorted((side * 7., side * 17.))
        pads.append(ramp(79., 85., y0, y1, lambda x: pad_height(x)-PAD_THICKNESS,
                         pad_height))
    under_pad = lambda x: pad_height(x) - PAD_THICKNESS
    bridge = ramp(79., 85., -RAIL_Y, RAIL_Y, lambda x: 100., under_pad)
    carrier = rails[0].fuse(rails[1], bridge).clean()
    return {'cradle': carrier, 'pad_left': pads[1], 'pad_right': pads[0],
            'backer_left': backers[1], 'backer_right': backers[0]}


def screws():
    # Conservative flat-top envelopes for four M3 x 10 button-head screws.
    solids = []
    for y in (-RAIL_Y, RAIL_Y):
        for x in HOLE_X:
            seat = ROOF + 4.
            solids.append(bore(x, y, seat - 10, 3., 10).fuse(bore(x, y, seat, 5.7, 1.65)))
    return cq.Compound.makeCompound(solids)


def drilled_roof():
    """Reconstruct the donor faceted solid and drill four through-holes; trunk mm."""
    from OCP.StlAPI import StlAPI_Reader
    from OCP.TopoDS import TopoDS_Shape
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
    from xml.etree import ElementTree

    inputs = OUT.parent / 'collision' / 'inputs'
    mesh = TopoDS_Shape()
    if not StlAPI_Reader().Read(mesh, str(inputs/'duck_meshes'/'body_middle_top.stl')):
        raise IOError('Restore the donor meshes with scripts/restore_assets.py')
    sew = BRepBuilderAPI_Sewing(1e-7)
    sew.Add(mesh)
    sew.Perform()
    shells = cq.Shape.cast(sew.SewedShape()).Shells()
    if len(shells) != 1 or not shells[0].Closed():
        raise ValueError('Expected one closed roof shell')
    roof = cq.Solid.makeSolid(shells[0]).scale(1000)
    roof_geom = ElementTree.parse(inputs/'robot_motors.xml').find(".//geom[@mesh='body_middle_top']")
    if roof_geom.get('quat','1 0 0 0') != '1 0 0 0':
        raise ValueError('Unexpected donor roof orientation')
    shift = 1000*np.fromstring(roof_geom.get('pos'), sep=' ')
    roof = roof.translate(tuple(shift)).clean()
    if roof.Volume() < 0:
        roof = cq.Solid(roof.wrapped.Reversed())
    before = roof.Volume()
    for x in HOLE_X:
        for y in (-RAIL_Y, RAIL_Y):
            roof = roof.cut(bore(x,y,ROOF-7,3.3,10))
    removed = before - roof.Volume()
    if not (roof.isValid() and len(roof.Solids()) == 1 and
            abs(removed - 4*math.pi*1.65**2*3) < .02):
        raise ValueError(f'Unexpected 3 mm deck or through-hole geometry: {removed}')
    return roof


def export(directory=OUT):
    directory.mkdir(parents=True, exist_ok=True)
    shapes = parts()
    assembly = cq.Assembly(name='Centered_stow_cradle')
    for name, solid in shapes.items():
        if not solid.isValid() or len(solid.Solids()) != 1:
            raise ValueError(f'{name} is not a single valid solid')
        assembly.add(solid, name=name)
    assembly.add(screws(), name='installed_screw_envelopes')
    assembly.export(str(directory / 'Cradle_assembly.step'))
    # Place the broad top faces of both rails on the bed.
    for name in ('cradle','pad_left'):
        solid = shapes[name]
        if name == 'cradle':
            solid = solid.rotate((0,0,0), (1,0,0), 180)
        else:
            # The interchangeable pad is a parallel-face wedge in trunk coordinates.
            solid = solid.rotate((0,0,0),(0,1,0), math.degrees(math.atan(PAD_SLOPE)))
        b = solid.BoundingBox()
        cq.exporters.export(solid.translate((-b.xmin,-b.ymin,-b.zmin)),
                            str(directory / ('cradle.stl' if name=='cradle' else 'contact_pad.stl')), tolerance=.04, angularTolerance=.1)
    roof = drilled_roof()
    cq.exporters.export(roof, str(directory/'roof_drilled.stl'), tolerance=.03, angularTolerance=.1)
    return shapes


if __name__ == '__main__':
    export()
