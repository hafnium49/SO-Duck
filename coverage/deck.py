"""Four mounting holes in the original three-millimetre roof panel."""

import math
import cadquery as cq
import numpy as np


def drilled_roof(directory):
    """Reconstruct the donor faceted solid and drill four through-holes; trunk mm."""
    from OCP.StlAPI import StlAPI_Reader
    from OCP.TopoDS import TopoDS_Shape
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
    from xml.etree import ElementTree

    inputs = directory.parent / "collision" / "inputs"
    mesh = TopoDS_Shape()
    if not StlAPI_Reader().Read(
        mesh, str(inputs / "duck_meshes" / "body_middle_top.stl")
    ):
        raise IOError("Restore the donor meshes with scripts/restore_assets.py")
    sew = BRepBuilderAPI_Sewing(1e-7)
    sew.Add(mesh)
    sew.Perform()
    shells = cq.Shape.cast(sew.SewedShape()).Shells()
    if len(shells) != 1 or not shells[0].Closed():
        raise ValueError("Expected one closed roof shell")
    roof = cq.Solid.makeSolid(shells[0]).scale(1000)
    roof_geom = ElementTree.parse(inputs / "robot_motors.xml").find(
        ".//geom[@mesh='body_middle_top']"
    )
    if roof_geom.get("quat", "1 0 0 0") != "1 0 0 0":
        raise ValueError("Unexpected donor roof orientation")
    shift = 1000 * np.fromstring(roof_geom.get("pos"), sep=" ")
    roof = roof.translate(tuple(shift)).clean()
    if roof.Volume() < 0:
        roof = cq.Solid(roof.wrapped.Reversed())
    before = roof.Volume()
    for x in (-103.0, -81.0):
        for y in (-10.0, 10.0):
            roof = roof.cut(cq.Solid.makeCylinder(1.65, 10, cq.Vector(x, y, 100.8909)))
    removed = before - roof.Volume()
    if not (
        roof.isValid()
        and len(roof.Solids()) == 1
        and abs(removed - 4 * math.pi * 1.65**2 * 3) < 0.02
    ):
        raise ValueError(f"Unexpected 3 mm deck or through-hole geometry: {removed}")
    return roof.translate(tuple(-shift))
