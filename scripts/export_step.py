"""Inspect or export the one-piece SO-Duck forearm STEP. Requires CadQuery."""
from pathlib import Path
import argparse


def main():
    p = argparse.ArgumentParser(description="Inspect or STL-export the public forearm STEP")
    p.add_argument("--source", default=None, help="Input STEP; defaults to cad/Case_mounted_forearm.step")
    p.add_argument("--output", default=None, help="Optional output STL path")
    p.add_argument("--inspect-only", action="store_true", help="No export")
    args = p.parse_args()
    import cadquery as cq
    root = Path(__file__).resolve().parents[1]
    src = Path(args.source) if args.source else root / "cad/Case_mounted_forearm.step"
    shape = cq.importers.importStep(str(src)).val()
    if not shape.isValid() or len(shape.Solids()) != 1:
        raise ValueError("Expected exactly one valid solid in forearm STEP")
    bounds = shape.BoundingBox()
    print(f"Solid volume: {shape.Volume()/1000:.3f} cm^3")
    print(f"Bounds: {bounds.xlen:.2f} x {bounds.ylen:.2f} x {bounds.zlen:.2f} mm")
    if not args.inspect_only:
        dst = Path(args.output) if args.output else root / "cad/Case_mounted_forearm.stl"
        cq.exporters.export(shape, str(dst), tolerance=0.05, angularTolerance=0.11)
        print(f"Exported {dst}")


if __name__ == "__main__":
    main()
