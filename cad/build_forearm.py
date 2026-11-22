"""Moving-case forearm for the native Duck roll linkage. Print datum, millimetres."""
import cadquery as cq
import numpy as np
from pathlib import Path
from cadquery import Vector
from OCP.gp import gp_Trsf
from xml.etree import ElementTree
from scipy.spatial.transform import Rotation
OUT = Path(__file__).resolve().parent
assets = OUT
so = ElementTree.parse(OUT.parent / "collision/inputs/so101.xml")
root = np.eye(4)
root[:3, :3] = [[0,0,1],[0,1,0],[-1,0,0]]
root[:3, 3] = [78.95,0,129.06]

def element(node):
    result = np.eye(4)
    wxyz = np.fromstring(node.get("quat", "1 0 0 0"), sep=" ")
    result[:3, :3] = Rotation.from_quat(np.roll(wxyz, -1)).as_matrix()
    result[:3, 3] = 1000 * np.fromstring(node.get("pos", "0 0 0"), sep=" ")
    return result

axis_z=129.06

def box(x0,x1,y0,y1,z0,z1):
    return cq.Solid.makeBox(x1-x0,y1-y0,z1-z0,Vector(x0,y0,z0))

def cyl(r,x0,x1,y,z):
    return cq.Solid.makeCylinder(r,x1-x0,Vector(x0,y,z),Vector(1,0,0))

def place(filename,transform):
    shape=cq.importers.importStep(str(assets/filename)).val()
    t=gp_Trsf(); t.SetValues(*transform[:3,:].ravel().tolist())
    return shape.located(cq.Location(t))

def turn_quarter(shape):
    return shape.rotate((97.15,0,0),(97.15,0,1),90)

def loft_section(z,cx,dx,dy,r):
    w=cq.Workplane('XY',origin=(cx,0,z)).rect(dx,dy).wires().val()
    f=cq.Face.makeFromWires(w)
    return f.fillet2D(r,f.Vertices()).outerWire()

def clock_forward(shape):
    return shape.rotate((97.15,0,axis_z),(97.15,1,axis_z),90)

def build():
    pieces=[]; cuts=[]
    # A rigid ring takes the stock rear bearing. An outboard washer pair secures the outer race.
    pieces.extend([cyl(21.6,-28.6,-20.5,0,axis_z),box(-28.6,-20.5,-8.5,8.5,146,173.5),box(-28.6,101,-8.5,8.5,164.48,173.5)])
    cuts.extend([cyl(16.1,-28.7,-21.08,0,axis_z),cyl(14.8,-21.11,-20.4,0,axis_z)])
    # The rear inboard lower quadrant clears an axial assembly stroke before bearing insertion.
    cuts.append(box(-22.95,-20.4,-14.75,14.75,105.0,126.5))
    for y in [-19.,19.]:
        cuts.append(cyl(1.25,-28.7,-21.7,y,axis_z))
    # Stock case rear lugs and connector, captured on both faces. 0.05-mm CAD allowance.
    for sign in [-1,1]:
        y0,y1=sorted((sign*7.2,sign*14.2))
        pieces.extend([box(59.7,63.7,y0,y1,154.45,166.8),box(95.6,99.6,y0,y1,158.2,166.8)])
    pieces.append(box(59.7,101,-14.2,14.2,164.35,168.5))
    cuts.append(box(95.4,97.65,-9.45,9.45,157.81,165.85))
    for y in (-10.25,10.25):
        for x,z,drive in [(59.7,158.06,True),(95.6,161.81,False)]:
            cuts.append(cyl(1.25,x-.2,x+4.2,y,z))
            cb=x-.1 if drive else x+2.4
            cuts.append(cyl(2.25,cb,cb+1.7,y,z))
    for y in (-10.25,10.25):
        cuts.append(cyl(2.30,98.02,155.,y,161.81))
    # Leave an open channel between the saddle rails for the servo bus cable.
    cuts.append(box(73,91,-6.3,6.3,163.4,167.8))
    # The wrist-end features are cut from the actual stock SO-101 native solid.
    lower=so.find(".//body[@name='lower_arm']/geom[@mesh='under_arm_so101_v1']")
    fork=turn_quarter(place('under_arm_so101_v1.step',root@element(lower)))
    crown=fork.intersect(box(47,148,-63,63,183.7,273))
    crown=clock_forward(crown)
    # The offset neck lets the donor wrist sweep vertically with a horizontal forearm.
    # Plane YZ profiles make the forearm open forward as it drops below the saddle.
    def section(x,z,height,width,r):
        w=loft_section(0,0,height,width,r).rotate((0,0,0),(0,1,0),90)
        return w.translate((x,0,z))
    transition=cq.Solid.makeLoft([section(98.0,169.5,9.4,26,3),section(102,168.5,11,27,3),
        section(113,158.5,15,25,4),section(129,141,18,32,5),section(146,129.9,22,45,5),
        section(151.8,axis_z,24,52,5),section(153,axis_z,24,52,5)])
    pieces.extend([transition,crown]); shape=pieces[0].fuse(*pieces[1:]).clean().cut(*cuts).clean()
    assert shape.isValid() and len(shape.Solids())==1
    return shape

if __name__=='__main__':
    s=build()
    cq.exporters.export(s,str(OUT/'Case_mounted_forearm.step'))
    cq.exporters.export(s,str(OUT/'Case_mounted_forearm.stl'),tolerance=.05,angularTolerance=.11)
    print('volume',s.Volume(), 'faces',len(s.Faces()),flush=True)
