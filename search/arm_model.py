"""Fixed-torso Open Duck Mini v2 / SO-101 hybrid in SI units.

Extracts below are the translation and quaternion attributes of the source MJCF
bodies and sites, rounded only where noise is smaller than 1e-12. All six
revolute coordinates are local +Z. The gripperframe tool point has a fixed jaw.
A/E/C are degrees; q is radians. The six joints are neck_pitch, head_pitch,
head_yaw, terminal (formerly head_roll), wrist_flex, wrist_roll.
"""
import numpy as np
from scipy.spatial.transform import Rotation

LIMITS = np.deg2rad(np.array([[-20.,65.],[-45.,45.],[-160.,160.],[-30.,30.],
                              [-95.,95.],[-157.,163.]]))
LIMITS[-1] = [-2.7438472969992493, 2.841206309382605]
def T(rotation, translation):
    t = np.eye(4)
    t[:3,:3] = rotation
    t[:3,3] = translation
    return t
def body(pos, wxyz):
    return T(Rotation.from_quat(np.roll(wxyz,-1)).as_matrix(),pos)
def turn(axis, angle):
    return Rotation.from_euler(axis,angle,degrees=True).as_matrix()
def rotate_about(rotation, point):
    return T(rotation,point-rotation@point)
# MJCF (torso coordinates, metres). Printed Duck CAD datum is linked via the
# visual transform of head_pitch_to_yaw in its identically named body.
NECK = body([.001,.01915,.0900009],[.707107,.707107,0,0])
PITCH = body([0,.066,0],[1,0,0,0])
YAW = body([0,.057,.01905],[.707107,-.707107,0,0])
PITCH_VISUAL = body([-.020,-.09111,.01905],[.707107,-.707107,0,0])
TRUNK_FROM_PRINT = NECK @ PITCH @ PITCH_VISUAL
PRINT_FROM_TRUNK = np.linalg.inv(TRUNK_FROM_PRINT)
# SO-101 MJCF. The separate direct-mount carrier datum is in print coordinates.
ROOT = T(np.array([[0,0,1],[0,1,0],[-1,0,0]]),[.07895,0,.12906])
MOUNT_CENTER = np.array([.09715,0,.12906])
WRIST = body([-.1349,.0052,0],[.707107,0,0,-.707107])
ROLL = body([0,-.0611,.0181],[.0172091,-.0172091,.706897,.706897])
TCP = body([-.0079,-.000218121,-.0981274],[.707107,0,.707107,0])
def frames(azimuth=90.,elevation=0.,clock=0.,two_part=False,base='trunk'):
    """Return six local joint transforms and final fixed tool transform.

    +X forward, +Y left, +Z up in the print datum. Rotations change a single
    carrier mounting pose. At A=90,E=0 the wrist flex sweeps sagittally at
    zero torso/neck angles. Clock changes forearm angle at terminal zero.
    two_part selects the axis-decoupled kinematics: a fixed +X terminal
    axis and an independently tilted fork. This historical parameter name
    does not imply a physical part count. A stock-carrier case-mounted link
    reproduces the same screw axes and wrist frame with one custom part.
    """
    h = rotate_about(turn('z',azimuth)@turn('y',-elevation)@turn('x',clock),MOUNT_CENTER)
    terminal = ROOT if two_part else h@ROOT
    wrist = h@ROOT@WRIST
    pivot = TRUNK_FROM_PRINT@terminal
    first = NECK if base=='trunk' else PRINT_FROM_TRUNK@NECK
    return np.array([first,PITCH,YAW,np.linalg.inv(NECK@PITCH@YAW)@pivot,
                     np.linalg.inv(terminal)@wrist,ROLL,TCP])
def fk(q, fs, length=.15):
    """Batched tool transforms and 6x6 spatial Jacobians scaled [v/L; omega]."""
    q=np.asarray(q,dtype=float)
    if q.shape[-1]!=6: raise ValueError('Six radians expected per joint vector')
    r=np.broadcast_to(np.eye(3),q.shape[:-1]+(3,3))
    p=np.zeros(q.shape[:-1]+(3,))
    points=[]; axes=[]
    for index,f in enumerate(fs[:6]):
        p=p+(r@f[:3,3,None])[...,0]
        r=r@f[:3,:3]
        points.append(p)
        axes.append(r[..., :,2])
        cs=np.cos(q[...,index]); sn=np.sin(q[...,index])
        rz=np.zeros(r.shape); rz[...,0,0]=cs; rz[...,1,1]=cs
        rz[...,1,0]=sn; rz[...,0,1]=-sn; rz[...,2,2]=1
        r=r@rz
    p=p+(r@fs[6,:3,3,None])[...,0]
    r=r@fs[6,:3,:3]
    jac=np.concatenate((np.cross(np.stack(axes,axis=-2), p[...,None,:]-np.stack(points,axis=-2))/length,
                        np.stack(axes,axis=-2)),axis=-1).swapaxes(-1,-2)
    pose=np.zeros(q.shape[:-1]+(4,4))
    pose[...,:3,:3]=r
    pose[...,:3,3]=p
    pose[...,3,3]=1
    return pose,jac
