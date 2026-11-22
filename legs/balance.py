"""Hybrid static gravity-load model. This is an unmeasured mass estimate:
stock Duck trunk/legs/first two neck bodies, SO lower_arm/wrist/gripper/closed
jaw inertias. Robots' two yaw/terminal cases weigh 55 g apiece, with an 18 g
adapter at the midpoint of their centers. Jaw at XML zero pose.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
import legs
def homo(r,p):
    T=np.eye(4)
    T[:3,:3]=r
    T[:3,3]=p
    return T
def zj(a):
    return homo(R.from_rotvec([0,0,a]).as_matrix(),[0,0,0])
MASS=np.array([.06618,.0169378,.055,.055,.018,.104,.079,.087,.012])
def mass_points(q,frames):
    """SI frames as in arm_model.frames; returns nine mass coordinates and TCP. Single q only."""
    Ts=[]; parents=[]; T=np.eye(4)
    for i,t in enumerate(frames[:6]):
        T=T@t
        parents.append(T)
        T=T@zj(q[i]); Ts.append(T)
    wrist=parents[4]
    original_wrist=homo(R.from_euler('z',-90,degrees=True).as_matrix(),[-.1349,.0052,0])
    lower=wrist@np.linalg.inv(original_wrist)
    jaw=Ts[5]@homo(R.from_euler('x',90,degrees=True).as_matrix(),[.0202,.0188,-.0234])
    # Native yaw/terminal case centers, using geometric centroid of donor servo housing as proxy.
    yaw=legs.point(Ts[2],[.01280146, 0,-.01847294])
    terminal=legs.point(parents[3],[.01280146,0,.01857294])
    pts=np.array([
        legs.point(Ts[0],[-.00000563137,.0492968,.0181786]),
        legs.point(Ts[1],[-.00766247,.026015,.0186681]),
        yaw,terminal,(yaw+terminal)/2,
        legs.point(lower,[-.0980701,.00324376,.0182831]),
        legs.point(Ts[4],[-.000103312,-.0386143,.0281156]),
        legs.point(Ts[5],[.000213627,.000245138,-.025187]),
        legs.point(jaw,[-.00157495,-.0300244,.0192755])])
    return pts,(Ts[-1]@frames[6])[:3,3]
def arm_mass(q,frames,payload=0.):
    pts,tcp=mass_points(q,frames)
    return np.sum(MASS)+payload, MASS@pts+payload*tcp
def hybrid_com(S,q,frames,payload=0.):
    m,h=arm_mass(q,frames,payload)
    return (S.rest_moment+h)/(S.rest_mass+m)
