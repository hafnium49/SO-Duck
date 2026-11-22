"""Grounded duck biped Jacobian. All vectors in trunk basis, metre, radian.
The joint-rate norm is Euclidean across all enabled actuators; the floating
base is an unactuated, dependent variable. Condenses perfectly rigid flat feet.
"""
from pathlib import Path
from dataclasses import dataclass
from functools import lru_cache
import xml.etree.ElementTree as ET
import numpy as np
from scipy.linalg import block_diag, null_space
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
ROOT=ET.parse(HERE/'duck.xml').getroot()
TRUNK=ROOT.find('.//body[@name="trunk_assembly"]')
NAMES=('hip_yaw','hip_roll','hip_pitch','knee','ankle')
def vec(text):
    return np.fromstring(text,sep=' ')
def rest(el):
    t=np.eye(4)
    t[:3,3]=vec(el.get('pos','0 0 0'))
    quat=vec(el.get('quat','1 0 0 0'))
    t[:3,:3]=Rotation.from_quat(np.roll(quat,-1)).as_matrix()
    return t

def point(T,p):
    return np.asarray(p)@T[:3,:3].T+T[:3,3]

def at_point(p):
    v=np.broadcast_to(np.eye(6),np.shape(p)[:-1]+(6,6)).copy()
    # column j is e_j cross p
    v[...,:3,3:]=np.cross(np.eye(3),np.asarray(p)[...,None,:]).swapaxes(-1,-2)
    return v

def read_sole():
    records=np.frombuffer((HERE/'sole.stl').read_bytes(),offset=84,dtype=[('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')])
    vertices=np.unique(records['v'].reshape(-1,3),axis=0)
    return vertices[vertices[:,2] < vertices[:,2].min()+0.00010]  # plane within 0.1 mm
SOLE=read_sole()
def link_specs(side):
    return [ROOT.find(f'.//joint[@name="{side}_{n}"]/..') for n in NAMES]
SIDES=('left','right')
@lru_cache
def specs(side):
    ls=link_specs(side)
    geom=ls[-1].find('geom[@class="collision"]')
    return ([rest(link) for link in ls],np.array([float(link.find('inertial').get('mass')) for link in ls]),np.array([vec(link.find('inertial').get('pos')) for link in ls]),rest(geom),rest(ls[-1].find('site')))
def leg_fk(side,q):
    steps,mass,com,geom,site=specs(side)
    frames=[]; trans=np.eye(4)
    for step,angle in zip(steps,q):
        trans=trans@step
        c,s=np.cos(angle),np.sin(angle)
        trans[:3,:3]=trans[:3,:3]@np.array([[c,-s,0],[s,c,0],[0,0,1]])
        frames.append(trans.copy())
    frames=np.array(frames)
    origin=frames[:,:3,3]
    axis=frames[:,:3,2]
    foot=trans@site
    J=np.vstack([np.cross(axis,foot[:3,3]-origin).T,axis.T])
    centers=origin+(frames[:,:3,:3]@com[:,:,None])[:,:,0]
    return foot,J,point(trans@geom,SOLE),mass@centers
def sagittal_q(hip=-.65,knee=1.33,ankle=None,yaw=0.):
    """Zero hip rolls, symmetric opposite signed hip pitch, both knees/ankles positive/negative.
    For level trunk & soles set ankle=-hip-knee; yaw angles positive toes-out.
    """
    if ankle is None: ankle=-hip-knee
    return np.array([[yaw,0,hip,knee,ankle],[-yaw,0,-hip,knee,ankle]])
@dataclass
class Stance:
    base: np.ndarray
    leg_rates: np.ndarray
    feet: np.ndarray
    sole_points: np.ndarray
    rest_mass: float
    rest_moment: np.ndarray
    residual: float
    def tip_jac(self,p):
        return at_point(p)@self.base
    def margin(self, com, up=(0,0,1)):
        """Horizontal signed distance to support hull; + inside. Trunk-frame upward normal."""
        up=np.array(up,dtype=float); up/=np.linalg.norm(up)
        basis=null_space(up[None,:])
        hull=ConvexHull(self.sole_points@basis)
        eq=hull.equations
        return -np.max(np.asarray(com)@basis@eq[:,:2].T+eq[:,2],axis=-1)

def stance(mode='double',q=None):
    q=sagittal_q() if q is None else np.asarray(q).reshape(2,5)
    ids={'double':[0,1],'left':[0],'right':[1]}[mode]
    fks=[leg_fk(s,theta) for s,theta in zip(SIDES,q)]
    Fs=np.asarray([x[0] for x in fks])
    A=[at_point(Fs[i,:3,3]) for i in ids]
    J=[fks[i][1] for i in ids]
    # Each leg alone fixes base twist = candidates[i] qdot_i.
    candidates=[-np.linalg.solve(a,j) for a,j in zip(A,J)]
    if mode=='double':
        comparison=np.hstack([candidates[0],-candidates[1]])
        comparison[:3]/=0.15
        rates=null_space(comparison,rcond=1.e-8)
        B=candidates[0]@rates[:5]
    else:
        rates=np.eye(5)
        B=candidates[0] 
    err=np.max(np.abs(np.concatenate(A)@B+block_diag(*J)@rates))
    m0=float(TRUNK.find('inertial').get('mass'))
    h0=m0*vec(TRUNK.find('inertial').get('pos'))
    return Stance(B,rates,Fs,np.concatenate([fks[i][2] for i in ids]),
                  m0+sum(sum(specs(s)[1]) for s in SIDES),h0+sum(f[-1] for f in fks),err)

def sagittal_base(hip=-.65,knee=1.33,ankle=None,reference_q=None):
    """Returns trunk T in common world (= reference trunk frame) with anchored feet.
    Angles are left-hip_pitch, and bilateral knee/ankle, in rad. At default,
    ankle is chosen to keep the trunk level; independent ankle permits torso pitch.
    Perfectly symmetric posture anchored at both feet, from parallel-foot neutral.
    """
    q=sagittal_q(hip,knee,ankle)
    reference_q=sagittal_q() if reference_q is None else np.asarray(reference_q)
    f_ref=leg_fk('left',reference_q[0])[0]
    f_now=leg_fk('left',q[0])[0]
    return f_ref@np.linalg.inv(f_now),q

if __name__=='__main__':
    for mode in ('double','left','right'):
        S=stance(mode)
        print(mode,'base',S.base.shape,np.linalg.svd(S.base,compute_uv=False),'residual',S.residual)
        print('sole min max',1000*S.sole_points.min(0),1000*S.sole_points.max(0),'bare COM',S.rest_mass,S.rest_moment/S.rest_mass,'margin',S.margin(S.rest_moment/S.rest_mass))
    for h,k,a in [(-.4,1.1,-.7),(-.5,1.3,-1.)]:
        T,q=sagittal_base(h,k,a)
        print('T',T.round(5),'left/right contact pose error',np.max(np.abs(T@stance(q=q).feet-stance().feet)))
    print('foot orientations',stance().feet)
