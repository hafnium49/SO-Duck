"""Independent central differences, contact closure, scaling and stance sensitivity."""
import csv
import numpy as np
from scipy.spatial.transform import Rotation
from scan_robot import (legs,OUT,LIMITS,sample,frames,fk,whole,evaluate)
from itertools import product
def check_jacobian(fs):
    q=sample(LIMITS,8,935106)
    eps=1e-6
    Jnum=np.empty((len(q),6,6))
    for j in range(6):
        dq=np.eye(6)[j]*eps
        plus,_=fk(q+dq,fs)
        minus,_=fk(q-dq,fs)
        Jnum[:,:3,j]=(plus[:,:3,3]-minus[:,:3,3])/(2*eps*.15)
        Jnum[:,3:,j]=Rotation.from_matrix(plus[:,:3,:3]@minus[:,:3,:3].transpose(0,2,1)).as_rotvec()/(2*eps)
    return np.max(np.abs(Jnum-fk(q,fs)[1]))
def native_fk(side,q):
    return legs.leg_fk(side,q)[0]
def check_whole(fs,sq):
    s=legs.stance('double',sq)
    n=s.base.shape[1];eps=1e-6
    q=np.deg2rad([8,-12,34,7,49,-15]);T,j=fk(q,fs)
    J=whole(s,T,j,.15);num=np.empty_like(J)
    ref=s.feet[0]
    err=0.
    for z in range(n+6):
        dq=np.zeros_like(q);dl=np.zeros((2,5))
        if z<n: dl=s.leg_rates[:,z].reshape(2,5)
        else:dq[z-n]=1.
        plus=[];minus=[]
        for mul,out in ((1,plus),(-1,minus)):
            now=sq+mul*eps*dl
            body=ref@np.linalg.inv(native_fk('left',now[0]))
            # Leg actuator columns are tangent to both fixed feet. Central first
            # variations of the right pose must vanish as well.
            out.extend((body@fk(q+mul*eps*dq,fs)[0],body@native_fk('right',now[1])))
        err=max(err,np.max(abs((plus[1]-minus[1])/(2*eps))))
        high=plus[0];low=minus[0]
        num[:,z]=np.r_[(high[:3,3]-low[:3,3])/.15,Rotation.from_matrix(high[:3,:3]@low[:3,:3].T).as_rotvec()]/(2*eps)
    return np.max(abs(num-J)),err
def do_checks():
    sq=legs.sagittal_q(-.4,1.25,yaw=np.deg2rad(10))
    for cfg in [(90,0,0,False),(90,0,0,True),(90,0,60,False),(90,0,60,True),(75,-30,30,False)]:
        fs=frames(*cfg)
        print('A,E,C,decoupled',cfg,'max fixed-torso central diff',check_jacobian(fs),'whole + right contact residual',check_whole(fs,sq))
    sample_q=sample(LIMITS,13,840701)
    cfgs=[(90,0,c,b) for c,b in product([0,30,60,90],[False,True])]
    poses={'parallel feet':legs.sagittal_q(-.4,1.25),
           'near straight':legs.sagittal_q(-.20,.50,yaw=np.deg2rad(10)),
           'reference':sq,
           'deeper':legs.sagittal_q(-.65,1.55,yaw=np.deg2rad(10))}
    stances={name:legs.stance('double',v) for name,v in poses.items()}
    rows=evaluate(cfgs,sample_q,stances)
    for r in rows:r['base_rank']=stances[r['mode']].base.shape[1]
    with (OUT/'stance_sensitivity.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
if __name__=='__main__':
    do_checks()
