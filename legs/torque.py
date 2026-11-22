"""Quasi-static gravity torque for accepted forward grasps; joint order per arm_model.
Inputs measured SI inertial XML for donor links; added hardware masses are sensitivity cases.
Torso is held at the achieved articulated-leg pose when varying each upstream arm coordinate.
"""
import csv,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent.parent
sys.path[:0]=[str(root/'study'),str(root/'legs'),str(root/'search')]
import arm_model as am
import pose_test as pt
import balance as b
import legs
F=am.frames(90,0,90,True)
DEF=am.T(np.eye(3),[-.018,0,0])@am.rotate_about(am.turn('x',180),[0,0,.12906])@am.ROOT@am.T(np.diag([-1,1,-1]),[.0125,0,.0187])
CASE=legs.point(DEF,[-.000301462,0,.000127063])
def geometry(q,native):
    pts,tcp=b.mass_points(q,F)
    R=np.eye(4);arr=[]
    for h,angle in zip(F[:4],q):
        R=R@h@b.zj(angle);arr.append(R)
    print_to_upper=R@np.linalg.inv(am.ROOT)
    link=legs.point(print_to_upper,[.12400,-.00010,.14060])
    bearing=legs.point(print_to_upper,[-.0246,0,.12906])
    h=b.MASS@pts
    if native:
        # Keep native yaw assembly, replace head-mounted electronics with a rotating bare terminal case.
        h-=b.MASS[2:5]@pts[2:5]
        ycom=legs.point(arr[2],[.00412907,.00000395745,-.0222828])
        h+=.0918099*ycom+.055*legs.point(print_to_upper,CASE)
    return h+.085*link+.018*bearing,tcp
def main():
    rows=list(csv.DictReader((root/'study'/'poses_O90.csv').open()))
    cr={r['target']:r for r in csv.DictReader((root/'collision'/'collision_poses_O90.csv').open())}
    accepted=[r for r in rows if r['bank']=='front' and r['static_margin_5mm_ok']=='1' and cr[r['target']]['hit_count']=='0']
    header=['target','mass_model','payload_g']+['tau_'+j+'_Nm' for j in ['neck_pitch','head_pitch','yaw','terminal','wrist_flex','wrist_roll']]
    out=[]
    for native in (False,True):
        A={0.:[],.05:[]}
        for r in accepted:
            q=np.array([float(v) for k,v in r.items() if k.endswith('_rad') and k!='stance_yaw_rad'])
            s=legs.stance(q=q[:10].reshape(2,5));wb=pt.ANCHOR[0]@np.linalg.inv(s.feet[0]);up=wb[2,:3]
            eps=1.e-5
            arr=[]
            for delta in eps*np.eye(6):
                h1,t1=geometry(q[-6:]+delta,native);h2,t2=geometry(q[-6:]-delta,native)
                arr.append([9.80665*up@(h1-h2)/(2*eps),9.80665*up@(t1-t2)/(2*eps)])
            arr=np.array(arr)
            for load in A:
                tau=arr[:,0]+arr[:,1]*load;A[load].append(tau);out.append([r['target'],'native moving terminal' if native else 'nominal approximate lump',int(load*1000),*tau])
        for load,B in A.items():
            vals=abs(np.asarray(B)); order=np.unravel_index(np.argmax(vals),vals.shape)
            print('native',native,'load',load,'N',len(vals),'P95',np.round(np.quantile(vals,.95,axis=0),3),'max',np.round(vals.max(axis=0),3),'P95 max_all_wrt_pose',np.quantile(vals.max(axis=1),.95),'worst target,joint',accepted[order[0]]['target'],order[1])
    with (root/'legs'/'gravity_O90.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(header);w.writerows(out)
if __name__=='__main__':main()
