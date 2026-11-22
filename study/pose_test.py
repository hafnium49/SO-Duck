"""Compare fixed world grasp targets with both feet grounded by constrained IK.
SI units. Run from package root with Python, NumPy and SciPy. See report.
"""
from pathlib import Path
import sys, csv
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from scipy.stats import qmc
ROOT=Path(__file__).resolve().parent.parent
sys.path[:0]=[str(ROOT/'legs'),str(ROOT/'search')]
import legs
import arm_model as arm
L=.15
REF=legs.sagittal_q(-.40,1.25,yaw=np.deg2rad(10))
GROUND=legs.stance(q=REF)
W0=np.eye(4);W0[2,3]=-GROUND.sole_points[:,2].min()
ANCHOR=W0@GROUND.feet
HULL=ConvexHull(GROUND.sole_points[:,:2]).equations
IFS=[legs.specs(side) for side in legs.SIDES]
LEG_LIMITS=np.array([legs.vec(el.find('joint').get('range')) for side in legs.SIDES for el in legs.link_specs(side)])
LIMITS=np.vstack((LEG_LIMITS,arm.LIMITS))
D=[slice(0,5),slice(5,10),slice(10,16)]
def compose(R,p,u):
    return (R@np.asarray(u)[...,None])[...,0]+p
def chain(angles,frames,tail,coms=None):
    r=np.broadcast_to(np.eye(3),(len(angles),3,3))
    p=np.zeros((len(angles),3));locs=[];axes=[];centers=[]
    for j,f in enumerate(frames):
        p=compose(r,p,f[:3,3]);r=r@f[:3,:3]
        locs.append(p);axes.append(r[:,:,2])
        c=np.cos(angles[:,j]);s=np.sin(angles[:,j])
        rz=np.zeros_like(r);rz[:,0,0]=c;rz[:,1,1]=c
        rz[:,1,0]=s;rz[:,0,1]=-s;rz[:,2,2]=1.
        r=r@rz
        if coms is not None:centers.append(compose(r,p,coms[j]))
    t=compose(r,p,tail[:3,3])
    return (r@tail[:3,:3],t,np.stack(locs,1),np.stack(axes,1),
            np.stack(centers,1) if centers else None)
def relative_jac(p,locs,axes):
    return np.concatenate((np.cross(axes,p[:,None,:]-locs)/L,axes),axis=2).swapaxes(1,2)
def rot_jac(R,j):
    return np.concatenate((R@j[:,:3], R@j[:,3:]),axis=1)
def FK(q,frames,metrics=False):
    ls=[chain(q[:,sl],sp[0],sp[4],sp[2] if metrics else None) for sl,sp in zip(D,IFS)]
    # Fixed original Duck trunk and first 2 neck links; modified yaw + terminal
    # case lumped on yaw link. SO masses are taken directly from the upstream arm.
    # custom layouts can decouple terminal and SO lower-arm link frames
    so_lower_from_terminal=frames[4]@np.linalg.inv(arm.WRIST)
    so_centroid=compose(so_lower_from_terminal[:3,:3],so_lower_from_terminal[:3,3],np.array([-.0980701,.00324376,.0182831]))
    grip_com=(.087*np.array([.000213627,.000245138,-.025187])+.012*np.array([.0202-.00157495,.0188-.0192755,-.0234-.0300244]))/.099
    arm_com=[[-.00000563137,.0492968,.0181786],[-.00766247,.026015,.0186681],
             (.055*np.array([.01280146,0,-.01847294])+.055*compose(frames[3][:3,:3],frames[3][:3,3],np.array([.01280146,0,.01857294])))/.110,so_centroid,[-.000103312,-.0386143,.0281156],grip_com] if metrics else None
    ar=chain(q[:,10:],frames[:6],frames[6],arm_com)
    R=ANCHOR[0,:3,:3]@ls[0][0].transpose(0,2,1)
    p=ANCHOR[0,:3,3]-(R@ls[0][1][...,None])[...,0]
    left,right=ls
    tip=(R@ar[0],compose(R,p,ar[1])); foot=(R@right[0],compose(R,p,right[1]))
    J=np.zeros((len(q),12,16))
    J[:,:6,:5]=rot_jac(R,-relative_jac(ar[1],left[2],left[3]))
    J[:,:6,10:]=rot_jac(R,relative_jac(ar[1],ar[2],ar[3]))
    J[:,6:,:5]=rot_jac(R,-relative_jac(right[1],left[2],left[3]))
    J[:,6:,5:10]=rot_jac(R,relative_jac(right[1],right[2],right[3]))
    if not metrics:return tip,foot,J
    trunk=legs.TRUNK.find('inertial');m0=float(trunk.get('mass'));moment=m0*legs.vec(trunk.get('pos'))
    wh=np.array([.06618,.0169378,.128,.104,.079,.099])
    moment=moment+np.einsum('j,njk->nk',wh,ar[4])
    mass=m0+sum(wh)
    for sp,le in zip(IFS,ls):
        mass+=sum(sp[1]);moment+=np.einsum('j,njk->nk',sp[1],le[4])
    com=compose(R,p,moment/mass)
    margin=-np.max(com[:,:2]@HULL[:,:2].T+HULL[:,2],axis=1)
    # Eliminate right-foot closure in the 16-actuator metric; left contact is exact by construction.
    jac=J[:,:6];closure=J[:,6:,:10]
    _,s,vec=np.linalg.svd(closure,full_matrices=True)
    if (s[:,-1]<1e-8).any():raise ValueError('Degenerate support configuration')
    leg=jac[:,:,:10]@vec[:,6:,:].swapaxes(1,2)
    sv=np.linalg.svd(np.concatenate((leg,jac[:,:,10:]),axis=2),compute_uv=False)
    return tip,foot,J,margin,sv,com,mass
def rot_err(target,actual):
    return Rotation.from_matrix(target@actual.transpose(0,2,1)).as_rotvec()
def errors(f,tip,foot):
    tp,tr=f
    et=np.concatenate(((tp-tip[1])/L,rot_err(tr,tip[0])),axis=1)
    ef=np.concatenate(((ANCHOR[1,:3,3]-foot[1])/L,rot_err(ANCHOR[1,:3,:3],foot[0])),axis=1)
    return np.concatenate((et,ef),axis=1)
def make_targets(count=64):
    u=qmc.Sobol(6,scramble=True,seed=701).random_base2(int(np.log2(count)))
    p=np.array([.18,-.065,.365])+u[:,:3]*[.12,.13,.12]
    euler=(u[:,3:]*2-1)*[40,30,45]
    r=Rotation.from_euler('ZYX',euler,degrees=True).as_matrix()
    v=qmc.Sobol(3,scramble=True,seed=307).random_base2(5)
    u1,u2,u3=v.T
    quats=np.column_stack([np.sqrt(1-u1)*np.sin(2*np.pi*u2),np.sqrt(1-u1)*np.cos(2*np.pi*u2),
                            np.sqrt(u1)*np.sin(2*np.pi*u3),np.sqrt(u1)*np.cos(2*np.pi*u3)])
    return np.vstack((p,p[:len(v)])),np.concatenate((r,Rotation.from_quat(quats).as_matrix()))
def stance_targets(reach=0.):
    p,r=make_targets()
    yaw=np.zeros(len(p));offset=np.zeros_like(p)
    if reach:
        # After walking, plant the same two-foot pattern facing each tool's
        # projected approach direction. Do not treat step motion as an actuator.
        yaw=np.arctan2(r[:,1,0],r[:,0,0])
        srot=Rotation.from_euler('z',yaw[:,None]).as_matrix()
        rel=p.copy();rel[:,0]=reach;rel[:,1]=0
        offset=p-(srot@rel[...,None])[...,0]
        p=rel;r=srot.transpose(0,2,1)@r
    return p,r,offset,yaw
def solve(frames,starts,active,steps=55,reach=0.):
    tp,tr,offset,yaw=stance_targets(reach);size=len(tp)
    tg=(np.tile(tp,(starts,1)),np.tile(tr,(starts,1,1)))
    q=np.tile(np.r_[REF.ravel(),np.zeros(6)],(size*starts,1))
    u=qmc.Sobol(6,scramble=True,seed=102).random_base2(int(np.ceil(np.log2(starts))))
    ax=np.array([[-17,50],[-37,39],[-70,70],[-25,25],[3,90],[-90,90]])
    seeds=np.deg2rad(ax[:,0]+(ax[:,1]-ax[:,0])*u[:starts])
    if starts>8:
        # Additional starts explore the opposite elbow branch and wider yaw/roll.
        seeds[8:]=arm.LIMITS[:,0]+(arm.LIMITS[:,1]-arm.LIMITS[:,0])*u[8:starts]
    q[:,10:]=np.repeat(seeds,size,axis=0);ids=np.arange(len(q))
    # allow 2 degrees of physical travel beyond every solved joint value
    lo,hi=LIMITS.T+np.deg2rad(np.array([[2.],[-2.]]))
    for i in range(steps):
        if not len(ids):break
        qs=q[ids]
        tip,foot,J=FK(qs,frames);e=errors((tg[0][ids],tg[1][ids]),tip,foot)
        norms=np.linalg.norm(e.reshape(-1,4,3),axis=2)
        settled=(norms<np.array([.0006/L,.006,.00008/L,.00065])).all(axis=1)
        if settled.all():break
        ids=ids[~settled];qs=qs[~settled]; e=e[~settled];J=J[~settled]
        rows=12 if active else 6
        Js=J[:,:rows,:] if active else J[:,:rows,10:]
        err=e[:,:rows].copy()
        if active:
            Js[:,6:]*=2.; err[:,6:]*=2.
        # Active-set projection for saturated joints. Damping limits steps near singularities.
        qs_used=qs if active else qs[:,10:]; limlo=lo if active else lo[10:]; limhi=hi if active else hi[10:]
        mask=np.ones_like(qs_used)
        for it in range(3):
            a=Js*mask[:,None,:]
            cov=a@a.swapaxes(1,2)
            cov[:,np.arange(rows),np.arange(rows)]+=.035**2
            dq=(a.swapaxes(1,2)@np.linalg.solve(cov,err[:,:,None]))[:,:,0]
            if it<2:mask[((qs_used<limlo+.002)&(dq<0))|((qs_used>limhi-.002)&(dq>0))]=0.
        limit=np.max(np.abs(dq),axis=1).clip(min=.24)/.24
        new=np.clip(qs_used+dq/limit[:,None],limlo,limhi)
        if active:q[ids]=new
        else:q[ids,10:]=new
    tip,foot,J,margin,sv,com,mass=FK(q,frames,True)
    if not active:sv=np.linalg.svd(J[:,:6,10:],compute_uv=False)
    errs=np.linalg.norm(errors(tg,tip,foot).reshape(-1,4,3),axis=2)
    errs[:,[0,2]]*=1000*L;errs[:,[1,3]]*=180/np.pi
    ok=(errs<np.array([1.5,1.,.2,.15])).all(1)
    safe=ok&(margin>=.005)
    # Select highest minimum singular value amongst feasible starts per target.
    rank=sv[:,-1]+100*ok+100*safe
    take=np.argmax(rank.reshape(starts,size),axis=0)*size+np.arange(size)
    return q[take],errs[take],sv[take],margin[take],ok[take],safe[take],mass
def run(name,a,e,c,two,active,starts=4,steps=55,reach=0.):
    f=arm.frames(a,e,c,two);q,err,sv,m,ok,safe,mass=solve(f,starts,active,steps,reach)
    op=Path(__file__).resolve().parent; tp,tr=make_targets();_,_,offset,yaw=stance_targets(reach)
    with open(op/f'poses_{name}.csv','w',newline='') as fp:
        w=csv.writer(fp)
        w.writerow(['target','bank','x_m','y_m','z_m','qw','qx','qy','qz','pose_and_feet_ok','static_margin_5mm_ok','tip_mm','tip_deg','right_sole_mm','right_sole_deg','com_margin_mm','sigma_min','sigma_max','condition']+[f'{side}_{j}_rad' for side in legs.SIDES for j in legs.NAMES]+[j+'_rad' for j in ['neck_pitch','head_pitch','head_yaw','terminal','wrist_flex','wrist_roll']]+['stance_x_m','stance_y_m','stance_yaw_rad'])
        for i in range(len(q)):w.writerow([i,'front' if i<64 else 'all_directions',*tp[i],*np.roll(Rotation.from_matrix(tr[i]).as_quat(),1),int(ok[i]),int(safe[i]),*err[i],m[i]*1000,sv[i,-1],sv[i,0],sv[i,0]/max(1e-14,sv[i,-1]),*q[i],*offset[i,:2],yaw[i]])
    for sl in [slice(64),slice(64,None)]:
        b=safe[sl]; sig=sv[sl][b]; print(name,'targets',len(b),'pose',sum(ok[sl]),'balanced',sum(b),'q05smin q95smax q95cond=',np.round([np.quantile(sig[:,-1],.05),np.quantile(sig[:,0],.95),np.quantile(sig[:,0]/sig[:,-1],.95)],3) if len(sig) else '-', 'mass',round(mass,3),flush=True)
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--az',type=float,default=90);p.add_argument('--el',type=float,default=0);p.add_argument('--clock',type=float,default=0);p.add_argument('--two',action='store_true');p.add_argument('--fixed',action='store_true');p.add_argument('--starts',type=int,default=4);p.add_argument('--steps',type=int,default=55)
    p.add_argument('--replant',type=float,default=0.,help='post-step horizontal TCP distance (m)')
    z=p.parse_args();run(z.name,z.az,z.el,z.clock,z.two,not z.fixed,z.starts,z.steps,z.replant)
