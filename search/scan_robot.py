"""Joint-space whole-robot Jacobian survey under rigid stance-foot closure.

Run with Python, NumPy and SciPy. Requires arm_model.py and the sibling legs
directory containing legs.py, duck.xml and sole.stl. The arm Sobol samples are
shared across designs and modes. The extra columns are leg-actuated torso motion
in an orthonormal basis of leg actuator rates satisfying support constraints.
"""
import sys
from pathlib import Path
from itertools import product
import numpy as np
from scan_arm import FOCUS,LIMITS,OUT,frames,fk,sample,stats,write,design,matrix
sys.path.insert(0,str(OUT.parent/'legs'))
import legs

MODES=('double','left','right')
METRICS=('smin05','smin10','smin50','smax95','k95','k90','k50','fraction_k20','fraction_k30','band')
def whole(s,pose,j,length):
    legj=s.tip_jac(pose[...,:3,3]).copy()
    legj[...,:3,:] /= length
    return np.concatenate([legj,j],axis=-1)
def evaluate(configs,q,stances,length=.15):
    rows=[]
    for cfg in configs:
        a,e,c,t=cfg
        pose,j=fk(q,frames(*cfg),length=length)
        for name,s in stances.items():
            whole_j=j if s is None else whole(s,pose,j,length)
            rows.append(dict(azimuth=a,elevation=e,clock=c,two_part=t,mode=name,length=length,**stats(whole_j)))
    return rows
def key(r):
    return r['k95']
def narrow(grid):
    keep=[]
    for mode in MODES:
        rr=[r for r in grid if r['mode']==mode]
        for coll in ((r for r in rr if not r['two_part'] and r['azimuth']==90 and r['elevation']==0),
                     (r for r in rr if not r['two_part']), (r for r in rr if r['two_part'])):
            keep.extend(design(r) for r in sorted(coll,key=key)[:4])
    return list(dict.fromkeys(keep))
def table(rows):
    return [(design(r),r['mode'],*[round(r[k],4) for k in ('smin05','smax95','k50','k95')]) for r in rows]
if __name__=='__main__':
    stance_q=legs.sagittal_q(hip=-.40,knee=1.25,yaw=np.deg2rad(10))
    stances={m:legs.stance(mode=m,q=stance_q) for m in MODES}
    grid=matrix()+[(90,0,c,True) for c in range(-90,91,15)]
    search=evaluate(grid,sample(LIMITS,9,76410),stances)
    write(search,'robot_grid.csv')
    shortlist=list(dict.fromkeys(narrow(search)+matrix(False)+[(90,0,c,True) for c in range(-90,91,15)]+[(0,0,0,False)]))
    for lname,limits in [('full',LIMITS),('central',FOCUS)]:
        q=sample(limits,13,840701)
        validation=evaluate(shortlist,q,stances)
        write(validation,'robot_validation_'+lname+'.csv')
        for mode in MODES:
            group=[r for r in validation if r['mode']==mode]
            for variants in [[r for r in group if r['azimuth']==90 and r['elevation']==0 and r['two_part']==b] for b in [False,True]]:
                print(lname,*table(sorted(variants,key=key)[:3]),flush=True)
    sens=[]
    q=sample(LIMITS,13,840701)
    for length in [.10,.15,.20]:
        shortlist=[(90,0,c,b) for c,b in product(range(0,91,15),(False,True))]
        sens.extend(evaluate(shortlist,q,stances,length))
    write(sens,'robot_sensitivity.csv')
