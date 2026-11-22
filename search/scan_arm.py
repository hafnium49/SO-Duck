"""Compare carrier orientations over a common Sobol sample. All angles in the CSV are degrees."""
import csv
from pathlib import Path
import numpy as np
from scipy.stats import qmc
from arm_model import LIMITS,frames,fk

FOCUS=np.deg2rad([[-10,50],[-35,35],[-75,75],[-27,27],[-70,70],[-115,115]])
OUT=Path(__file__).resolve().parent
def sample(limits,exp,seed):
    s=qmc.Sobol(6,scramble=True,seed=seed).random_base2(exp)
    return qmc.scale(s,limits[:,0],limits[:,1])
def stats(j):
    sv=np.linalg.svd(j,compute_uv=False)
    hi=sv[:,0];lo=sv[:,-1]; cond=hi/np.maximum(lo,np.finfo(float).tiny)
    return dict(smin05=np.quantile(lo,.05),smin10=np.quantile(lo,.10),smin50=np.median(lo),
                smax95=np.quantile(hi,.95),k95=np.quantile(cond,.95),k90=np.quantile(cond,.90),
                k50=np.median(cond),fraction_k20=np.mean(cond<=20),fraction_k30=np.mean(cond<=30),
                band=np.mean((lo>=.10)&(hi<=4)))
def run(config,q):
    return stats(fk(q,frames(*config))[1])
def write(rows,file):
    with (OUT/file).open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0])); w.writeheader();w.writerows(rows)
def matrix(wide=True):
    return [(a,e,c,False) for a in ([0,30,45,60,75,90,105,120,135,150,180] if wide else [90]) for e in ([-30,-15,0,15,30] if wide else [0]) for c in range(-90,91,15)]
def evaluate(cfgs,q):
    rows=[]
    for cfg in cfgs:
        a,e,c,t=cfg
        rows.append(dict(azimuth=a,elevation=e,clock=c,two_part=t,**run(cfg,q)))
    return rows
def shortlist(grid):
    sagittal=[r for r in grid if r['azimuth']==90 and r['elevation']==0]
    best=lambda arr: sorted(arr,key=lambda r:r['k95'])[:5]
    return best(grid),best(sagittal)
def design(r):
    return (r['azimuth'],r['elevation'],r['clock'],r['two_part'])
if __name__=='__main__':
    broad=sample(LIMITS,10,4591)
    rows=evaluate(matrix(),broad)
    write(rows,'arm_grid.csv')
    free,sag=shortlist(rows)
    cfgs=list(dict.fromkeys([design(r) for r in free+sag]+matrix(False)+[(0,0,0,False),(90,0,0,True)]))
    for name,limits in [('full',LIMITS),('central',FOCUS)]:
        q=sample(limits,14,123094)
        rows=evaluate(cfgs,q)
        write(rows,'arm_validation_'+name+'.csv')
        free,sag=shortlist(rows)
        print(name, 'global',free[0], 'sagittal',sag[0],flush=True)
