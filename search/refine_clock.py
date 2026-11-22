"""Forward-clock sensitivity with the orthogonal terminal/wrist pair."""
from scan_robot import legs,frames,fk,write,LIMITS,FOCUS,sample,evaluate
from pathlib import Path
import numpy as np
if __name__=='__main__':
    stance_q=legs.sagittal_q(-.40,1.25,yaw=np.deg2rad(10))
    s={'double':legs.stance('double',stance_q)}
    configs=[(90,0,c,True) for c in range(45,151,5)]
    for group,lim in [('full',LIMITS),('central',FOCUS)]:
        rows=evaluate(configs,sample(lim,14,840701),s)
        for r in rows:
            fs=frames(90,0,r['clock'],True)
            p=fk(np.zeros(6),fs)[0][:3,3]
            r.update(zero_tip_x_m=p[0],zero_tip_y_m=p[1],zero_tip_z_m=p[2])
        write(rows,'clock_refinement_'+group+'.csv')
        print(group,*(f"{r['clock']}deg/{r['k95']:.1f}" for r in rows))
