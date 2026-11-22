"""Triangle intersection screening of hybrid arm against trunk, neck and both legs at achieved poses."""
from pathlib import Path
import ctypes as ct
import csv,sys,re,collections
import xml.etree.ElementTree as ET
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation
here=Path(__file__).resolve().parent
sys.path[:0]=[str(here.parent/'search'),str(here.parent/'legs')]
import arm_model as a
import legs
TREE=here/'inputs'
SOFILE=TREE/'so101.xml'
ASSETS=TREE/'arm_meshes'
@lru_cache
def read_stl(path):
    x=path.read_bytes();n=int.from_bytes(x[80:84],'little')
    if len(x)==84+50*n:
        return np.frombuffer(x,offset=84,dtype=[('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')])['v'].astype(float)
    from vtkmodules.vtkIOGeometry import vtkSTLReader
    from vtkmodules.util.numpy_support import vtk_to_numpy
    r=vtkSTLReader();r.SetFileName(str(path));r.Update();o=r.GetOutput()
    return vtk_to_numpy(o.GetPoints().GetData())[vtk_to_numpy(o.GetPolys().GetConnectivityArray()).reshape(-1,3)]
def move(t,v):return v@t[:3,:3].T+t[:3,3]
def rotz(q):
    out=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
    co=np.cos(q);si=np.sin(q)
    out[:,0,0]=co;out[:,1,1]=co;out[:,0,1]=-si;out[:,1,0]=si
    return out
def chain(fs,q):
    acc=np.broadcast_to(np.eye(4),(len(q),4,4))
    out=[]
    for j,f in enumerate(fs[:q.shape[1]]):
        acc=acc@f@rotz(q[:,j]);out.append(acc)
    return out
def build_geometry(split=()):
    raw=(TREE/'robot_motors.xml').read_text()
    raw=re.sub(r'<!--\s*(<geom\b[^>]*?mesh="wj-[^"]+"\s*/>)\s*-->',r'\1',raw)
    duck=ET.fromstring(raw)
    so=ET.parse(SOFILE)
    fs=a.frames(90,0,90,True)
    zero=chain(fs,np.zeros((1,6)))
    name_by_body={'trunk_assembly':'trunk','neck_pitch_assembly':'neck_pitch','head_pitch_to_yaw':'head_pitch','neck_yaw_assembly':'yaw'}
    for side in legs.SIDES:
        name_by_body.update({link.get('name'):side+'_'+n for link,n in zip(legs.link_specs(side),legs.NAMES)})
    groups=collections.defaultdict(list); links={}
    def add(name,link,path,T,scale=1):
        if name in split:name=name+'.'+path.stem
        groups[name].append(move(T,read_stl(path)*scale));links[name]=link
    for node in duck.iter('body'):
        label=name_by_body.get(node.get('name'))
        if not label:continue
        for geo in node.findall('geom'):
            name=geo.get('mesh','')
            f=TREE/'duck_meshes'/(name+'.stl')
            if f.is_file() and name!='head_yaw_to_roll':
                add(label,label,f,legs.rest(geo))
    g=np.linalg.inv(zero[2][0])@a.TRUNK_FROM_PRINT
    add('yaw','yaw',TREE/'yaw_to_roll.stl',g,.001)
    g=np.linalg.inv(zero[3][0])@a.TRUNK_FROM_PRINT
    add('custom_forearm','terminal',here/'candidate_forearm.stl',g,.001)
    m=a.T(np.diag([-1,1,-1]),[.0125,0,.0187])
    tf=a.T(np.eye(3),[-.018,0,0])@a.rotate_about(a.turn('x',180),[0,0,.12906])@a.ROOT@m
    add('terminal_case','terminal',ASSETS/'sts3215_03a_v1.stl',g@tf)
    sources=[('lower_arm','terminal',fs[4]@np.linalg.inv(a.WRIST)),
             ('wrist','wrist',np.eye(4)),('gripper','roll',np.eye(4))]
    finger=so.find(".//body[@name='gripper']/body")
    sources.append((finger.get('name'),'roll',legs.rest(finger)))
    for n,link,offset in sources:
        for geo in so.find(f".//body[@name='{n}']").findall("geom[@class='visual']"):
            mesh=geo.get('mesh')
            if mesh=='under_arm_so101_v1':continue
            group={'terminal':'wrist_holder_and_motor','wrist':'flex_link_and_motor','roll':'gripper_and_jaw'}[link]
            add(group,link,ASSETS/(mesh+'.stl'),offset@legs.rest(geo))
    return {k:np.concatenate(v) for k,v in groups.items()},links
def motions(q):
    f=a.frames(90,0,90,True)
    arm=chain(f,q[:,10:]); dyn=dict(zip(('neck_pitch','head_pitch','yaw','terminal','wrist','roll'),arm))
    dyn['trunk']=np.broadcast_to(np.eye(4),(len(q),4,4))
    for i,side in enumerate(legs.SIDES):
        fs=legs.specs(side)[0]
        mats=chain(fs,q[:,i*5:i*5+5])
        dyn.update({side+'_'+k:v for k,v in zip(legs.NAMES,mats)})
    return dyn
def collision_api():
    lib=ct.CDLL(str(here/'bvh.so'));P=ct.c_void_p
    lib.load.argtypes=[P,ct.c_int];lib.load.restype=P
    lib.collisions.argtypes=[P,P,P,P,ct.c_int];lib.collisions.restype=ct.c_int
    return lib
def build_inputs(path):
    rows=list(csv.DictReader(open(path)))
    leg_names=[side+'_'+joint+'_rad' for side in legs.SIDES for joint in legs.NAMES]
    arm_names=[joint+'_rad' for joint in ('neck_pitch','head_pitch','head_yaw','terminal','wrist_flex','wrist_roll')]
    q=np.array([[float(r[k]) for k in leg_names+arm_names] for r in rows])
    return rows,q
def screen(file, output_dir=here):
    rows,q=build_inputs(file)
    geometry,links=build_geometry()
    dyn=motions(q)
    lib=collision_api()
    handles={}
    for name,tris in geometry.items():
        mm=np.ascontiguousarray(tris*1000)
        handles[name]=lib.load(mm.ctypes.data,len(mm))
    distal=['custom_forearm','terminal_case','wrist_holder_and_motor','flex_link_and_motor','gripper_and_jaw']
    obstacles=[n for n in geometry if n not in distal]
    pairs=[(n,o) for n in distal for o in obstacles if (n,o)!=('terminal_case','yaw')]
    pairs.extend((r,'gripper_and_jaw') for r in ['custom_forearm','terminal_case','wrist_holder_and_motor'])
    # The flex link and the terminal drive are separated by the forearm span.
    pairs.append(('terminal_case','flex_link_and_motor'))
    hybrid_pair_count=len(pairs)
    # Non-adjacent native structural members with no intended mating surface.
    # Upstream exclusions omit torso-to-proximal hip and torso-to-yaw.
    leg_names=lambda side:[side+'_'+k for k in legs.NAMES]
    left,right=map(leg_names,legs.SIDES)
    pairs.extend((n,o) for n in left for o in right)
    for side in (left,right):
        pairs.extend((n,o) for i,n in enumerate(side) for o in side[i+2:])
        pairs.extend((n,o) for n in side for o in ['neck_pitch','head_pitch','yaw'])
        pairs.extend((n,'trunk') for n in side[3:])
    pairs.extend([('head_pitch','trunk'),('yaw','neck_pitch')])
    masks=[]
    for n,o in pairs:
        rel=np.ascontiguousarray(np.linalg.inv(dyn[links[o]])@dyn[links[n]])
        rel[:,:3,3]*=1000
        out=np.zeros(len(q),dtype=np.uint8)
        lib.collisions(handles[n],handles[o],rel.ctypes.data,out.ctypes.data,len(q))
        masks.append(out)
    # Absolute floor z from the flat planted left ankle site. Replant scenarios
    # translate horizontally, so the same left reference z is applicable.
    from scipy.spatial import ConvexHull
    stance=legs.stance('double',legs.sagittal_q(-.40,1.25,yaw=np.deg2rad(10)))
    anchor=stance.feet[0].copy()
    anchor[2,3]-=stance.sole_points[:,2].min()
    foot_local=legs.specs('left')[4]
    world=anchor@np.linalg.inv(dyn['left_ankle']@foot_local)
    ground=[]
    for n,v in geometry.items():
        if n.endswith('_ankle'):continue
        pts=v.reshape(-1,3); extreme=pts[ConvexHull(pts).vertices]
        row=(world@dyn[links[n]])[:,2,:]
        minimum=(row[:,:3]@extreme.T+row[:,3,None]).min(axis=1)
        ground.append((n,minimum))
    hist=collections.Counter();tags=[];hybrid=[]
    for idx,r in enumerate(rows):
        hits=['/'.join(pair) for pair,mask in zip(pairs,masks) if mask[idx]]
        hybrid.append('|'.join('/'.join(pair) for pair,mask in zip(pairs[:hybrid_pair_count],masks[:hybrid_pair_count]) if mask[idx]))
        hits.extend(n+'/ground' for n,h in ground if h[idx]<-.0001)
        tags.append('|'.join(hits))
        if bool(int(r['static_margin_5mm_ok'])):hist.update(hits)
    outname='collision_'+file.name
    with (output_dir/outname).open('w',newline='') as stream:
        cols=['target','bank','pose_and_feet_ok','static_margin_5mm_ok','collision_checked','hybrid_arm_intersecting_pairs','min_nonfoot_ground_clearance_mm','hit_count','intersecting_pairs']
        w=csv.DictWriter(stream,fieldnames=cols);w.writeheader()
        for idx,(r,tag) in enumerate(zip(rows,tags)):
            w.writerow(dict(**{k:r[k] for k in cols[:4]},collision_checked=1,hybrid_arm_intersecting_pairs=hybrid[idx],min_nonfoot_ground_clearance_mm=min(h[idx] for n,h in ground)*1000,hit_count=0 if not tag else tag.count('|')+1,intersecting_pairs=tag))
    print(file.name,len(pairs),'pairs',len(geometry),'mesh groups',sum(len(a) for a in geometry.values()),'triangles',flush=True)
    for bank in sorted({r['bank'] for r in rows}):
        for cond in ['pose_and_feet_ok','static_margin_5mm_ok']:
            sel=[i for i,r in enumerate(rows) if r['bank']==bank and int(r[cond])]
            print(bank,cond,'pass',sum(not tags[i] for i in sel),'of',len(sel),flush=True)
    print('hit counts among nominal static-margin passes',hist,flush=True)
if __name__=='__main__':
    for n in sys.argv[1:]:screen(Path(n))
