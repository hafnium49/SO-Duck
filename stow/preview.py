"""Make the assembled GLB in metres. Decimation applies to this preview only."""
import json
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from verify import OUT, approach, parts, screws, triangles, screen, pose_test


def simplify(tris):
    # A display-only grid weld. Keep the winding of the first surviving source triangle.
    points = tris.reshape(-1,3)
    grid = np.floor((points-points.min(axis=0))/.003).astype(np.int64)
    stride = grid.max(axis=0)+1
    key = (grid[:,0]*stride[1] + grid[:,1])*stride[2] + grid[:,2]
    _, inv, counts = np.unique(key, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    vertices = np.column_stack([np.bincount(inv, weights=axis)/counts for axis in points.T])
    ids = inv.reshape(-1,3)
    distinct = (ids[:,0]!=ids[:,1]) & (ids[:,0]!=ids[:,2]) & (ids[:,1]!=ids[:,2])
    ids = ids[distinct]
    _, first = np.unique(np.sort(ids,axis=1),axis=0,return_index=True)
    return vertices.astype('<f4'), ids[first].ravel().astype('<u4')


def main():
    geo,links = screen.build_geometry(split=('trunk',))
    trunk = [geo.pop(n) for n in list(geo) if n.startswith('trunk.') and n!='trunk.body_middle_top']
    del geo['trunk.body_middle_top']
    trunk.append(screen.read_stl(OUT/'roof_drilled.stl')*.001)
    geo['trunk'] = np.concatenate(trunk)
    links['trunk'] = 'trunk'
    hardware = {**parts(), 'screws': screws()}
    for n,p in hardware.items():
        geo[n] = triangles(p)*.001
        links[n] = 'trunk'
    q = approach()
    q = q[np.unique(np.r_[np.arange(0,len(q),4),len(q)-1])]
    dyn = screen.motions(np.column_stack([np.tile(pose_test.REF.ravel(),(len(q),1)), np.deg2rad(q)]))
    doc = dict(asset={'version':'2.0'}, scene=0, scenes=[{'nodes':[0]}], buffers=[{}],
               bufferViews=[], accessors=[], materials=[], meshes=[],
               nodes=[dict(name='Trunk frame to glTF',rotation=[-2**-.5,0,0,2**-.5],children=[])])
    anim = dict(name='Deploy to supported stow',channels=[],samplers=[])
    raw = bytearray()

    def accessor(array, kind, component=5126):
        while len(raw)%4:
            raw.append(0)
        view = len(doc['bufferViews'])
        doc['bufferViews'].append(dict(buffer=0,byteOffset=len(raw),byteLength=array.nbytes))
        raw.extend(array.tobytes())
        count = len(array)
        a = dict(bufferView=view,componentType=component,count=count,type=kind)
        a.update(min=array.reshape(count,-1).min(0).tolist(),max=array.reshape(count,-1).max(0).tolist())
        doc['accessors'].append(a)
        return len(doc['accessors'])-1

    palette = [[.48,.57,.62],[.25,.38,.43],[.025,.49,.44],[.91,.49,.13],
               [.66,.08,.22],[.14,.14,.18],[.65,.7,.72]]
    for color in palette:
        doc['materials'].append({'pbrMetallicRoughness':{'baseColorFactor':color+[1.],
                                  'metallicFactor':.12,'roughnessFactor':.72},'doubleSided':True})
    times = accessor(np.linspace(0,8,len(q),dtype='<f4'),'SCALAR')
    for n,tri in geo.items():
        if n in hardware:
            verts,indices=np.unique(np.asarray(tri.reshape(-1,3),dtype='<f4'),axis=0,return_inverse=True)
            indices=indices.reshape(-1).astype('<u4')
        else:
            verts,indices=simplify(tri)
        material = (4 if n=='cradle' else 5 if n.startswith('pad') else 6 if n in hardware else
                    2 if n in ('custom_forearm','terminal_case','wrist_holder_and_motor') else
                    3 if n in ('flex_link_and_motor','gripper_and_jaw') else
                    1 if links[n] in ('neck_pitch','head_pitch','yaw') else 0)
        component = 5125
        if len(verts) <= 65535:
            indices = indices.astype('<u2')
            component = 5123
        prim={'attributes':{'POSITION':accessor(verts,'VEC3')},
              'indices':accessor(indices,'SCALAR',component),'material':material}
        doc['meshes'].append({'name':n,'primitives':[prim]})
        frames=dyn[links[n]]
        xyz=frames[:,:3,3].astype('<f4')
        quat=Rotation.from_matrix(frames[:,:3,:3]).as_quat().astype('<f4')
        for i in range(1,len(quat)):
            if np.dot(quat[i-1],quat[i]) < 0:
                quat[i] *= -1
        node=len(doc['nodes'])
        doc['nodes'].append(dict(mesh=len(doc['meshes'])-1,name=n,translation=xyz[-1].tolist(),
                                 rotation=quat[-1].tolist()))
        doc['nodes'][0]['children'].append(node)
        for key,data,kind in [('translation',xyz,'VEC3'),('rotation',quat,'VEC4')]:
            if np.max(np.abs(data-data[-1])) < 1e-6:
                continue
            anim['channels'].append(dict(sampler=len(anim['samplers']),target=dict(node=node,path=key)))
            anim['samplers'].append(dict(input=times,output=accessor(data,kind),interpolation='LINEAR'))
    doc['animations']=[anim]
    while len(raw)%4:
        raw.append(0)
    doc['buffers'][0]['byteLength']=len(raw)
    header=json.dumps(doc,separators=(',',':')).encode()
    header+=b' '*(-len(header)%4)
    path=OUT/'Centered_stow.glb'
    with path.open('wb') as f:
        f.write(struct.pack('<4sII',b'glTF',2,28+len(header)+len(raw)))
        f.write(struct.pack('<I4s',len(header),b'JSON')+header)
        f.write(struct.pack('<I4s',len(raw),b'BIN\0')+raw)
    print(f'{path.name}: {len(geo)} rigid meshes, {len(q)} preview keys')


if __name__=='__main__':
    main()
