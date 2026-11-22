"""Check calibrated joint travel, mounting, contact, balance, path, and saved grasps."""
import argparse
import csv
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

from design import OUT, PARK, PAD_Z82, WORK_ELBOW, elbow_motor, parts, screws, drilled_roof
ROOT = OUT.parent
sys.path.insert(0, str(ROOT / 'collision'))
import screen
import balance
import torque
import pose_test
import legs
import arm_model

ARM_NAMES = ('neck_pitch', 'head_pitch', 'head_yaw', 'terminal', 'wrist_flex', 'wrist_roll')
LEG_NAMES = [f'{side}_{joint}' for side in legs.SIDES for joint in legs.NAMES]


def triangles(shape):
    verts, faces = shape.tessellate(.03, .09)
    return np.array([v.toTuple() for v in verts])[np.asarray(faces)]


def approach():
    # Cast from power-on geometric zero with empty, closed gripper and flat feet.
    # Flex the wrist before lowering the neck, then lower the forearm onto the pads.
    waypoints = [np.zeros(6), [0,0,0,0,90,0], [60,-60,0,0,90,0], PARK]
    samples = [np.asarray(waypoints[0])]
    for start, end in zip(waypoints, waypoints[1:]):
        intervals = int(np.ceil(np.max(np.abs(np.subtract(end, start))) / .5))
        samples.extend(np.linspace(start, end, intervals + 1)[1:])
    return np.array(samples)


def arm_in_trunk(angles):
    frames = arm_model.frames(90,0,90,True)
    zero = screen.chain(frames, np.zeros((1,6)))[3][0]
    current = screen.chain(frames, np.deg2rad(np.atleast_2d(angles)))[3][0]
    t = current @ np.linalg.inv(zero) @ arm_model.TRUNK_FROM_PRINT
    t[:3,3] *= 1000
    move = gp_Trsf()
    move.SetValues(*t[:3].ravel())
    return cq.Location(move)


def contact(shapes):
    boom = cq.importers.importStep(str(ROOT/'cad'/'Case_mounted_forearm.step')).val()
    parked = boom.located(arm_in_trunk(PARK))
    rigid_gap = parked.distance(shapes['cradle'])
    print(f'Rigid cradle to parked forearm: {rigid_gap:.3f} mm')
    assert rigid_gap > 1
    for name in ('pad_left','pad_right'):
        pad = shapes[name]
        gap, volume = parked.distance(pad), parked.intersect(pad).Volume()
        # Check four corners on each finite pad footprint against native B-rep.
        top, = [face for face in pad.Faces() if face.normalAt().z > .5]
        errors = [parked.distance(vertex) for vertex in top.Vertices()]
        print(name, f'gap {gap:.5f} mm, common volume {volume:.5f} mm3,',
              f'max corner error {max(errors):.5f} mm')
        assert len(errors) == 4 and max(errors) < .01 and volume < .1
    release = PARK.copy()
    release[1] += .5
    released = boom.located(arm_in_trunk(release))
    release_gap = min(released.distance(shapes[k]) for k in ('pad_left','pad_right'))
    print(f'First 0.5° of release opens {release_gap:.3f} mm at the pads')
    assert release_gap > .5
    roof = drilled_roof()
    for name,piece in {**shapes, 'screws': screws()}.items():
        assert roof.intersect(piece).Volume() < .02, f'Roof overlap: {name}'
    return parked


def mass_properties(shapes):
    # Densities in kg/mm^3: packed polymer, TPU, aluminum and steel respectively.
    installed = [(p, (2.70 if k.startswith('backer') else
                      1.20 if k.startswith('pad') else 1.24)*1e-6) for k,p in shapes.items()]
    installed.append((screws(), 7.85e-6))
    mass = sum(p.Volume()*rho for p,rho in installed)
    moment = sum(p.Volume()*rho*np.array(p.Center().toTuple())*.001 for p,rho in installed)
    return mass, moment


def margin(q, hardware):
    stance = legs.stance(q=q[:10].reshape(2,5))
    up = (pose_test.ANCHOR[0] @ np.linalg.inv(stance.feet[0]))[2,:3]
    upper_moment,_ = torque.geometry(q[10:], True)
    upper_mass = balance.MASS.sum()-balance.MASS[2:5].sum()+.0918099+.055+.085+.018
    mass, moment = hardware
    com = (stance.rest_moment + upper_moment + moment) / (stance.rest_mass+upper_mass+mass)
    return float(1000*stance.margin(com, up))


def elbow_reaction():
    # Virtual work for the elbow coordinate, with other axes held and no payload.
    # A centered frictionless pair of pads shares this combined normal reaction.
    eps = 1e-5
    q = np.deg2rad(PARK)
    dq = np.eye(6)[1] * eps
    p1,_ = torque.geometry(q+dq, True)
    p0,_ = torque.geometry(q-dq, True)
    gravity = 9.80665*(p1[2]-p0[2])/(2*eps)
    frames = arm_model.frames(90,0,90,True)
    terminal = screen.chain(frames, np.stack([q,q+dq,q-dq]))[3]
    pad = np.array([.082, 0, .001*PAD_Z82, 1])
    local = np.linalg.solve(terminal[0], pad)
    velocity = ((terminal[1]-terminal[2])@local)[:3]/(2*eps)
    angle = np.deg2rad(PARK[0]+PARK[1])
    normal = np.array([-np.sin(angle),0,np.cos(angle)])
    reaction = gravity / (normal @ velocity)
    assert reaction > 0, 'Support is on the wrong side of the elbow'
    print(f'Empty elbow gravity {abs(gravity):.3f} Nm, normal lever '
          f'{abs(normal@velocity)*1000:.1f} mm, combined pad reaction {reaction:.2f} N')
    return reaction


def write_csv(path, columns, rows):
    with path.open('w', newline='') as fp:
        writer = csv.writer(fp, lineterminator='\n')
        writer.writerow(columns)
        writer.writerows(rows)


class MeshCheck:
    """Native collision surfaces in mm, placements in SI as in collision/screen.py."""
    def __init__(self, geometry):
        self.lib = screen.collision_api()
        self.handles = {}
        for key, tri in geometry.items():
            tri = np.ascontiguousarray(tri, dtype=float)
            self.handles[key] = self.lib.load(tri.ctypes.data, len(tri))

    def hits(self, first, second, first_frame, second_frame):
        rel = np.ascontiguousarray(np.linalg.inv(second_frame) @ first_frame)
        rel[:,:3,3] *= 1000
        mask = np.zeros(len(rel), dtype='uint8')
        self.lib.collisions(self.handles[first], self.handles[second], rel.ctypes.data,
                            mask.ctypes.data, len(rel))
        return mask.astype(bool)


def main(out):
    out.mkdir(parents=True, exist_ok=True)
    shapes = parts()
    contact(shapes)
    elbow_reaction()
    hardware = mass_properties(shapes)
    print(f'Added mass estimate {hardware[0]*1000:.1f} g')
    angles = approach()
    work = arm_model.LIMITS.copy()
    assert np.allclose(np.rad2deg(work[1]),WORK_ELBOW)
    work[1,0] = np.deg2rad(PARK[1])
    assert np.all(np.deg2rad(angles) >= work[:,0]-1e-8) and np.all(np.deg2rad(angles) <= work[:,1]+1e-8)
    print('Elbow raw angle at park and at the +45° work limit:', elbow_motor(np.array([PARK[1],45.])))
    q_path = np.column_stack([np.tile(pose_test.REF.ravel(), (len(angles),1)), np.deg2rad(angles)])
    # Run the original 146-pair checker and its floor test without changing archived study results.
    with TemporaryDirectory() as tmp:
        move = Path(tmp)/'stow.csv'
        write_csv(move, ['target','bank','pose_and_feet_ok','static_margin_5mm_ok']+
                  [n+'_rad' for n in LEG_NAMES+list(ARM_NAMES)],
                  ([i,'path',1,int(margin(q,hardware)>=5),*q] for i,q in enumerate(q_path)))
        screen.screen(move, Path(tmp))
        with (Path(tmp)/'collision_stow.csv').open() as f:
            baseline = list(csv.DictReader(f))
    # Split the native neck for extra non-mating checks at the extended elbow angles.
    geometry, links = screen.build_geometry(split=('neck_pitch','trunk'))
    meshes = {k:v*1000 for k,v in geometry.items()}
    fixtures = {**shapes, 'screws': screws()}
    meshes.update({k:triangles(v) for k,v in fixtures.items()})
    checker = MeshCheck(meshes)
    banks = []
    for name in ('O90','Replant22','Replant27'):
        rows, q = screen.build_inputs(ROOT/'study'/f'poses_{name}.csv')
        with (ROOT/'collision'/f'collision_poses_{name}.csv').open() as fp:
            original = {r['target']:r for r in csv.DictReader(fp)}
        banks.append((name, rows, q, original))
    q = np.vstack([q_path]+[b[2] for b in banks])
    dyn = screen.motions(q)
    fixture_hits = [[] for _ in q]
    for part in fixtures:
        for n,link in links.items():
            if link == 'trunk':
                continue
            hit = checker.hits(part,n,dyn['trunk'],dyn[link])
            for i in np.flatnonzero(hit):
                # The only designed moving contacts are the two soft pads at the seated endpoint.
                if part.startswith('pad') and n=='custom_forearm' and i==len(q_path)-1:
                    continue
                fixture_hits[i].append(f'{part}/{n}')
    # Static installed clearance to every native trunk part other than the drilled roof.
    # Roof contact and through-bores are checked above with the modified native B-rep.
    identity = np.eye(4)[None]
    for part in fixtures:
        for n,link in links.items():
            if link == 'trunk' and n != 'trunk.body_middle_top':
                assert not checker.hits(part,n,identity,identity)[0], f'Installed clash: {part}/{n}'
    # Ground is horizontal in each saved world pose. New hardware is fixed to the torso.
    world = pose_test.ANCHOR[0] @ np.linalg.inv(dyn['left_ankle'] @ legs.specs('left')[4])
    for part in fixtures:
        verts = np.unique(meshes[part].reshape(-1,3), axis=0)*.001
        lowest = (world[:,2,:3] @ verts.T).min(axis=1)+world[:,2,3]
        for i in np.flatnonzero(lowest < -.0001):
            fixture_hits[i].append(f'{part}/ground')
    # Extra near-neighbor checks skipped by the legacy robot screen.
    extra = [[] for _ in q_path]
    for n,link in links.items():
        others = []
        if link=='neck_pitch' and 'sheet' in n:
            others = ['trunk.trunk_top']
        if link=='head_pitch':
            others = [k for k in links if k.startswith('neck_pitch.') and
                      ('sheet' in k or 'case' in k or 'cabinet' in k)]
        if link=='yaw':
            others += [k for k in links if k.startswith('trunk.')]
        for o in others:
            hits = checker.hits(n,o,dyn[link][:len(q_path)],dyn[links[o]][:len(q_path)])
            for i in np.flatnonzero(hits):
                extra[i].append(f'{n}/{o}')
    path = []
    for i,(q0,row) in enumerate(zip(q_path,baseline)):
        collisions = '|'.join(filter(None,[row['intersecting_pairs'],*extra[i],*fixture_hits[i]]))
        path.append((i,*angles[i],round(elbow_motor(angles[i,1]),3),round(margin(q0,hardware),3),
                     round(float(row['min_nonfoot_ground_clearance_mm']),3),collisions))
    write_csv(out/'stow_path.csv',['sample',*[n+'_deg' for n in ARM_NAMES],'elbow_motor_deg',
                                  'static_margin_mm','min_nonfoot_floor_mm','unintended_contacts'],path)
    report = []
    offset = len(path)
    for name,rows,qs,original in banks:
        for row,qi,hits in zip(rows,qs,fixture_hits[offset:offset+len(rows)]):
            old = (row['pose_and_feet_ok']=='1' and row['static_margin_5mm_ok']=='1' and
                   original[row['target']]['hit_count']=='0')
            m = margin(qi,hardware)
            bare = margin(qi,(0.,np.zeros(3)))
            report.append((name,row['bank'],row['target'],int(old),round(bare,3),round(m,3),'|'.join(hits),
                           int(old and m>=5 and not hits)))
        offset += len(rows)
    write_csv(out/'grasp_replay.csv',['stance','bank','target','original_pass',
              'bare_heavier_margin_mm','revised_margin_mm','fixture_contacts','retained'],report)
    print('Path samples',len(path),'unintended-hit samples',sum(bool(r[-1]) for r in path),
          'min/end static margins (mm)', min(r[-3] for r in path), path[-1][-3])
    for bank in ('front','all_directions'):
        subgroup = [r for r in report if r[1]==bank]
        print(bank,'retained unique targets / original:',
              len({r[2] for r in subgroup if r[-1]}),len({r[2] for r in subgroup if r[3]}))
        assert {r[2] for r in subgroup if r[-1]} == {r[2] for r in subgroup if r[3]}, (
            f'Lost a previously feasible {bank} target across all stances')
    if any(r[-1] or r[-3]<5 or r[-2]<0 for r in path):
        raise RuntimeError('Stow path failed; see stow_path.csv')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, default=OUT/'results')
    main(parser.parse_args().out)
