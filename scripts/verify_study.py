"""Check study files/hashes, metadata and CSV schemas with no external dependencies.

Not a physics validation; checks only reproducibility/integrity invariants.
"""
from pathlib import Path
from hashlib import sha256
import csv, json, sys, ast

ROOT=Path(__file__).resolve().parents[1]
required_paths = [
 'cad/Case_mounted_forearm.step', 'cad/Case_mounted_forearm.stl',
 'cad/Forward_neck_and_wrist.glb', 'cad/build_forearm.py',
 'legs/duck.xml', 'legs/legs.py', 'study/pose_test.py',
 'collision/bvh.cpp', 'collision/screen.py',
 'collision/inputs/robot_motors.xml',
 'SO_Duck_pose_explorer.html', 'search/arm_model.py',
 'study/poses_O90.csv', 'study/poses_Replant22.csv', 'study/poses_Replant27.csv',
 'collision/collision_poses_O90.csv', 'docs/SHA256SUMS.txt',
]

def main():
 errors=[]
 for rel in required_paths:
  if not (ROOT/rel).is_file(): errors.append('missing: '+rel)
 manifest=ROOT/'docs/STUDY_INVENTORY.json'
 if manifest.exists():
  d=json.loads(manifest.read_text(encoding='utf-8'))
  for e in d['files']:
   p=ROOT/e['path']
   if not p.is_file(): errors.append('missing inventoried: '+e['path']); continue
   raw=p.read_bytes()
   if len(raw)!=e['bytes'] or sha256(raw).hexdigest()!=e['sha256']:
    errors.append('content mismatch: '+e['path'])
  if sum(e['bytes'] for e in d['files']) != d['total_bytes']: errors.append('inventory total mismatch')
 else: errors.append('missing docs/STUDY_INVENTORY.json')
 for rel, keys in [
  ('study/poses_O90.csv', {'target','bank','x_m','y_m','z_m','pose_and_feet_ok','com_margin_mm'}),
  ('study/endpoint_summary.csv', {'placement','target','bank','screened','gain_bounded'}),
  ('collision/collision_poses_O90.csv',{'target','bank','collision_checked','hit_count'}),
  ('search/robot_grid.csv',{'azimuth','elevation','clock','two_part','smin05','smax95','k95'}),
  ('legs/gravity_O90.csv',{'payload_g','tau_head_pitch_Nm'}),
 ]:
  f=ROOT/rel
  if not f.is_file(): errors.append('missing dataset '+rel); continue
  with f.open(newline='',encoding='utf-8') as inp: hdr=set(next(csv.reader(inp),[]))
  if not keys.issubset(hdr): errors.append('missing CSV columns in '+rel+': '+','.join(sorted(keys-hdr)))
 for p in ROOT.rglob('*.py'):
  if '__pycache__' not in p.parts:
   try: ast.parse(p.read_text(encoding='utf-8'),filename=str(p))
   except SyntaxError as e: errors.append(str(e))
 for path,magic in [('cad/Case_mounted_forearm.step',b'ISO-10303-21;'),('cad/Forward_neck_and_wrist.glb',b'glTF')]:
  p=ROOT/path
  if p.is_file() and not p.read_bytes().startswith(magic): errors.append('invalid file signature '+path)
 if errors:
  print('STUDY CHECK FAILED\n'+'\n'.join('- '+x for x in errors),file=sys.stderr)
  return 1
 print('STUDY CHECK PASSED — inventoried bytes match, expected research source present, CSV signatures and Python syntax valid (NOT a physics or hardware test)')
 return 0

if __name__=='__main__':raise SystemExit(main())
