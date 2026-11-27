"""Deterministically write a SHA-256 inventory for the research inputs/assets.

The generated inventory excludes its own two output files and transient caches.
Run from any current working directory. Requires only Python standard library.
"""
from pathlib import Path
from hashlib import sha256
import json

ROOT = Path(__file__).resolve().parents[1]
EXCLUDE = {'docs/SHA256SUMS.txt', 'docs/STUDY_INVENTORY.json', 'collision/bvh.so'}

def tracked_files():
    for p in sorted(ROOT.rglob('*')):
        if not p.is_file() or {'.git', '__pycache__', '.pytest_cache', '.superpowers'} & set(p.parts):
            continue
        rel = p.relative_to(ROOT).as_posix()
        if rel in EXCLUDE or rel.startswith(('archives/study-assets.zip.part', 'hollow/parts/stock/')) or p.suffix in {'.log', '.pyc', '.zip', '.tmp'}:
            continue
        yield p, rel

def main():
    entries = []
    for p, rel in tracked_files():
        raw = p.read_bytes()
        entries.append({'path': rel, 'bytes': len(raw), 'sha256': sha256(raw).hexdigest()})
    (ROOT/'docs/SHA256SUMS.txt').write_text(''.join(f"{e['sha256']}  {e['path']}\n" for e in entries), encoding='utf-8')
    (ROOT/'docs/STUDY_INVENTORY.json').write_text(json.dumps({'files': entries, 'total_bytes': sum(e['bytes'] for e in entries)}, indent=2)+'\n',encoding='utf-8')
    print(f"Wrote inventory of {len(entries)} files ({sum(e['bytes'] for e in entries):,} bytes)")

if __name__ == '__main__': main()
