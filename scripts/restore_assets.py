"""Losslessly restore full published SO-Duck model geometry from Git-tracked chunks.

Run from anywhere with Python 3. Standard library only. Idempotent and safe against
unexpected ZIP paths and content checksums. Does not change the original model.
"""
from pathlib import Path
from hashlib import sha256
import io, json, zipfile

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT/'archives'

def die(message): raise RuntimeError(message)

def main():
    manifest=json.loads((ARCHIVE/'MANIFEST.json').read_text())
    data=bytearray()
    for part in manifest['parts']:
        p=ARCHIVE/part['filename']
        raw=p.read_bytes()
        if len(raw)!=part['bytes'] or sha256(raw).hexdigest()!=part['sha256']:
            die('chunk checksum mismatch: '+p.name)
        data.extend(raw)
    if len(data)!=manifest['archive_total_bytes'] or sha256(data).hexdigest()!=manifest['archive_sha256']:
        die('combined archive checksum mismatch')
    source_manifest=json.loads((ROOT/'docs/STUDY_INVENTORY.json').read_text())
    expected={entry['path']: entry for entry in source_manifest['files']}
    written=0
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        if z.testzip(): die('corrupt zip member')
        for item in z.infolist():
            if item.is_dir(): continue
            rel=item.filename
            if rel.startswith('/') or '\\' in rel or any(v in {'','.','..'} for v in rel.split('/')):
                die('unsafe archive path: '+rel)
            if rel not in expected: die('unrecognized binary asset path: '+rel)
            b=z.read(item)
            e=expected[rel]
            if len(b)!=e['bytes'] or sha256(b).hexdigest()!=e['sha256']:
                die('original asset checksum mismatch: '+rel)
            dest=ROOT/rel
            if dest.exists() and dest.read_bytes()==b: continue
            dest.parent.mkdir(parents=True,exist_ok=True)
            dest.write_bytes(b)
            written+=1
    print(f'Restored {written} changed geometry files, verified {len(z.namelist())} archive members.')

if __name__=='__main__': main()
