# Large model geometry — lossless split archive

The 45 native CAD/mesh/image assets (36,855,480 uncompressed bytes) are stored in a **lossless ZIP** split into 29 regular Git-tracked binary chunks, each at most 450,000 bytes. This is necessary because the connected GitHub writer could not upload the largest individual GLB/STL objects in one request. The original paths and SHA-256 values are preserved by the source inventory.

**Restoration (Python 3 standard library only):**

```bash
python scripts/restore_assets.py
python scripts/verify_study.py
```

Archive: `study-assets.zip` · size: 12,920,315 bytes · SHA-256: `e0b17306c806da70f88c714fa2494c867602da312fd2e3affaae8cfaa69b021c`. Individual part sizes/hashes are in `MANIFEST.json`. The restore script checks the chunk checksums, archive checksum, avoids path traversal and checks all restored files against `docs/STUDY_INVENTORY.json`.

The custom forearm STEP is additionally available directly as `cad/Case_mounted_forearm.step` for immediate browsing/download. The remaining STL/STEP/GLB/PNG files are available after asset restoration. The restored files are **data inputs**, not generated measurements of a physical SO-Duck.
