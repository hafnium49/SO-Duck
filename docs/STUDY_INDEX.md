# Full study index

The original full study archive contained **102 files, 39,611,727 uncompressed bytes**. This public repository reorganizes the same scientific source tree, provides original binary geometry via lossless, reconstructable chunks in `archives/` rather than leaving missing dependencies, and adds method/validation and integrity documentation. Source snapshots are represented with versioned checksums in `docs/SHA256SUMS.txt`.

| Path | Purpose | Read first |
|---|---|---|
| `study/legacy_pose_explorer.html` | Standalone viewer for saved poses | Browser |
| `cad/` | Forearm design, native STEP, simulation GLB and printed-mesh derivative | `cad/README.md`, `cad/Mount_fit.md` |
| `search/` | Orientation and joint-space sampling, robot Jacobian including leg constraints | `search/README.md` |
| `legs/` | Contact, inertial and static torque model | `legs/README.md` |
| `study/` | Numerical whole-body endpoint grasp and replanted stance experiments | `study/README.md` |
| `collision/` | Native triangle-mesh collision inputs, BVH and VTK checker, filtered endpoints | `collision/README.md` |
| `docs/` | Reproduction, provenance, results, assumptions and machine-checkable inventory | `REPRODUCIBILITY.md` |

A 39 MB repository can be versioned directly without Git LFS: the largest single saved asset in this study is about 5.1 MB, below GitHub's ordinary file-size limit. Binary `.stl`, `.step`, `.glb` and image assets are kept as exact file bytes; tests compare SHA-256 digests.
