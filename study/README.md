# Six-dimensional world-pose endpoint experiments

`pose_test.py` solves the joint configuration of the full Duck legs and hybrid neck/wrist for finite world-frame gripper targets. Saved `poses_*.csv` contain target pose, the solver output and flags for reaching, planted-foot closure and nominal COM support. They do **not** describe executed walking trajectories.

The `O90` design corresponds to the 90° forward-facing forearm, and `Replant22`/`Replant27` correspond to alternate assumed stance placements 220/270 mm from the target. Other orientations and `P*` samples are retained for study comparisons. `endpoint_summary.csv` consolidates acceptance against the collision and Jacobian thresholds.

**Reproduce:** run `python study/pose_test.py O90 --two --clock 90 --starts 24 --steps 100` from the root and inspect the input arguments in `pose_test.py`. The program may overwrite CSV outputs; regenerate in a separate working copy.
