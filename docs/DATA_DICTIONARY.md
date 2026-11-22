# CSV results and units

Saved CSV files are **experimental outputs**. The column table below is generated from the observed files by `scripts/study_inventory.py` and exists to avoid misinterpreting flags as continuous quantities.

| File group | Typical fields | Meaning |
|---|---|---|
| `search/arm_grid.csv`, `search/robot_grid.csv` | `azimuth,elevation,clock,two_part,smin05,smax95,k95` | Joint orientation sweep parameters and sampled Jacobian gain summaries |
| `search/*validation*.csv` | same as above | Held-out independent validation samples |
| `search/clock_refinement*.csv` | `clock`, `mode`, `length` | Finer forward-clock survey |
| `search/stance_sensitivity.csv` | `mode,length` | Different modeled support/normalization conditions |
| `study/poses_*.csv` | `target,bank,x_m,y_m,z_m,qw,qx,qy,qz,...` | Goal pose, success, joint solutions, foot closure and balance |
| `study/endpoint_summary.csv` | `placement,target,bank,screened,gain_bounded` | Combined endpoint and gain acceptance flags |
| `collision/collision_poses_*.csv` | `collision_checked,hit_count,intersecting_pairs,min_nonfoot_ground_clearance_mm` | Screened pose pair overlaps and floor clearance |
| `collision/contact_details.csv` | `scenario,target,bank,intersecting_mesh_components` | Nonempty pairs at an investigated pose |
| `legs/gravity_O90.csv` | `tau_*_Nm`, `payload_g` | Static modeled joint torques in N·m, payload in grams |
| `legs/payload_*.csv` | `baseline_mm`, `base_50g_mm`, `case_extra_mm`, `case_extra_50g_mm` | COM support polygon margin in mm |
| `legs/single_support_checked.csv` | `torso_*_deg`, `min_support_margin_m`, `collision_hit_count` | Particular single-foot support candidate |

**Conventions:** `x_m/y_m/z_m` are SI metres; quaternion `qw/qx/qy/qz` is a unit orientation; `*_mm` means millimetres; `*_rad` means radians; `tau_*_Nm` means newton-metres; `smin05` means the 5th percentile of sampled minimum singular values, not a 5% motor tolerance; `k95` means 95th percentile of sampled condition numbers. A zero `hit_count` applies only to tested rigid mesh pairs at the saved endpoint. `pose_and_feet_ok` is not a dynamic walking success flag.

All exact headers and types remain visible in the CSVs; `scripts/verify_study.py` checks required schema keys.
