# Results and interpretation

This page records the earlier longitudinal-roll design. The [current crosswise assembly](../crosswise/README.md) uses the stock SO-101 forearm.

**All figures below are saved computational observations from the study**, not real-hardware measurements. For generation commands, see [Reproducibility](REPRODUCIBILITY.md) and the original source.

## Servo orientation — sampled kinematic conditioning

Rows represent different mount/axis design hypotheses. Task-space linear velocity is divided by 150 mm, angular velocity left in rad/s, and servo rates are equally weighted.

| Layout | 5th-percentile minimum gain ↑ | 95th-percentile maximum gain ↓ | 95th-percentile condition number ↓ |
|---|---:|---:|---:|
| Stock upright forearm, both shafts transverse | 0.127 | 5.090 | 37.88 |
| Native terminal roll and transverse wrist, upright | 0.155 | 4.909 | 29.00 |
| **Native terminal roll and transverse wrist, forward-horizontal** | **0.162** | **4.602** | **22.86** |

The 90° forearm clock was selected for usable forward grasp pose coverage, not because it minimizes the condition number alone. An independently sampled 120° clock achieved 95th-percentile condition number 21.68, but lower neutral tool height. The full trade-space is recorded in `search/` CSV files.

## World-space finite target banks

Feasible means: converged to pose and support foot tolerance; nominal projected COM ≥5 mm within sole polygon; screened collision pairs have no surface intersection.

| Tested stance policy | Forward bank (64) | Broad orientation bank (32) |
|---|---:|---:|
| Original double-foot stance | 45 | 9 |
| Replant at 220 mm | 43 | 21 |
| Replant at 270 mm | 59 | 26 |
| Union of these three endpoint stances | **62** | **26** |
| Union with illustrative `s_min≥0.15`, `s_max≤4.5`, `κ≤25` | **61** | **26** |

Replants are independent endpoint configurations: the design does **not** include a control policy that safely reaches the other stances while holding a load. These counts depend on one chosen target bank, fixed joint limits, solver starting seeds, ideal contact and mass model. They are not an arbitrary-pose coverage percentage over continuous SE(3).

## Gravity and single support

Using an increased moving-link and bearing mass model, maximum static head-pitch gravity torque over 45 accepted original-stance forward grasps was 0.756 N·m empty and 0.929 N·m with 50 g at the tip. Wrist-flex peaks were 0.116 and 0.193 N·m. This is calculated gravity torque, not a validated holding capacity.

A specially selected left-foot-support pose was modeled with ~41° torso roll and 30° pitch, giving 5.47 mm unloaded and 3.24 mm with 50 g COM margin in the heavier mass model. It does not prove that the robot can transition to single support safely.

## Reproduction artifacts

- `search/*.csv`: joint-space orientation and stance comparisons, sensitivity and validation.
- `study/poses_*.csv`: per-target end-effector goals, success flags, final joint solutions, foot closure, COM margin and gain metrics.
- `collision/collision_poses_*.csv`: pass/fail, intersecting group pairs, and nonfoot ground results for selected study poses.
- `legs/gravity_O90.csv`, `legs/payload_*.csv`: modeled load and static balance sensitivity.

See [data dictionary](DATA_DICTIONARY.md) for filenames and measurement units and [validation](VALIDATION.md) for known limitations.
