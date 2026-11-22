# Arm-only Jacobian experiments

Install dependencies using: python -m pip install -r requirements-core.txt

Run: python search/check_jacobian.py
Run: python search/scan_arm.py --samples 8192 --output arm_only_results.csv

All scans share the same Sobol arm configurations and 150 mm translation scaling. **These scans fix the torso** and do NOT reproduce whole-body, planted-foot Jacobian numbers from docs/WHOLE_BODY_STUDY.md.

The geometric frames are derived from donor MJCF body transforms. The independent_axes flag means an independently oriented downstream forearm can retain the Duck terminal roll axis while tipping the SO-101 wrist flex axis. It does **not** mean a physically confirmed assembly, nor does it assert two custom parts.
