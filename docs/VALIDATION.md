# Validation claims and limits

- The CAD STEP and recorded HTML viewer are **design artifacts**, not evidence of printed fit or physical stiffness.
- The arm-only six-joint Jacobian is implemented in search/arm_model.py. Run search/check_jacobian.py to compare it with independent central differences. Singular values must always specify position scaling (default 150 mm) and chosen joint-rate weighting.
- The archived whole-body study reports sampled solutions subject to modeled two-foot support, static center-of-mass margin, and selected collision checks. It does not prove continuous collision-free motion, dynamic balance, or unique optimality.
- Finite-sample inverse-kinematics failures do not establish geometric impossibility. Conversely, accepted simulated endpoints do not establish hardware realizability.
- Servo torque/speed/thermal duty limits, cable routing, structural compliance, joint backlash, fastener pullout, payload/contact forces and walking transfer are **not verified**.
- Robot operation is not included in this initial public source snapshot. The original Duck walking policy cannot be assumed stable under a moving manipulator.

The GitHub Actions job is a syntax/kinematics smoke test only. It is **not** a MuJoCo, collision, fabrication or actuator certification test.
