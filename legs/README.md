# Leg kinematics, rigid contact and gravity

Uses the original Duck V2 leg link and motor model (`duck.xml`) and full sole outline (`sole.stl`). `legs.py` provides constrained leg/body transforms and contact kinematics, `balance.py` computes simple support-polygon margins, and `torque.py` estimates gravitational joint loads. All models are idealized, with no learned gait controller, no soft-ground contact, and no thermal motor duty cycle.

The CSVs record selected nominal and 50 g tool payload cases, including left-single-support sensitivity. The alternate foot placements in `study/` are endpoints, not physically proven transitions. See `docs/METHODS.md` and `docs/VALIDATION.md`.
