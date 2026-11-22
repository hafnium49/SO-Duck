# Closest public precedents (search completed November 22, 2026)

## Yo.Town Gym: https://github.com/yotown/gym
Supplies *separate* parametric CadQuery representations of both Open Duck Mini V2 and SO-101 plus a Flamingo variant (longer legs, neck, wings and bird-shaped head). Flamingo has a simulated marching policy but no documented gripper or full 6D manipulator. Do not confuse its finite simulated march with physical validation: the project itself notes large knee-torque demands.

## robot_jarvis: https://github.com/matte1782/robot_jarvis
Proposes two 2-DoF gripper arms on the Duck's **torso sides**. The published guide is an engineering proposal rather than verified working hardware. The proposed STL filenames were absent from inspected custom CAD directories; some electromechanical claims conflicted with the Duck's serial-servo architecture. The solution differs from SO-Duck's shared neck/manipulation chain.

## ToddlerBot: https://github.com/hshi74/toddlerbot
A different, physically demonstrated open humanoid platform showing the value of studying whole-body manipulation and balance jointly. Much more complex than SO-Duck.

**Scope of novelty claim**: No exact public Open Duck V2 neck plus SO-101 terminal-wrist and gripper integration was verified in the examined repositories. That is not proof that no other implementation exists.
