# Buildable Hollow Arm Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the three custom SO-Duck arm parts from the SO-101 STEP solids, shortened so the shoulder servo carries no more than a stock SO-101 arm, and deliver printable files, a matching model, re-checked postures and assembly documents.

**Architecture:** One parameter file drives everything. A load script fixes the required lengths before any CAD. CadQuery builders cut SO-101 donor solids and join them into the three parts. A build script exports parts and regenerates `hollow/robot.xml`. The existing posture search, replay and viewer checks then run on the real geometry.

**Tech Stack:** Python 3.12, CadQuery 2.7, MuJoCo 3.17, python-fcl, VTK helper (`collision/bvh.so`), NumPy, SciPy, trimesh, pytest.

**Spec:** `docs/superpowers/specs/2026-11-25-buildable-hollow-arm-design.md`

## Global Constraints

- Exactly three custom parts. If a stock part fails to match its SO-101 donor, stop and report; no fourth part.
- STS3215 at 7.4 V on every joint. No motor change.
- The mount uses the Duck's original four neck screws and nothing else on the trunk. No stock part is drilled, cut or glued. The roof parking pad is the one existing exception and may move.
- New joining geometry is an open channel or shell with walls like the donor's. No solid bars or tubes. Wall thickness nowhere below 1.6 mm.
- At least 1.5 mm between parts that move relative to each other, over the joint's range.
- Every part fits a 180 x 180 mm bed and prints in PLA or PETG without soluble supports.
- Custom part mass is full solid volume at 1.24 g/cm³. Servo 55 g. Payload 100 g at the `gripperframe` site.
- Load limit is the stock SO-101 baseline computed by `hollow/loads.py`, not a typed-in number.
- Coverage gate: at least 85 of 96 stored postures pass, searched at `--starts 128` and then `--starts 2048` for open targets.
- Leg gap rule unchanged: 0.3 mm hip link, 1 mm thigh, shin and foot. Support margin at least 5 mm.
- Stop rules: required lengths do not pack; goal 3 or the coverage gate cannot be met; a stock part does not match. In each case stop, report the numbers, and wait for the owner.
- Documents say "designed and checked in CAD and simulation, not yet printed". No claim of strength or fit.
- Apache-2.0 notices stay with derived geometry. `docs/PROVENANCE.md` names SO-ARM100 commit `141aea7bcac2744d92998e30268c413202a56f96`.
- The three review meshes and `hollow/meshes/` stay in the tree. Removing them needs the owner's acceptance of the new parts.
- Before every commit run `python scripts/study_inventory.py` and include `docs/SHA256SUMS.txt` and `docs/STUDY_INVENTORY.json`. Delete `__pycache__` and `.pytest_cache` first.
- Commands run from the repository root after `python -m pip install -r crosswise/requirements.txt`, `python scripts/restore_assets.py` and `python collision/build_bvh.py`, with `OPENBLAS_NUM_THREADS=1`.
- Commits use author `hafnium49 <hafnium49@users.noreply.github.com>` and end with the line `Co-Authored-By: Claude Fable 5.2 <noreply@anthropic.com>`. Work on branch `design/buildable-hollow-arm`. Do not push or merge without the owner's word.

## Review Focus

1. **A servo that cannot be inserted.** A part clears its servo when assembled, but the servo cannot slide into the band. Expected: each servo can travel 40 mm out along its insertion axis without touching the part. Test in Tasks 5 to 7.
2. **A blocked cable.** The band or bridge covers a servo's connectors. Expected: both connector windows of every held servo are open. Test in Tasks 5 to 7.
3. **A sliver left by cutting a donor.** A cut plane leaves a wall thinner than the nozzle can print. Expected: minimum wall 1.6 mm on every part. Test in Tasks 5 to 7.
4. **Model ranges wider than the parts allow.** A stored posture sits where real parts collide. Expected: model joint ranges come from sweeps of the real meshes, and stored postures stay 2 degrees inside them. Test in Task 8.
5. **A horn mounted one spline tooth off.** The STS3215 horn has 25 teeth, so one tooth is 14.4 degrees. Expected: `hollow/BUILD.md` gives a zero-pose jig check for every joint. Test in Task 12 checks the guide names all seven joints.

---

## File Structure

| Path | Responsibility |
| --- | --- |
| `hollow/cad/params.py` | Every dimension, clearance, mass rule and chosen length. No logic beyond `joint_frames()`. |
| `hollow/cad/donors/` | Seven pinned SO-101 STEP files, `SHA256SUMS`, `README.md` with commit and licence. |
| `hollow/loads.py` | Stock baseline and arm gravity torques. |
| `hollow/cad/stock.py` | Places each stock SO-101 solid and each servo in its model body frame; writes `placements.json`. |
| `hollow/cad/assemble.py` | Writes an MJCF from params, masses and mesh files. |
| `hollow/cad/lengths.py` | Joints-only study that picks the shortening. Writes `length_study.csv`. |
| `hollow/cad/checks.py` | Geometry checks shared by tests: wall, clearance, insertion, holes, joint sweep. |
| `hollow/cad/parts.py` | `forearm_link()`, `wrist_link()`, `neck_mount()`. |
| `hollow/cad/build.py` | Exports `hollow/parts/*.step` and `.stl`, stock meshes, and `hollow/robot.xml`. |
| `hollow/postures.py` (modify) | Lowest-load selection; overhead, parked and fold clips. |
| `hollow/viewer_build.py` | Puts the new meshes and cutaways into `hollow/viewer.html`. |
| `hollow/inspect_viewer.py` (modify) | Reads masses from the model; compares viewer meshes with `hollow/parts`. |
| `hollow/BUILD.md` | Parts and fasteners, printing, assembly. |
| `hollow/tests/` | pytest files, one per module above. |

Scripts in this repository import siblings by path, not as packages. `hollow/tests/conftest.py` puts `hollow`, `hollow/cad`, `coverage`, `crosswise` and `study` on `sys.path`. Lengths are metres in models and millimetres in CAD, as elsewhere in the repository.

---

### Task 1: Donors, parameters and test scaffold

**Files:**
- Create: `hollow/cad/donors/{Under_arm_SO101,Upper_arm_SO101,Rotation_Pitch_SO101,Wrist_Roll_Pitch_SO101,Motor_holder_SO101_Wrist,Motor_holder_SO101_Base,Base_motor_holder_SO101}.step`, `hollow/cad/donors/SHA256SUMS`, `hollow/cad/donors/README.md`, `hollow/cad/params.py`, `hollow/tests/conftest.py`, `hollow/tests/test_params.py`
- Modify: `crosswise/requirements.txt` (add `pytest`, `trimesh`), `scripts/study_inventory.py` (skip any path with a `.pytest_cache` part), `.gitignore` (`.pytest_cache/`), `docs/PROVENANCE.md`

**Interfaces:**
- Produces: `params.DONOR_COMMIT: str`; `params.FRAME: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]]` keyed by the six `replay.JOINTS` names plus `"gripper"`, holding each joint body's position (m) and quaternion (w, x, y, z) in its parent, copied from `hollow/robot.xml`; `params.FOREARM_SHORTEN`, `params.WRIST_SIDE_SHORTEN`, `params.WRIST_AXIAL_SHORTEN` (m, all `0.0` for now); `params.FOREARM_RANGE = (0.040, 0.055)`, `params.WRIST_SIDE_RANGE = (0.040, 0.050)`, `params.WRIST_AXIAL_MAX = 0.012`; `params.MOVING_CLEARANCE_MM = 1.5`, `params.MIN_WALL_MM = 1.6`, `params.BED_MM = 180.0`, `params.DENSITY = 1240.0`, `params.SERVO_KG = 0.055`, `params.PAYLOAD_KG = 0.1`, `params.LOAD_SAMPLES = 20000`; `params.joint_frames(forearm=None, wrist_side=None, wrist_axial=None) -> dict[str, tuple[np.ndarray, np.ndarray]]`, which applies the shortening (arguments default to the module constants); `params.donor(name: str) -> cq.Solid` in millimetres.

- [ ] **Step 1: Fetch the donors**

```bash
mkdir -p hollow/cad/donors && cd hollow/cad/donors
for f in Under_arm_SO101 Upper_arm_SO101 Rotation_Pitch_SO101 Wrist_Roll_Pitch_SO101 Motor_holder_SO101_Wrist Motor_holder_SO101_Base Base_motor_holder_SO101; do
  curl -sSfL -o $f.step https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/141aea7bcac2744d92998e30268c413202a56f96/STEP/SO101/$f.step
done
sha256sum *.step > SHA256SUMS
```

Expected: seven files, 3.9 MB in total.

- [ ] **Step 2: Write the failing tests** in `hollow/tests/test_params.py`

```python
VOLUME_CM3 = {"Under_arm_SO101": 93.0, "Upper_arm_SO101": 117.4, "Rotation_Pitch_SO101": 68.8, "Wrist_Roll_Pitch_SO101": 32.3,
              "Motor_holder_SO101_Wrist": 13.9, "Motor_holder_SO101_Base": 13.3, "Base_motor_holder_SO101": 23.7}

def test_donor_checksums():            # every line of SHA256SUMS matches its file
@pytest.mark.parametrize("name", VOLUME_CM3)
def test_donor_is_one_valid_solid(name):
    solid = params.donor(name)
    assert solid.isValid() and len(solid.Solids()) == 1
    assert abs(solid.Volume() / 1000 - VOLUME_CM3[name]) < 0.1

def test_frames_match_model():         # zero shortening reproduces body pos and quat in hollow/robot.xml to 1e-9
def test_shortening_moves_two_joints():
    base, short = params.joint_frames(0, 0, 0), params.joint_frames(0.040, 0.040, 0.012)
    assert np.allclose(short["forearm_roll"][0] - base["forearm_roll"][0], [-0.040, 0, 0])
    assert np.allclose(short["wrist_flex"][0] - base["wrist_flex"][0], [0.040, 0, -0.012])
    assert all(np.allclose(short[j][0], base[j][0]) for j in base if j not in ("forearm_roll", "wrist_flex"))
```

- [ ] **Step 3: Run** `python -m pytest hollow/tests/test_params.py -q`. Expected: FAIL, `params` not found.
- [ ] **Step 4: Implement** `params.py`, `conftest.py`, the donor `README.md` (commit, licence, file list) and the three small modifications. The body order for `FRAME` is the chain in `hollow/model.py` `BODIES`.
- [ ] **Step 5: Run** the same command. Expected: 10 passed.
- [ ] **Step 6: Commit** "Add the SO-101 donor solids and the parameter file".

---

### Task 2: Load calculation

**Files:**
- Create: `hollow/loads.py`, `hollow/tests/test_loads.py`

**Interfaces:**
- Consumes: `params.LOAD_SAMPLES`, `params.PAYLOAD_KG`; `model.robot_model()`; `postures.read_targets()`, `postures.UNREACHED`.
- Produces: `loads.stock_baseline(samples: int = params.LOAD_SAMPLES, seed: int = 0) -> tuple[float, float]`, the worst shoulder-lift gravity torque of `collision/inputs/so101.xml` in N m, empty and with the payload; `loads.arm_torques(robot, q: np.ndarray, payload_kg: float = 0.0) -> np.ndarray` of shape (n, 6), absolute gravity torque at the six arm joints for postures `q` of shape (n, 16); `loads.SHOULDER = 1` (column of shoulder pitch); running `python hollow/loads.py` prints the baseline and, for the passing rows of `hollow/targets.csv`, the worst and median shoulder torque, empty and loaded.

- [ ] **Step 1: Write the failing tests**

```python
def test_stock_baseline():
    empty, loaded = loads.stock_baseline()
    assert 0.84 < empty < 0.88 and 1.24 < loaded < 1.29      # spike: 0.863 and 1.265

def test_present_arm_is_over_the_limit(robot, passing_q):     # fixtures in conftest.py
    empty = loads.arm_torques(robot, passing_q)[:, loads.SHOULDER].max()
    assert 1.10 < empty < 1.25 and empty > loads.stock_baseline()[0]

def test_payload_never_lowers_the_worst_case(robot, passing_q):
    assert loads.arm_torques(robot, passing_q, 0.1)[:, loads.SHOULDER].max() > loads.arm_torques(robot, passing_q)[:, loads.SHOULDER].max()
```

- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_loads.py -q`. Expected: FAIL, `loads` not found.
- [ ] **Step 3: Implement.** Baseline: parse `so101.xml`, drop every `geom`, `asset`, `default`, `actuator`, `contact` and `sensor` element and every `class` and `childclass` attribute, load the string, sample joint angles uniformly in range. Arm: place each posture with `robot.placements`, disable contacts (`mjDSBL_CONTACT`), call `mj_forward`, read `qfrc_bias` at the arm joints, and add `J_site^T [0, 0, m g]` for the payload.
- [ ] **Step 4: Run** the tests. Expected: 3 passed.
- [ ] **Step 5: Commit** "Compute arm loads and the stock SO-101 baseline".

---

### Task 3: Place the stock parts and servos

**Files:**
- Create: `hollow/cad/stock.py`, `hollow/cad/placements.json`, `hollow/tests/test_stock.py`

**Interfaces:**
- Consumes: `params.donor`; display meshes `hollow/meshes/<name>.stl` (millimetres, body frame).
- Produces: `stock.STOCK = {"shoulder_pitch_link": "Rotation_Pitch_SO101", "shoulder_collar": "Motor_holder_SO101_Base", "forearm_pitch_link": "Upper_arm_SO101", "wrist_roll_link": "Wrist_Roll_Pitch_SO101"}`; `stock.SERVOS = ("shoulder_yaw_case", "shoulder_pitch_case", "forearm_pitch_case", "forearm_roll_case", "wrist_flex_case", "wrist_roll_case")`, each fitted with `collision/inputs/arm_meshes/sts3215_03a_v1.stl` (`wrist_roll_case` with `sts3215_03a_no_horn_v1.stl`); `stock.register(name: str) -> tuple[np.ndarray, float, float]` returning the 4x4 transform from donor millimetres to body-frame millimetres, the RMS distance in mm, and the share of display vertices within 0.25 mm; `stock.placement(name: str) -> np.ndarray` read from `placements.json`; `stock.solid(name: str) -> cq.Solid` (stock links, placed, mm); `stock.mesh(name: str) -> trimesh.Trimesh` (links and servos, placed, mm); `stock.SERVO_AXIS` and `stock.SERVO_CONNECTORS`: in the servo mesh's own frame, the horn axis, the insertion direction, and two 9 x 5 x 12 mm connector boxes, measured once from the servo mesh and written as constants.

- [ ] **Step 1: Write the failing test**

```python
@pytest.mark.parametrize("name", [*stock.STOCK, *stock.SERVOS])
def test_stock_part_matches_its_display_mesh(name):
    transform, rms, share = stock.register(name)
    assert rms < 0.15 and share > 0.95
    assert np.allclose(transform[:3, :3] @ transform[:3, :3].T, np.eye(3), atol=1e-9)
    assert np.allclose(transform, stock.placement(name), atol=1e-6)
```

- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_stock.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement.** Fit by trimmed point-to-point ICP with a KD-tree on 200,000 points sampled from the donor surface, started from each of the 24 axis-aligned rotations, keeping the best. Write the result to `placements.json`.
- [ ] **Step 4: Run** the test. Expected: 10 passed. **If any part fails, stop and report its name, RMS and share. Do not continue to Task 4.**
- [ ] **Step 5: Commit** "Place the stock SO-101 parts and servos in the model frames".

---

### Task 4: Choose the lengths from load, before CAD

**Files:**
- Create: `hollow/cad/assemble.py`, `hollow/cad/lengths.py`, `hollow/cad/length_study.csv`, `hollow/tests/test_lengths.py`
- Modify: `coverage/resolve.py` (`choose` gains `rank`), `hollow/cad/params.py` (the three chosen values)

**Interfaces:**
- Consumes: `params.joint_frames`, `loads.arm_torques`, `loads.stock_baseline`, `resolve.candidates`, `resolve.passes`, `replay.leg_clearance`.
- Produces: `resolve.choose(robot, pairs, found, positions, rotations, block=8, depth=240, rank=None)`, where `rank(q: np.ndarray) -> np.ndarray` gives a cost per candidate, lowest tried first, and `None` keeps today's order of falling support margin; `assemble.write_model(path: Path, frames: dict, masses: dict[str, tuple[float, np.ndarray]], meshes: dict[str, list[tuple[str, str]]]) -> None`, where `masses` maps each model body to mass (kg) and centre (m) and `meshes` maps each model body to (geom name, mesh file relative to `path`) pairs; `assemble.estimated_masses(frames) -> dict` using `inspect_viewer.arm_masses` with each custom part's centre moved with its joint; `lengths.evaluate(forearm: float, wrist_side: float, wrist_axial: float, starts: int) -> dict` with keys `covered`, `shoulder_empty`, `shoulder_loaded`; `lengths.smallest_passing(rows: list[dict], limit: tuple[float, float]) -> dict | None`.

- [ ] **Step 1: Write the failing tests**

```python
def test_smallest_passing_prefers_less_shortening():
    rows = [dict(forearm=.040, wrist_side=.040, shoulder_empty=.91, shoulder_loaded=1.29, covered=91),
            dict(forearm=.045, wrist_side=.045, shoulder_empty=.85, shoulder_loaded=1.20, covered=90),
            dict(forearm=.055, wrist_side=.050, shoulder_empty=.82, shoulder_loaded=1.15, covered=89)]
    assert lengths.smallest_passing(rows, (0.86, 1.27))["forearm"] == .045
    assert lengths.smallest_passing(rows[:1], (0.86, 1.27)) is None

def test_rank_orders_the_candidates():   # choose() with rank=lambda q: q[:, 0] returns, for a target with two passing candidates, the one with the smaller q[0]
def test_chosen_lengths_are_recorded_and_pass():
    row = next(r for r in csv.DictReader(open("hollow/cad/length_study.csv")) if r["chosen"] == "1")
    assert float(row["forearm"]) == params.FOREARM_SHORTEN and params.FOREARM_RANGE[0] <= params.FOREARM_SHORTEN <= params.FOREARM_RANGE[1]
    assert float(row["wrist_side"]) == params.WRIST_SIDE_SHORTEN and params.WRIST_SIDE_RANGE[0] <= params.WRIST_SIDE_SHORTEN <= params.WRIST_SIDE_RANGE[1]
    empty, loaded = loads.stock_baseline()
    assert float(row["shoulder_empty"]) <= empty and float(row["shoulder_loaded"]) <= loaded and int(row["covered"]) >= 85
```

- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_lengths.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement.** `evaluate` writes a model with `assemble.write_model` (joint frames from params, estimated masses, the existing display meshes), searches all 96 targets with `resolve.candidates(robot, frames, [], ...)`, keeps candidates passing `resolve.passes(..., full=False)` and the leg gap, and takes for each target the one with the lowest loaded shoulder torque. A second pass at 1024 starts covers targets the first missed. `python hollow/cad/lengths.py` evaluates forearm 40, 45, 50 and 55 mm against wrist side 40, 45 and 50 mm with wrist axial 12 mm, in order of rising total shortening, writes every row, marks the first that meets both limits with `covered >= 85`, and stops there. "Smallest" means lowest `forearm + wrist_side`, ties to the lower `forearm`.
- [ ] **Step 4: Run the study** with `python hollow/cad/lengths.py --starts 128`, then copy the chosen values into `params.py`. Expected: a chosen row. The spike found 40/40 just over the limit and 55/50 under it. **If no row passes, stop and report the table.**
- [ ] **Step 5: Run** the tests. Expected: 3 passed.
- [ ] **Step 6: Commit** "Choose the forearm and wrist lengths from the load limit".

---

### Task 5: Elbow-to-forearm-roll link

**Files:**
- Create: `hollow/cad/checks.py`, `hollow/cad/parts.py`, `hollow/tests/test_checks.py`, `hollow/tests/test_forearm_link.py`

**Interfaces:**
- Consumes: `params`, `stock.solid`, `stock.mesh`, `stock.placement`, `stock.SERVO_AXIS`, `stock.SERVO_CONNECTORS`.
- Produces: `parts.forearm_link() -> cq.Solid`, millimetres, in the frame of model body `forearm_pitch_linkage` (origin on the elbow axis, z along it); `parts.HORN_HOLES_MM = [(-4.95, -4.95), (-4.95, 4.95), (4.95, -4.95), (4.95, 4.95)]`, the STS3215 horn's tapped holes about its axis; and in `checks.py`: `tessellate(solid, tolerance=0.02) -> trimesh.Trimesh`; `min_wall_mm(solid) -> float` (shortest inward ray from 20,000 surface samples, 1st percentile); `clearance_mm(a, b, skip=None) -> float` (smallest distance between two meshes, ignoring triangles for which `skip(points) -> bool array` is true; 0 if they intersect); `servo_overlap_mm3(part: cq.Solid, servo: str, shift=(0, 0, 0)) -> float` (volume the part shares with the servo's bounding solid shrunk by 0.15 mm, the servo moved by `shift` mm in its own frame); `hole_is_open(part: cq.Solid, centre, axis, radius=1.4, depth=8.0) -> bool`; `fits_bed(solid) -> bool` (two of the three extents at most `params.BED_MM` after the best axis-aligned placement).

- [ ] **Step 1: Write the failing check tests** in `test_checks.py`: a 20 mm cube with a 16 mm cubic pocket has `min_wall_mm` 2.0 within 0.1; two boxes 3 mm apart have `clearance_mm` 3.0 within 0.05 and 0 when overlapping; a plate with a 3.2 mm hole gives `hole_is_open` true at the hole and false 5 mm away.
- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_checks.py -q`. Expected: FAIL. Implement `checks.py`. Run again. Expected: PASS.
- [ ] **Step 3: Write the failing part tests** in `test_forearm_link.py`

```python
def test_one_printable_solid(link):
    assert link.isValid() and len(link.Solids()) == 1 and checks.tessellate(link).is_watertight
    assert checks.min_wall_mm(link) >= params.MIN_WALL_MM and checks.fits_bed(link)

def test_roll_servo_sits_at_the_chosen_length(link):
    # the roll servo, moved back by the chosen shortening, is held without overlap
    assert checks.servo_overlap_mm3(link, "forearm_roll_case") < 1.0

def test_roll_servo_can_be_inserted(link):
    for out in range(2, 42, 2):   # mm along stock.SERVO_AXIS insertion direction
        assert checks.servo_overlap_mm3(link, "forearm_roll_case", shift=insertion(out)) < 1.0

def test_connectors_are_open(link):          # both stock.SERVO_CONNECTORS boxes of the roll servo share < 1 mm3 with the part
def test_cheeks_take_the_elbow_horn_screws(link):   # hole_is_open at each of parts.HORN_HOLES_MM on both cheeks, axis = elbow axis
def test_elbow_swing_keeps_clearance(link):
    near_horn = lambda p: np.hypot(p[:, 0], p[:, 1]) < 13.0
    for angle in range(-95, 96, 5):          # the upper arm and elbow servo turned about the elbow axis
        assert checks.clearance_mm(checks.tessellate(link), upper_arm_at(angle), skip=near_horn) >= params.MOVING_CLEARANCE_MM
```

- [ ] **Step 4: Run** `python -m pytest hollow/tests/test_forearm_link.py -q`. Expected: FAIL, `forearm_link` not found.
- [ ] **Step 5: Implement `forearm_link()`.** Cut the elbow-end cheeks and the channel section from `Under_arm_SO101`, cut the motor band from `Motor_holder_SO101_Wrist`, place the band around the roll servo at its placement moved by `-FOREARM_SHORTEN` along x, and join cheeks to band with a channel of the under-arm's wall thickness, open on the side the servo enters. Where 95 degrees of elbow swing cannot keep 1.5 mm, narrow the range the test sweeps and record it as `params.ELBOW_RANGE_DEG`; Task 8 measures the final range.
- [ ] **Step 6: Run** the tests. Expected: 6 passed. **If the chosen length cannot pack, stop and report the longest shortening that does and its load from `lengths.evaluate`.**
- [ ] **Step 7: Commit** "Draw the elbow-to-forearm-roll link from SO-101 solids".

---

### Task 6: Forearm-roll-to-wrist link

**Files:**
- Modify: `hollow/cad/parts.py`
- Create: `hollow/tests/test_wrist_link.py`

**Interfaces:**
- Consumes: as Task 5.
- Produces: `parts.wrist_link() -> cq.Solid`, millimetres, in the frame of model body `forearm_roll_linkage` (origin on the roll servo's horn, z along the roll axis).

- [ ] **Step 1: Write the failing tests**, mirroring Task 5: `test_one_printable_solid`; `test_wrist_servo_sits_at_the_chosen_offset` (`wrist_flex_case` moved by `+WRIST_SIDE_SHORTEN` in x and `-WRIST_AXIAL_SHORTEN` in z, overlap under 1 mm³); `test_wrist_servo_can_be_inserted`; `test_connectors_are_open`; `test_seat_takes_the_roll_horn_screws` (four `HORN_HOLES_MM` about the body z axis); `test_roll_turn_keeps_clearance` (the forearm link and roll servo turned about z from -155 to 155 in steps of 5, skipping points within 13 mm of the axis).
- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_wrist_link.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement `wrist_link()`.** A horn seat on the roll servo's horn, the `Motor_holder_SO101_Wrist` solid around the wrist-flex servo, and a channel bridge between them with paired webs from the horn screws into the holder.
- [ ] **Step 4: Run** the tests. Expected: 6 passed. Same stop rule as Task 5.
- [ ] **Step 5: Commit** "Draw the forearm-roll-to-wrist link from SO-101 solids".

---

### Task 7: Neck socket mount

**Files:**
- Modify: `hollow/cad/parts.py`, `hollow/cad/params.py` (`NECK_SCREWS_MM`, `NECK_SCREW_AXIS`)
- Create: `hollow/tests/test_neck_mount.py`

**Interfaces:**
- Consumes: as Task 5, plus the trunk meshes of `coverage/robot.xml` and `hollow/review-meshes/neck-socket-mount.preview.stl`.
- Produces: `params.NECK_SCREWS_MM: list[tuple[float, float, float]]`, four hole centres in the trunk frame; `params.NECK_SCREW_AXIS: tuple[float, float, float]`; `parts.neck_mount() -> cq.Solid`, millimetres, trunk frame.

- [ ] **Step 1: Measure the four neck screws.** Find the four holes in the trunk meshes where the Duck's neck assembly was screwed on, and the matching holes in the review mount mesh. Write the centres and axis into `params.py`.
- [ ] **Step 2: Write the failing tests**

```python
def test_recorded_screws_are_real_holes():
    assert len(params.NECK_SCREWS_MM) == 4
    for centre in params.NECK_SCREWS_MM:     # a 1.0 mm probe along the axis hits neither the trunk meshes nor the review mount mesh
        assert probe_is_clear(trunk_mesh, centre) and probe_is_clear(review_mount_mesh, centre)

def test_one_printable_solid(mount):         # as Task 5
def test_flange_takes_the_four_neck_screws(mount):   # hole_is_open at each NECK_SCREWS_MM along NECK_SCREW_AXIS
def test_yaw_servo_is_held_where_the_model_has_it(mount):   # servo_overlap_mm3(mount, "shoulder_yaw_case") < 1.0
def test_yaw_servo_can_be_inserted(mount):
def test_connectors_are_open(mount):
def test_mount_clears_the_trunk(mount):      # shares no volume with the trunk meshes beyond the flange's contact face
def test_yaw_turn_keeps_clearance(mount):    # the shoulder stage turned about the yaw axis from -108 to 108 in steps of 6, skipping points within 13 mm of the axis
```

- [ ] **Step 3: Run** `python -m pytest hollow/tests/test_neck_mount.py -q`. Expected: FAIL.
- [ ] **Step 4: Implement `neck_mount()`.** The `Base_motor_holder_SO101` profile around the yaw servo at its placement, a flange on the four neck screws, and an open channel upright between them.
- [ ] **Step 5: Run** the tests. Expected: 8 passed.
- [ ] **Step 6: Commit** "Draw the neck socket mount from SO-101 solids".

---

### Task 8: Build the parts and the model; measure joint ranges

**Files:**
- Create: `hollow/cad/build.py`, `hollow/parts/{neck_socket_mount,elbow_to_forearm_roll,forearm_roll_to_wrist}.{step,stl}`, `hollow/parts/stock/*.stl`, `hollow/tests/test_build.py`
- Modify: `hollow/cad/checks.py` (`joint_range`), `hollow/cad/params.py` (`RANGES_DEG`), `hollow/robot.xml` (regenerated), `hollow/model.py` (pairs for the new meshes)

**Interfaces:**
- Consumes: `parts.*`, `stock.*`, `assemble.write_model`, `params`.
- Produces: `python hollow/cad/build.py` writes the six part files, a tessellated STL of each stock link and servo under `hollow/parts/stock/`, and `hollow/robot.xml`; `python hollow/cad/build.py --check` rebuilds into a temporary directory and exits non-zero unless each part's volume and bounding box equal the committed STEP's within 0.001 cm³ and 0.01 mm; `checks.joint_range(robot, joint: str, step_deg: float = 1.0) -> tuple[float, float]`, the widest interval around zero in which every tested pair that the joint separates stays apart; `params.RANGES_DEG: dict[str, tuple[float, float]]` for the six arm joints; `model.tested_pairs(robot)` with the `"crossing"` rule kept only for the four `SEATS` pairs and the triangle test used for every other pair.

- [ ] **Step 1: Write the failing tests**

```python
def test_committed_parts_are_current():      # build.py --check exits 0
def test_part_masses_are_solid_volume():     # each custom geom's body mass contribution equals STEP volume * 1.24e-6 kg within 1e-6
def test_zero_pose_has_no_intersections(robot):
    frames, _ = robot.placements(np.zeros((1, 7)))
    assert not any(replay.pair_hits(robot, pair, frames).any() for pair in model.tested_pairs(robot))

def test_model_ranges_come_from_sweeps(robot):
    for joint, (low, high) in params.RANGES_DEG.items():
        measured = checks.joint_range(robot, joint)
        assert measured[0] <= low < 0 < high <= measured[1]
        assert np.allclose(np.rad2deg(robot.model.joint(joint).range), (low, high), atol=1e-6)
```

- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_build.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** `build.py` and `joint_range`, run the build, set `RANGES_DEG` to the measured ranges rounded inward to whole degrees, and run the build again so the model carries them.
- [ ] **Step 4: Run** `python -m pytest hollow/tests -q`. Expected: all pass.
- [ ] **Step 5: Commit** "Build the three parts and regenerate the model from them".

---

### Task 9: Search the 96 targets on the real model

**Files:**
- Modify: `hollow/postures.py`, `hollow/targets.csv`
- Create: `hollow/tests/test_targets.py`

**Interfaces:**
- Consumes: `resolve.search` and `resolve.choose(rank=...)`, `loads.arm_torques`, `loads.stock_baseline`, `model.tested_pairs`.
- Produces: `python hollow/postures.py --search [--starts N]` discards every stored row (the geometry changed), searches all 96 targets and stores, for each, the passing posture with the lowest loaded shoulder torque; `postures.UNREACHED` is the tuple of targets that end with no passing posture; `targets.csv` keeps its columns, and `source` is `"re-solved"` on every row.

- [ ] **Step 1: Write the failing tests**

```python
def test_coverage_gate(robot):
    q, _ = postures.read_targets()
    ok, _ = resolve.passes(robot, model.tested_pairs(robot), q, np.arange(96), *targets())
    assert ok.sum() >= 85 and sorted(np.flatnonzero(~ok)) == sorted(postures.UNREACHED)

def test_stored_postures_meet_the_load_limit(robot, passing_q):
    empty, loaded = loads.stock_baseline()
    assert loads.arm_torques(robot, passing_q)[:, loads.SHOULDER].max() <= empty
    assert loads.arm_torques(robot, passing_q, params.PAYLOAD_KG)[:, loads.SHOULDER].max() <= loaded
```

- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_targets.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** the ranked search, run `python hollow/postures.py --search --starts 128`, then `--starts 2048` (it searches only rows that fail), and set `UNREACHED`.
- [ ] **Step 4: Run** the tests. Expected: 2 passed. **If either fails, stop and report the count, the failing targets and the loads.**
- [ ] **Step 5: Commit** "Search the 96 targets on the rebuilt arm".

---

### Task 10: Pickup, overhead, parked and fold postures

**Files:**
- Modify: `hollow/postures.py`, `hollow/pickup.csv`, `hollow/cad/params.py` (`PAD_XY_MM`, `CUBE_XY_MM`)
- Create: `hollow/overhead.csv`, `hollow/park.csv`, `hollow/fold_waypoint.csv`, `hollow/tests/test_clips.py`

**Interfaces:**
- Consumes: `postures.fixed_legs`, `postures.search_pickup`, `resolve.converge`, `motion.between` (in `crosswise/motion.py`).
- Produces: `params.PAD_XY_MM` (today `(-104.0, 23.0)`) and `params.CUBE_XY_MM` (today `(182.1, -0.1)`); `python hollow/postures.py --make-clips` writes the three one-row CSV files of six arm angles in radians; `postures.fold_path() -> np.ndarray` of shape (n, 17), overhead to waypoint to parked in steps of at most 0.5 degrees per joint at the reference stance; `postures.overhead_height_mm() -> float`; the default run of `postures.py` replays these clips from the CSV files instead of from the viewer.

- [ ] **Step 1: Write the failing tests**

```python
def test_overhead_is_on_the_centre_line(robot):   # tool x = 200 mm and |y| < 0.3 mm; height equals postures.overhead_height_mm()
def test_overhead_is_the_highest_that_passes(robot):   # 2 mm higher has no arm solution that passes fixed_legs
def test_parked_fingertip_rests_on_the_pad(robot):     # gripper meets park_pad; the moving jaw is above the pad plane
def test_fold_is_clear_and_fine_grained(robot):
    path = postures.fold_path()
    assert np.abs(np.diff(np.rad2deg(path[:, 10:16]), axis=0)).max() <= 0.5 + 1e-9
    postures.fixed_legs(robot, model.tested_pairs(robot), path, "Fold", pad_contact=[len(path) - 1])

def test_pickup_holds_one_crouch_on_a_vertical_line(robot):   # postures.replay_pickup passes
```

- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_clips.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement.** Overhead: bisect the height between 400 and 660 mm to 1 mm, solving the arm alone at the reference stance. Parked: solve the fixed fingertip onto the pad top; if no posture passes at `PAD_XY_MM`, try pad centres on a 5 mm grid within the roof and keep the nearest that works. Pickup: `--search-pickup` at `CUBE_XY_MM`; if nothing passes, move the cube toward the feet in 5 mm steps. Fold: `between(overhead, waypoint)` then `between(waypoint, parked)`, the waypoint found by trying the stored waypoint first and then arm postures sampled between the two ends.
- [ ] **Step 4: Run** the tests, then `python hollow/postures.py`. Expected: 5 passed and `POSTURE REPLAY PASSED` up to the viewer comparison, which Task 11 restores.
- [ ] **Step 5: Commit** "Redo the pickup, overhead, parked and fold postures".

---

### Task 11: Viewer

**Files:**
- Create: `hollow/viewer_build.py`, `hollow/tests/test_viewer.py`
- Modify: `hollow/viewer.html`, `hollow/postures.py` (`write_viewer` covers every clip), `hollow/inspect_viewer.py`

**Interfaces:**
- Consumes: `hollow/parts/*.stl`, `hollow/parts/stock/*.stl`, the CSV files, `inspect_viewer.decode_mesh`, `inspect_viewer.load_viewer`.
- Produces: `python hollow/viewer_build.py` replaces, in the viewer's payload, the meshes of the three parts, the stock links and the servos with the new ones, snapped to the 0.1 mm grid with at most 65,535 vertices each, and the three cutaway pairs (`sectionOf`), cut at the same planes as today; it keeps every other mesh and the payload's key order; it sets the clip frame counts and rewrites the two texts that quote figures (overhead height, target count); `inspect_viewer.EXPECTED_FRAMES` follows the new clips; `inspect_viewer.check_meshes` asserts each part mesh in the viewer lies within 0.15 mm of its `hollow/parts` STL; `inspect_viewer.arm_masses` is replaced by reading body masses from `hollow/robot.xml`.

- [ ] **Step 1: Write the failing tests**

```python
def test_viewer_meshes_follow_the_parts():      # each of the three part meshes: every vertex within 0.15 mm of the STL surface
def test_viewer_meshes_fit_the_index_type():    # every mesh has <= 65535 vertices and indices in range
def test_viewer_shows_the_csv_postures(robot):  # postures.replay_viewer passes for targets, pickup, fold, overhead, park
def test_viewer_script_is_untouched_but_for_two_texts():   # diff of the inline script against HEAD~: only the two sentences
```

- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_viewer.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement**, then run `python hollow/viewer_build.py && python hollow/postures.py --write-viewer`.
- [ ] **Step 4: Run** `python -m pytest hollow/tests -q && python hollow/postures.py && python hollow/inspect_viewer.py --screen --legs --replay`. Expected: all pass, `POSTURE REPLAY PASSED`, `VIEWER CHECK PASSED`.
- [ ] **Step 5: Load the viewer in a browser.** In a scratch directory outside the repository, `npm install playwright && npx playwright install chromium-headless-shell`, then open `hollow/viewer.html` twice: once normally and once with `getContext('webgl2')` forced to return null. In each, click every mode, every part and every cutaway. Expected: no page errors, and a non-empty canvas in every state. Look at the screenshots of the assembly and of each cutaway.
- [ ] **Step 6: Commit** "Show the rebuilt parts and postures in the viewer".

---

### Task 12: Build guide, README, CI

**Files:**
- Create: `hollow/BUILD.md`, `hollow/tests/test_docs.py`
- Modify: `hollow/README.md`, `README.md`, `docs/PROVENANCE.md`, `.github/workflows/hollow.yml`

**Interfaces:**
- Consumes: the printed output of `loads.py`, `postures.py`, `inspect_viewer.py` and `build.py`.
- Produces: `hollow/BUILD.md` with three sections. **Parts and fasteners:** the three printed parts with mass and bounding box; every stock SO-101 part by its file name; seven servos with the joint each drives; every screw by size, length, count and where it goes, each traced to a donor feature or a measured hole. **Printing:** for each part the orientation, whether supports are needed and where, material, perimeters and infill. **Assembly:** numbered steps in build order; the servo bus order and IDs; for each of the seven joints, how to index the horn at the zero pose and the check that shows it is not one tooth (14.4 degrees) off.

- [ ] **Step 1: Write the failing tests**

```python
JOINTS7 = ["shoulder yaw", "shoulder pitch", "elbow", "forearm roll", "wrist flex", "wrist roll", "gripper"]
def test_build_guide_covers_every_joint_and_part():
    text = open("hollow/BUILD.md").read().lower()
    assert all(text.count(j) >= 2 for j in JOINTS7)             # once in the servo list, once in horn indexing
    assert all(n in text for n in ("neck_socket_mount", "elbow_to_forearm_roll", "forearm_roll_to_wrist"))
    assert "14.4" in text and "not yet printed" in text

def test_readme_figures_match_the_programs():    # the coverage count, the two shoulder loads, the baseline and the overhead height quoted in hollow/README.md equal the values the scripts compute, to the printed precision
def test_no_readiness_claim():                   # neither README nor BUILD.md contains "ready to print" or "ready to build"
```

- [ ] **Step 2: Run** `python -m pytest hollow/tests/test_docs.py -q`. Expected: FAIL.
- [ ] **Step 3: Write** `BUILD.md`, rewrite `hollow/README.md` around the new parts (what changed and why, the load table against the stock baseline, coverage, the reconstructed-model section replaced by the build, checks, limits), update the root `README.md` paragraph and `docs/PROVENANCE.md`.
- [ ] **Step 4: Extend CI.** In `hollow.yml` add, after the helper build: `python -m pytest hollow/tests -q` and `python hollow/cad/build.py --check`. Raise `timeout-minutes` to 40.
- [ ] **Step 5: Run** `python -m pytest hollow/tests -q`, `python scripts/verify_study.py`, `python scripts/verify_structure.py`, `cd coverage && python replay.py`. Expected: all pass; the six-part replay still reports `Front 64/64. Broad 31/32. Failed IDs [64]`.
- [ ] **Step 6: Commit** "Write the build guide and wire the new checks into CI".

---

## After the last task

Report to the owner: the chosen lengths, the two shoulder loads against the baseline, the coverage count with the unreached targets, the overhead height, the parking pad position, and the three part masses. Ask whether the new parts are accepted in the viewer; only then propose removing `hollow/review-meshes/` and `hollow/meshes/`, and pushing the branch.
