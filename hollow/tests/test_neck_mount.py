import mujoco
import numpy as np
import pytest
import trimesh
from scipy.spatial.transform import Rotation

import checks
import params
import parts
import stock

YAW_TURN_DEG = range(-108, 109, 6)
# The Duck's neck servo and its two horns leave the trunk: the mount takes their place.
NECK_SERVO = {"wj-wk00-0123middlecase_56", "wj-wk00-0124bottomcase_45", "wj-wk00-0122topcabinetcase_95", "drive_palonier", "passive_palonier"}


def trunk_meshes():
    """The trunk's meshes of coverage/robot.xml in the trunk frame, millimetres, by name."""
    model = mujoco.MjModel.from_xml_path(str(stock.ROOT / "coverage" / "robot.xml"))
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk_assembly")
    out = []
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] != trunk or model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[geom]
        vertices = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
        faces = model.mesh_face[model.mesh_faceadr[mesh] : model.mesh_faceadr[mesh] + model.mesh_facenum[mesh]]
        turn = Rotation.from_quat(np.roll(model.geom_quat[geom], -1)).as_matrix()
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh)
        out.append((name, trimesh.Trimesh(1000 * (vertices @ turn.T + model.geom_pos[geom]), faces.copy(), process=False)))
    return out


@pytest.fixture(scope="module")
def trunk():
    return trunk_meshes()


@pytest.fixture(scope="module")
def mount():
    return parts.neck_mount()


@pytest.fixture(scope="module")
def mesh(mount):
    return checks.tessellate(mount)


def probe_is_clear(mesh, start, end, radius=1.0):
    """Whether a rod of the given radius from start to end meets no surface of the mesh."""
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    along = (end - start) / np.linalg.norm(end - start)
    side = np.cross(along, [0.0, 0.0, 1.0])
    side /= np.linalg.norm(side)
    up = np.cross(along, side)
    ring = [np.zeros(3)] + [radius * (np.cos(a) * side + np.sin(a) * up) for a in np.linspace(0, 2 * np.pi, 8, endpoint=False)]
    origins = np.array([start + offset for offset in ring])
    hits, _, _ = mesh.ray.intersects_location(origins, np.tile(along, (len(origins), 1)), multiple_hits=True)
    hits = np.asarray(hits).reshape(-1, 3)
    return not np.any((hits - start) @ along < np.linalg.norm(end - start))


def test_recorded_screws_are_real_holes(trunk):
    top = next(mesh for name, mesh in trunk if name == "trunk_top")
    review = trimesh.load(stock.ROOT / "hollow" / "review-meshes" / "neck-socket-mount.preview.stl")
    assert len(params.NECK_SCREWS_MM) == 4
    for x, y, z in params.NECK_SCREWS_MM:
        outward = np.sign(y)
        assert probe_is_clear(top, (x, 15.5 * outward, z), (x, 23.5 * outward, z)), (x, y, z)  # through the socket wall
        assert probe_is_clear(review, (x, 10.0 * outward, z), (x, 16.5 * outward, z)), (x, y, z)  # through the review tab
        assert not probe_is_clear(top, (x, 15.5 * outward, z + 5.0), (x, 23.5 * outward, z + 5.0))  # the wall is there


def test_one_printable_solid(mount, mesh):
    assert mount.isValid() and len(mount.Solids()) == 1 and mesh.is_watertight
    assert checks.min_wall_mm(mount) >= params.MIN_WALL_MM
    assert checks.fits_bed(mount)


@pytest.mark.parametrize("centre", params.NECK_SCREWS_MM)
def test_tabs_take_the_four_neck_screws(mount, centre):
    # M2 screws cut their own thread in a 1.8 mm hole through the tab
    assert checks.hole_is_open(mount, centre, params.NECK_SCREW_AXIS, radius=0.8, depth=4.6)
    assert not checks.hole_is_open(mount, centre, params.NECK_SCREW_AXIS, radius=1.1, depth=4.6)


def test_yaw_servo_is_held_where_the_model_has_it(mesh):
    assert checks.servo_penetration_mm(mesh, "shoulder_yaw_case") <= checks.SERVO_ALLOWANCE_MM
    assert checks.clearance_mm(mesh, stock.mesh("shoulder_yaw_case")) < 0.3


@pytest.mark.parametrize("x, y, z", parts.YAW_SERVO_SCREWS_MM)
def test_yaw_servo_is_screwed_from_below(mount, x, y, z):
    # an M2 screw through the shelf into the servo's tapped hole, and a 4 mm driver below it
    assert checks.hole_is_open(mount, (x, y, z - 1.5), (0, 0, 1), radius=1.0, depth=2.6)
    assert checks.hole_is_open(mount, (x, y, z - 33.5), (0, 0, 1), radius=2.0, depth=60.0)


def test_yaw_servo_can_be_inserted(mesh):
    for out in range(4, 48, 4):  # lifted out of the sleeve
        depth = checks.servo_penetration_mm(mesh, "shoulder_yaw_case", shift=(0, 0, out))
        assert depth <= checks.SERVO_ALLOWANCE_MM, (out, depth)


def test_connectors_are_open(mount):
    assert checks.connector_blocked_mm3(mount, "shoulder_yaw_case") < 1.0


def test_mount_clears_the_trunk(mesh, trunk):
    for name, part in trunk:
        if name in NECK_SERVO and part.bounds.mean(axis=0)[0] > -5.0:
            continue  # the neck servo, removed
        assert checks.clearance_mm(mesh, part) >= 0.05, name


def test_yaw_turn_keeps_clearance(mesh):
    position, quaternion = params.FRAME["shoulder_yaw"]
    frame = np.eye(4)
    frame[:3, :3] = Rotation.from_quat(np.roll(quaternion, -1)).as_matrix()
    frame[:3, 3] = 1000 * np.asarray(position)
    about_axis = mesh.copy()
    about_axis.apply_transform(np.linalg.inv(frame))  # the mount in the yaw body's frame
    # the stage sits on the servo's horn and on its idler; skip those seats
    seat = lambda points: np.hypot(points[:, 0], points[:, 1]) < 13.0
    for angle in YAW_TURN_DEG:
        turn = np.eye(4)
        turn[:3, :3] = Rotation.from_euler("z", angle, degrees=True).as_matrix()
        for name in ("shoulder_pitch_link", "shoulder_collar", "shoulder_pitch_case"):
            assert checks.clearance_mm(about_axis, stock.mesh(name), skip=seat, move_b=turn) >= params.MOVING_CLEARANCE_MM, (angle, name)
