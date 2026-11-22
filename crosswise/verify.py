"""Check CAD fit, MJCF/FK agreement, sampled trajectories, and opposing jaw contacts."""
from collections import Counter
from itertools import product
import xml.etree.ElementTree as ET

import mujoco

import cadquery as cq
import numpy as np
from scipy.spatial import ConvexHull

from geometry import ARM, DISTAL, FINGER, HERE, Robot, pose_test, screen
from motion import PAD_CENTER_MM, PAD_RADIUS_MM, PAD_HEIGHT_MM, ROOF_MM, CUBE_SIDE, HIGH, NEUTRAL, PARK, between, cube_frames, fold, pickup


def check_fit(robot):
    cad = cq.importers.importStep(str(HERE / "connector_assembly.step")).val().Solids()
    carrier = cq.importers.importStep(str(HERE / "carrier.step")).val()
    assert len(cad) == 4 and all(p.isValid() for p in [*cad, carrier])
    assert carrier.isValid() and len(carrier.Solids()) == 1
    assert abs(carrier.Volume() - cad[1].intersect(carrier).Volume()) < .01
    assert all(carrier.intersect(cad[i]).Volume() < .01 for i in (0, 2, 3))
    assert abs(carrier.distance(cad[3]) - 14.305) < .1
    print(f"Carrier {carrier.Volume():.1f} mm³, fork gap at zero {carrier.distance(cad[3]):.2f} mm")

    frames, _ = robot.placements(NEUTRAL[None])
    tree = ET.parse(HERE / "robot.xml")
    fork_geom = tree.find(".//geom[@name='forearm_under_arm_so101_v1']")
    fork_source = screen.read_stl(HERE.parent / "collision/inputs/arm_meshes/under_arm_so101_v1.stl")
    fork = screen.move(screen.legs.rest(fork_geom), fork_source)
    for name, solid in zip(("yaw_case", "carrier", "pitch_case", "forearm"), cad):
        transform = screen.a.PRINT_FROM_TRUNK @ np.linalg.inv(pose_test.W0) @ frames[name][0]
        points = screen.move(transform, fork if name == "forearm" else robot.mesh[name]).reshape(-1,3)*1000
        bounds = solid.BoundingBox()
        assert np.allclose(points.min(0), [bounds.xmin, bounds.ymin, bounds.zmin], atol=.02), name
        assert np.allclose(points.max(0), [bounds.xmax, bounds.ymax, bounds.zmax], atol=.02), name

    angles = np.arange(-50., 106.)
    sweep = np.zeros((len(angles), 7))
    sweep[:, 3] = angles
    frames, _ = robot.placements(sweep)
    clear = np.ones(len(angles), dtype=bool)
    for other in ("pitch_case", "carrier", "yaw_case"):
        clear &= ~robot.hits("forearm", other, frames)
    limits = np.rad2deg(robot.model.joint("forearm_pitch").range)
    assert not clear[angles == -42].item() and not clear[angles == 100].item()
    assert clear[(angles >= -41) & (angles <= 99)].all()
    assert np.allclose(limits, [-40, 96]) and limits[1] < np.rad2deg(1.69)
    print("Crosswise pivot local sweep: -41° to +99° clear in one-degree samples; model -40° to +96°")


def check_roof(robot):
    # Verify the entire circular pad fits on one flat region of the stock roof.
    elem = ET.parse(HERE / "robot.xml").find(".//geom[@mesh='body_middle_top']")
    mesh = screen.read_stl(HERE.parent / "collision/inputs/duck_meshes/body_middle_top.stl")
    roof = screen.move(screen.legs.rest(elem), mesh) * 1000
    top = roof[np.max(abs(roof[:, :, 2] - ROOF_MM), axis=1) < .002, :, :2]
    center = np.array(PAD_CENTER_MM)
    edges = Counter()
    inside = False
    for tri in top:
        ab = np.roll(tri, -1, axis=0) - tri
        ap = center - tri
        cross = ab[:, 0]*ap[:, 1] - ab[:, 1]*ap[:, 0]
        inside |= cross.min() >= 0 or cross.max() <= 0
        for start, end in zip(tri, np.roll(tri, -1, axis=0)):
            edges[tuple(sorted((tuple(start.round(4)), tuple(end.round(4)))))] += 1
    outline = np.array([edge for edge, count in edges.items() if count == 1])
    direction = outline[:, 1] - outline[:, 0]
    offset = center - outline[:, 0]
    along = np.clip((offset * direction).sum(1) / (direction**2).sum(1), 0, 1)
    edge_gap = np.linalg.norm(offset - direction * along[:, None], axis=1).min() - PAD_RADIUS_MM
    assert inside and edge_gap > 1.
    frames, _ = robot.placements(fold()[[-1]])
    top_z = pose_test.W0[2,3] + (ROOF_MM + PAD_HEIGHT_MM)*.001
    compression = 1000*(top_z - robot.lowest("gripper", frames)[0])
    moving_gap = 1000*(robot.lowest(FINGER, frames)[0] - top_z)
    assert .25 < compression < .45 and moving_gap > 3.
    print(f"Pad edge clearance {edge_gap:.2f} mm, nominal fixed-finger indentation "
          f"{compression:.2f} mm, moving finger above pad plane {moving_gap:.2f} mm")


def check_axes(robot):
    example = np.array([20., -15., 25., 10., 30., 0., 0.])
    for q, rank in [(NEUTRAL, 5), (example, 6)]:
        robot.placements(q[None])
        jac_pos = np.zeros((3, robot.model.nv))
        jac_rot = np.zeros_like(jac_pos)
        mujoco.mj_jacSite(robot.model, robot.data, jac_pos, jac_rot, robot.model.site("gripperframe").id)
        dofs = [robot.model.joint(name).dofadr[0] for name in ARM]
        assert np.linalg.matrix_rank(np.vstack([jac_pos/.15, jac_rot])[:, dofs], tol=1e-6) == rank
    robot.placements(NEUTRAL[None])
    for name in ("forearm_pitch", "wrist_flex"):
        assert np.allclose(robot.data.xaxis[robot.model.joint(name).id], [0,1,0], atol=1e-6)
    assert robot.model.actuator("forearm_pitch").id == 8


def check_path(robot, name, joints):
    indices = [robot.model.joint(j).id for j in (*ARM, "gripper")]
    limits = robot.model.jnt_range[indices]
    q = np.deg2rad(joints)
    assert np.all(q >= limits[:, 0] - 1e-6) and np.all(q <= limits[:, 1] + 1e-6), name
    frames, com = robot.placements(joints)
    collisions = {}
    for a, b in robot.pairs():
        hits = np.flatnonzero(robot.hits(a, b, frames))
        if name == "fold" and set((a, b)) == {"park_pad", "gripper"}:
            # Allow first contact only in the last 0.5 degrees of approach.
            assert len(hits) and hits[-1] == len(joints)-1 and hits[0] >= len(joints)-2, hits
            continue
        if len(hits):
            collisions[(a,b)] = hits.tolist()[:5]
    assert not collisions, (name, collisions)
    heights = [robot.lowest(p, frames).min() for p in robot.mesh if not p.startswith("foot_assembly")]
    margin = min(pose_test.GROUND.margin(point) for point in com)
    assert min(heights) > -.00001 and margin > .005, (name, min(heights), margin)
    for foot in ("foot_assembly", "foot_assembly_2"):
        assert np.max(abs(robot.lowest(foot, frames))) < .00011
    print(f"{name}: {len(joints)} samples, nonfoot clearance {1000*min(heights):.2f} mm, "
          f"unloaded static margin {1000*margin:.1f} mm")
    return frames


def check_height_and_gravity(robot):
    pose, grasp, _ = pickup()
    for name, joints in [("upright", NEUTRAL), ("folded", PARK), ("cube grasp", pose[grasp])]:
        frames, _ = robot.placements(joints[None])
        height = max(screen.move(frames[k][0], points)[:,2].max()
                     for k, points in robot.extreme.items())
        mujoco.mj_forward(robot.model, robot.data)
        dofs = [robot.model.joint(j).dofadr[0] for j in ARM]
        torques = robot.data.qfrc_bias[dofs]
        print(f"{name}: robot height {height*1000:.1f} mm, gravity torques {torques.round(3)} N m")
        if name == "folded":
            assert height < .450


def cheek_points(vertices):
    """Clip triangles to a 20 mm slab in cube y and z; retain actual contact features."""
    half = CUBE_SIDE / 2
    selected = vertices[(vertices[:, :, 1:].max(1) >= -half).all(1) &
                        (vertices[:, :, 1:].min(1) <= half).all(1)]
    points = []
    for triangle in selected:
        polygon = list(triangle)
        for axis, sign in product((1, 2), (-1, 1)):
            clipped = []
            for i, end in enumerate(polygon):
                start = polygon[i - 1]
                u, v = half - sign * start[axis], half - sign * end[axis]
                if (u >= 0) != (v >= 0):
                    clipped.append(start + u / (u - v) * (end - start))
                if v >= 0:
                    clipped.append(end)
            polygon = clipped
        points.extend(polygon)
    return np.array(points)


def check_cube(robot, joints, grasp, contact, frames):
    cube = cube_frames(joints, contact, frames)
    half = CUBE_SIDE / 2
    corners = np.array(list(product((-half, half), repeat=3)))
    robot.load("cube", corners[ConvexHull(corners).simplices])
    for part in robot.mesh:
        if part not in ("gripper", FINGER):
            assert not robot.hits(part, "cube", {**frames, "cube": cube}).any(), part
    palm = np.linalg.inv(cube) @ frames["gripper"]
    finger = np.linalg.inv(cube) @ frames[FINGER]
    # On each cheek find the deepest vertex of the true triangle/footprint intersection.
    # A cheek can be wholly above the cube early in the descent.
    widths = []
    for frame_a, frame_b in zip(palm, finger):
        a = cheek_points(screen.move(frame_a, robot.mesh["gripper"]))
        b = cheek_points(screen.move(frame_b, robot.mesh[FINGER]))
        left = a[:,0].max() if len(a) else -np.inf
        right = b[:,0].min() if len(b) else np.inf
        assert left <= -half + .00002 and right >= half - .00002
        widths.append(right-left)
    assert .040 < widths[grasp] < .042
    assert max(abs(np.array(widths[contact:]) - CUBE_SIDE)) < .00002
    lift = cube[-1, 2, 3] - cube[contact, 2, 3]
    assert abs(lift - .030) < .00002
    print(f"Cube {CUBE_SIDE*1000:.0f} mm, open gap {widths[grasp]*1000:.2f} mm, "
          f"closed gap {widths[contact]*1000:.2f} mm, assumed no-slip lift {lift*1000:.2f} mm")


def main():
    robot = Robot()
    assert robot.model.nu == 17 and robot.model.njnt == 18  # plus the floating base
    check_fit(robot)
    check_axes(robot)
    check_roof(robot)
    check_height_and_gravity(robot)
    trajectory, grasp, contact = pickup()
    check_path(robot, "fold", fold())
    check_path(robot, "neutral to pickup approach", between(NEUTRAL, trajectory[0]))
    frames = check_path(robot, "floor pickup", trajectory)
    check_cube(robot, trajectory, grasp, contact, frames)
    check_path(robot, "high reach", HIGH[None])
    gripper = robot.model.site("gripperframe").id
    tcp = robot.data.site_xpos[gripper]
    assert np.linalg.norm(tcp[:2] - [.200,0]) < .0002 and abs(tcp[2] - .65060) < .0002
    print("High TCP height above soles", round(tcp[2]*1000, 2), "mm")


if __name__ == "__main__":
    main()
