"""Bake a self-contained orbitable viewer of the current MJCF and sampled motions."""
import json
from itertools import product
from scipy.spatial import ConvexHull

from geometry import Robot, HERE, ROOT, FINGER, pose_test, np, screen
from motion import CUBE_SIDE, HIGH, cube_frames, fold, pickup


def simplify(triangles, size):
    points = triangles.reshape(-1, 3)
    grid = np.rint(points / size).astype(int)
    _, index, counts = np.unique(grid, axis=0, return_inverse=True, return_counts=True)
    index = index.ravel()
    vertices = np.column_stack([np.bincount(index, weights=c) / counts for c in points.T])
    faces = index.reshape(-1, 3)
    keep = (faces[:,0] != faces[:,1]) & (faces[:,1] != faces[:,2]) & (faces[:,0] != faces[:,2])
    faces = faces[keep]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return np.rint(vertices * 10000).astype(int), faces[first]


def main():
    robot = Robot()
    vertices = np.array(list(product((-CUBE_SIDE / 2, CUBE_SIDE / 2), repeat=3)))
    geometry = {**robot.mesh, "cube": vertices[ConvexHull(vertices).simplices]}
    meshes = []
    for i, (name, mesh) in enumerate(geometry.items()):
        verts, face = simplify(mesh, .0005 if name in ("park_pad", "cube") else .0028)
        color = 3 if name == "carrier" else 4 if name in ("park_pad", "cube") else (
            1 if name in ("forearm", "wrist", "gripper", FINGER) else 2 if "case" in name else 0)
        meshes.append([i, color, verts.ravel().tolist(), face.ravel().tolist()])
    poses = []
    joints, grasp, contact = pickup()
    paths = [("Fold over body", fold(), [0, .25, .5, .75, 1.], ["Ready", "25%", "50%", "75%", "Seated"]),
             ("Floor pickup", joints, None, ["Approach", "Open", "Contact", "Lifted 30 mm"]),
             ("Vertical reach", HIGH[None], [0], ["200 mm forward"])]
    for name, path, fractions, labels in paths:
        frames, com = robot.placements(path)
        if fractions is None:
            indices = [0, grasp, contact, len(path)-1]
            frames["cube"] = cube_frames(path, contact, frames)
        else:
            indices = np.rint(np.array(fractions) * (len(path)-1)).astype(int)
        for i, label in zip(indices, labels):
            tcp = frames["gripper"][i] @ screen.a.TCP
            poses.append(dict(mode=name, label=label, x=(tcp[:3,3]*1000).round(1).tolist(),
                              q=path[i].round(1).tolist(), m=round(1000*pose_test.GROUND.margin(com[i]), 1),
                              tcp=tcp[:3].round(8).tolist(),
                              c=[frames[n][i, :3].round(8).ravel().tolist() if n in frames else None
                                 for n in geometry]))
    points = pose_test.GROUND.sole_points
    sole = points[ConvexHull(points[:,:2]).vertices]
    sole = sole.copy()
    sole[:,2] = 0
    data = json.dumps(dict(meshes=meshes, poses=poses, sole=sole.round(6).tolist()), separators=(",", ":"))
    template = (HERE / "explorer.template.html").read_text()
    (ROOT / "SO_Duck_pose_explorer.html").write_text(template.replace("__POSE_DATA__", data))


if __name__ == "__main__":
    main()
