"""Shortest strut centerline around the servo case and the rotating horn keepout."""

import numpy as np
from scipy.sparse.csgraph import dijkstra


def route(a, b, start, axis, end, housing, base):
    # Work in mm. Expand obstacles by the 7.5 mm tube radius.
    def valid(points):
        p = housing.inv().apply(points - end)
        case = (
            (p[:, 0] > -18.5)
            & (p[:, 0] < 43.0)
            & (abs(p[:, 1]) < 20.2)
            & (p[:, 2] > -9.2)
            & (p[:, 2] < 46)
        )
        v = axis.inv().apply(points - start)
        rotor = (np.hypot(v[:, 0], v[:, 1]) < 46) & (v[:, 2] > -9.2) & (v[:, 2] < 46)
        return ~(case | (False if base else rotor))

    corners = np.array(
        [[x, y, z] for x in (-20, 45) for y in (-22, 22) for z in (-11, 48)]
    )
    ring = np.array(
        [
            [r * np.cos(t), r * np.sin(t), z]
            for r in (20, 50, 60)
            for t in np.arange(12) * np.pi / 6
            for z in (-17, 50)
        ]
    )
    nodes = np.vstack([a, b, end + housing.apply(corners), start + axis.apply(ring)])
    valid_nodes = valid(nodes)
    if not valid_nodes[:2].all():
        raise ValueError(
            f"Mounts need more space at {end}: {valid_nodes[:2]} incoming frame {housing.inv().apply(nodes[:2]-end)} outgoing frame {axis.inv().apply(nodes[:2]-start)}"
        )
    nodes = nodes[valid_nodes]
    dist = np.zeros((len(nodes), len(nodes)))
    for i, p in enumerate(nodes[:-1]):
        diff = nodes[i + 1 :] - p
        steps = max(5, int(np.ceil(np.linalg.norm(diff, axis=1).max() / 1.5)))
        check = p + np.linspace(0, 1, steps)[:, None, None] * diff[None]
        clear = valid(check.reshape(-1, 3)).reshape(steps, -1).all(0)
        dist[i, i + 1 :] = np.linalg.norm(diff, axis=1) * clear
    _, prev = dijkstra(dist, directed=False, indices=0, return_predecessors=True)
    path = [1]
    while path[-1] != 0:
        path.append(prev[path[-1]])
        if path[-1] < 0:
            raise ValueError(f"No printable route at {end}")
    return nodes[path[::-1]]
