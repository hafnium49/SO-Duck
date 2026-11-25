import numpy as np
import pytest

import stock


@pytest.mark.parametrize("name", [*stock.STOCK, *stock.SERVOS])
def test_stock_part_matches_its_display_mesh(name):
    transform, rms, share = stock.register(name)
    assert rms < 0.15 and share > 0.95
    assert np.allclose(transform[:3, :3] @ transform[:3, :3].T, np.eye(3), atol=1e-9)
    assert np.allclose(transform, stock.placement(name), atol=1e-4)


def test_horn_axis_passes_through_the_horn():
    servo = stock.servo_mesh()
    point, direction = stock.SERVO_AXIS
    assert np.allclose(direction, (0, 0, 1))
    horn = servo.vertices[servo.vertices[:, 2] > 17.5]
    assert np.allclose(horn[:, :2].mean(0), point[:2], atol=0.3)


def test_connector_boxes_start_at_the_connector_strip():
    servo = stock.servo_mesh()
    strip = servo.vertices[servo.vertices[:, 2] < -18.5]  # the only geometry below the idler-side cover
    assert len(stock.SERVO_CONNECTORS) == 2
    low = np.min([box[0] for box in stock.SERVO_CONNECTORS], axis=0)
    high = np.max([box[1] for box in stock.SERVO_CONNECTORS], axis=0)
    assert np.all(strip[:, :2] >= low[:2] - 1e-6) and np.all(strip[:, :2] <= high[:2] + 1e-6)
    for box_low, box_high in stock.SERVO_CONNECTORS:
        assert np.allclose(np.subtract(box_high, box_low), (5, 9, 12))


def test_placed_meshes_are_in_millimetres_in_the_body_frame(root):
    import trimesh

    display = trimesh.load(root / "hollow/meshes/forearm_roll_case.stl")
    assert np.allclose(stock.mesh("forearm_roll_case").bounds, display.bounds, atol=0.3)
    solid = stock.solid("forearm_pitch_link").BoundingBox()
    display = trimesh.load(root / "hollow/meshes/forearm_pitch_link.stl").bounds
    assert np.allclose([solid.xmin, solid.ymin, solid.zmin, solid.xmax, solid.ymax, solid.zmax], display.ravel(), atol=0.3)
