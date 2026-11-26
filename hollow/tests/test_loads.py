import loads


def test_stock_baseline():
    empty, loaded = loads.stock_baseline()
    assert 0.84 < empty < 0.88 and 1.24 < loaded < 1.29  # spike: 0.863 and 1.265


def test_payload_never_lowers_the_worst_case(robot, passing_q):
    loaded = loads.arm_torques(robot, passing_q, 0.1)[:, loads.SHOULDER].max()
    assert loaded > loads.arm_torques(robot, passing_q)[:, loads.SHOULDER].max()


def test_torques_have_one_row_per_posture_and_one_column_per_arm_joint(robot, passing_q):
    assert loads.arm_torques(robot, passing_q[:3]).shape == (3, 6)
