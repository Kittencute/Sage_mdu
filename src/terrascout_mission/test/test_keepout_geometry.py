from terrascout_mission.keepout_geometry import build_rectangle


def test_axis_aligned_rectangle():
    points = build_rectangle(center_x=10.0, center_y=20.0, x_length=8.0, y_length=6.0, yaw_deg=0.0)
    assert points == [
        (14.0, 23.0),
        (14.0, 17.0),
        (6.0, 17.0),
        (6.0, 23.0),
    ]


def test_rotated_rectangle_preserves_center():
    points = build_rectangle(center_x=2.5, center_y=-3.5, x_length=8.0, y_length=6.0, yaw_deg=30.0)
    center_x = sum(point[0] for point in points) / 4.0
    center_y = sum(point[1] for point in points) / 4.0
    assert abs(center_x - 2.5) < 1e-9
    assert abs(center_y + 3.5) < 1e-9
