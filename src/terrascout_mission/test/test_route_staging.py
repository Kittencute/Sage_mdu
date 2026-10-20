import math

import pytest

from terrascout_mission.route_staging import next_navigation_segment, next_path_segment


def test_long_leg_gets_bounded_intermediate_goal():
    point, staged = next_navigation_segment((0.0, 0.0), (100.0, 0.0), 40.0)

    assert staged is True
    assert point == pytest.approx((40.0, 0.0))


def test_short_leg_uses_destination_directly():
    destination = (30.0, 40.0)
    point, staged = next_navigation_segment((0.0, 0.0), destination, 50.0)

    assert staged is False
    assert point == destination


def test_far_destination_is_capped_to_local_planner_segment():
    point, staged = next_navigation_segment((45.91, 21.82), (-13.14, 20.90), 38.5)

    assert staged is True
    assert math.dist((45.91, 21.82), point) == pytest.approx(38.5)
    assert math.dist(point, (-13.14, 20.90)) > 0.0


def test_each_segment_is_bounded_and_advances_to_destination():
    current = (0.0, 0.0)
    destination = (100.0, 80.0)
    max_length = 35.0
    segments = 0

    while True:
        target, staged = next_navigation_segment(current, destination, max_length)
        assert math.dist(current, target) <= max_length + 1e-9
        segments += 1
        current = target
        if not staged:
            break

    assert current == destination
    assert segments == 4


@pytest.mark.parametrize("limit", [0.0, -1.0, math.inf, math.nan])
def test_invalid_segment_limit_rejected(limit):
    with pytest.raises(ValueError):
        next_navigation_segment((0.0, 0.0), (1.0, 0.0), limit)


def test_path_segment_uses_planned_curve_not_straight_line():
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (2.0, 8.0)]

    target, staged = next_path_segment(path, 4.0)

    assert staged is True
    assert target == pytest.approx((2.0, 2.0))


def test_short_planned_path_uses_destination():
    path = [(0.0, 0.0), (1.0, 1.0), (2.0, 1.0)]

    target, staged = next_path_segment(path, 5.0)

    assert staged is False
    assert target == path[-1]


def test_path_segment_rejects_empty_path():
    with pytest.raises(ValueError):
        next_path_segment([], 5.0)
