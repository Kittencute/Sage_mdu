import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fleet_common.send_mission import (
    HOME,
    LAND,
    TAKEOFF,
    WAYPOINT,
    build_plan_items,
    parse_coords,
)


def test_plan_starts_with_takeoff_and_ends_with_home_and_land():
    items = build_plan_items(
        [(47.397787, 8.545594, 10.0), (47.397810, 8.545660, 12.0)], altitude=10.0
    )

    assert [item["kind"] for item in items] == [TAKEOFF, WAYPOINT, WAYPOINT, HOME, LAND]
    assert [item["id"] for item in items] == [1, 2, 3, 4, 5]
    assert items[0]["altitude_agl"] == 10.0
    assert items[1]["latitude"] == 47.397787
    assert items[1]["longitude"] == 8.545594
    assert items[2]["altitude_agl"] == 12.0


def test_hold_is_attached_to_every_waypoint():
    items = build_plan_items([(47.397787, 8.545594, 5.0)], altitude=5.0, hold=3.0)

    assert items[1]["hold_s"] == 3.0
    assert "hold_s" not in items[0]


def test_parse_coords_fills_missing_altitude():
    assert parse_coords(["59.6,16.6", "59.7,16.7,4.0"], altitude=9.0) == [
        (59.6, 16.6, 9.0),
        (59.7, 16.7, 4.0),
    ]
