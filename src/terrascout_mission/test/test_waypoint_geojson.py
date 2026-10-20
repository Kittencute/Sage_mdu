import json

from fleet_common.waypoint_geojson import build_waypoints_geojson

WAYPOINTS = [
    {"lat": 47.397787, "lon": 8.545594},
    {"lat": 47.397877, "lon": 8.545726},
    {"lat": 47.397960, "lon": 8.545823},
]

GREEN = "#22c55e"
ORANGE = "#f97316"
BLUE = "#3b82f6"
RED = "#ef4444"
GRAY = "#9ca3af"


def _parse(geojson_str):
    fc = json.loads(geojson_str)
    points = [f for f in fc["features"] if f["geometry"]["type"] == "Point"]
    lines = [f for f in fc["features"] if f["geometry"]["type"] == "LineString"]
    return fc, points, lines


def test_all_pending():
    """current_index=0: all points blue, one pending LineString, no completed/active."""
    result = build_waypoints_geojson(WAYPOINTS, current_index=0)
    fc, points, lines = _parse(result)

    assert fc["type"] == "FeatureCollection"
    assert len(points) == 3
    for p in points:
        assert p["properties"]["style"]["color"] == BLUE

    # One pending line covering all waypoints
    assert len(lines) == 1
    assert lines[0]["properties"]["style"]["color"] == BLUE
    assert len(lines[0]["geometry"]["coordinates"]) == 3


def test_mid_route():
    """current_index=1: WP0 green, WP1 orange, WP2 blue.
    No green LineString (completed is single point). Active + pending lines only.
    """
    result = build_waypoints_geojson(WAYPOINTS, current_index=1)
    fc, points, lines = _parse(result)

    assert points[0]["properties"]["style"]["color"] == GREEN
    assert points[1]["properties"]["style"]["color"] == ORANGE
    assert points[2]["properties"]["style"]["color"] == BLUE

    # current_index=1: completed is just WP0 (single point, no line).
    # Active line: WP0→WP1 (orange). Pending line: WP1→WP2 (blue).
    assert len(lines) == 2
    line_colors = [line["properties"]["style"]["color"] for line in lines]
    assert line_colors == [ORANGE, BLUE]
    assert GREEN not in line_colors


def test_mid_route_two_completed():
    """current_index=2: WP0-1 green, WP2 orange. Green + orange lines."""
    result = build_waypoints_geojson(WAYPOINTS, current_index=2)
    fc, points, lines = _parse(result)

    assert points[0]["properties"]["style"]["color"] == GREEN
    assert points[1]["properties"]["style"]["color"] == GREEN
    assert points[2]["properties"]["style"]["color"] == ORANGE

    # Completed line: WP0→WP1 (green). Active line: WP1→WP2 (orange). No pending.
    assert len(lines) == 2
    line_colors = [line["properties"]["style"]["color"] for line in lines]
    assert GREEN in line_colors
    assert ORANGE in line_colors


def test_line_dash_array():
    """All lines have dashed style."""
    result = build_waypoints_geojson(WAYPOINTS, current_index=1)
    _, _, lines = _parse(result)

    for line in lines:
        assert line["properties"]["style"]["dashArray"] == "4 4"


def test_all_completed():
    """current_index=len(waypoints): all green."""
    result = build_waypoints_geojson(WAYPOINTS, current_index=3)
    fc, points, lines = _parse(result)

    for p in points:
        assert p["properties"]["style"]["color"] == GREEN

    for line in lines:
        assert line["properties"]["style"]["color"] == GREEN


def test_failure_at_waypoint():
    """failed=True at index 1: WP0 green, WP1 red, WP2 gray."""
    result = build_waypoints_geojson(WAYPOINTS, current_index=1, failed=True)
    fc, points, lines = _parse(result)

    assert points[0]["properties"]["style"]["color"] == GREEN
    assert points[1]["properties"]["style"]["color"] == RED
    assert points[2]["properties"]["style"]["color"] == GRAY


def test_explicit_waypoint_statuses_preserve_failed_destination():
    result = build_waypoints_geojson(
        WAYPOINTS,
        current_index=2,
        waypoint_statuses=["completed", "failed", "active"],
    )
    _, points, _ = _parse(result)

    statuses = [point["properties"]["metadata"]["status"] for point in points]
    colors = [point["properties"]["style"]["color"] for point in points]
    assert statuses == ["completed", "failed", "active"]
    assert colors == [GREEN, RED, ORANGE]


def test_point_metadata():
    """Points have name and metadata in properties."""
    result = build_waypoints_geojson(WAYPOINTS, current_index=0)
    fc, points, _ = _parse(result)

    assert points[0]["properties"]["name"] == "WP 1"
    assert points[1]["properties"]["name"] == "WP 2"
    assert points[0]["properties"]["metadata"]["lat"] == "47.397787"
    assert points[0]["properties"]["metadata"]["lon"] == "8.545594"


def test_geojson_coordinate_order():
    """GeoJSON uses [lon, lat] per RFC 7946."""
    result = build_waypoints_geojson(WAYPOINTS, current_index=0)
    fc, points, _ = _parse(result)

    coords = points[0]["geometry"]["coordinates"]
    assert coords == [8.545594, 47.397787]


def test_single_waypoint():
    """Single waypoint: one point, no line segments."""
    single = [{"lat": 47.397787, "lon": 8.545594}]
    result = build_waypoints_geojson(single, current_index=0)
    fc, points, lines = _parse(result)

    assert len(points) == 1
    assert len(lines) == 0

def test_first_waypoint_active():
    result = build_waypoints_geojson(
        WAYPOINTS,
        current_index=0,
        active=True,
    )

    fc, points, lines = _parse(result)

    assert points[0]["properties"]["style"]["color"] == ORANGE
    assert points[0]["properties"]["metadata"]["status"] == "active"

    for point in points[1:]:
        assert point["properties"]["style"]["color"] == BLUE


def test_home_is_orange_while_returning():
    result = build_waypoints_geojson(
        WAYPOINTS,
        current_index=len(WAYPOINTS),
        return_position={"lat": 47.397700, "lon": 8.545500},
        return_active=True,
    )
    _, points, lines = _parse(result)
    home = next(point for point in points if point["properties"]["name"] == "Home")

    assert home["properties"]["style"]["color"] == ORANGE
    assert home["properties"]["metadata"]["status"] == "active"
    return_leg = next(
        line for line in lines
        if line["geometry"]["coordinates"][-1] == [8.545500, 47.397700]
    )
    assert return_leg["properties"]["style"]["color"] == ORANGE


def test_home_is_green_when_mission_finishes():
    result = build_waypoints_geojson(
        WAYPOINTS,
        current_index=len(WAYPOINTS),
        return_position={"lat": 47.397700, "lon": 8.545500},
        mission_complete=True,
    )
    _, points, lines = _parse(result)
    home = next(point for point in points if point["properties"]["name"] == "Home")

    assert home["properties"]["style"]["color"] == GREEN
    assert home["properties"]["metadata"]["status"] == "completed"
    return_leg = next(
        line for line in lines
        if line["geometry"]["coordinates"][-1] == [8.545500, 47.397700]
    )
    assert return_leg["properties"]["style"]["color"] == GREEN
