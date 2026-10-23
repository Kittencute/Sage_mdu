import json

from fleet_common.waypoint_geojson import build_waypoints_geojson


def test_home_stays_pending_when_an_earlier_destination_fails():
    result = json.loads(build_waypoints_geojson(
        [{"lat": 47.0, "lon": 8.0}],
        current_index=0,
        failed=True,
        return_position={"lat": 47.1, "lon": 8.1},
    ))
    home = next(feature for feature in result["features"]
                if feature["properties"]["name"] == "Home")
    assert home["properties"]["metadata"]["status"] == "pending"
    assert home["properties"]["style"]["color"] == "#3b82f6"


def test_home_is_active_or_completed_only_when_return_state_says_so():
    kwargs = {
        "waypoints": [{"lat": 47.0, "lon": 8.0}],
        "current_index": 1,
        "return_position": {"lat": 47.1, "lon": 8.1},
    }
    active = json.loads(build_waypoints_geojson(**kwargs, return_active=True))
    complete = json.loads(build_waypoints_geojson(**kwargs, mission_complete=True))
    active_home = next(
        f for f in active["features"]
        if f["geometry"]["type"] == "Point" and f["properties"]["name"] == "Home"
    )
    complete_home = next(
        f for f in complete["features"]
        if f["geometry"]["type"] == "Point" and f["properties"]["name"] == "Home"
    )
    assert active_home["properties"]["metadata"]["status"] == "active"
    assert complete_home["properties"]["metadata"]["status"] == "completed"


def test_failed_waypoint_is_red_and_its_route_legs_are_removed():
    waypoints = [
        {"lat": 47.0, "lon": 8.0},
        {"lat": 47.1, "lon": 8.1},
        {"lat": 47.2, "lon": 8.2},
    ]
    result = json.loads(build_waypoints_geojson(
        waypoints,
        current_index=1,
        start_position={"lat": 46.9, "lon": 7.9},
        return_position={"lat": 47.3, "lon": 8.3},
        active=True,
        waypoint_statuses=["completed", "failed", "active"],
    ))

    failed = next(
        feature for feature in result["features"]
        if feature["geometry"]["type"] == "Point"
        and feature["properties"]["name"] == "WP 2"
    )
    lines = [
        feature["geometry"]["coordinates"] for feature in result["features"]
        if feature["geometry"]["type"] == "LineString"
    ]
    assert failed["properties"]["metadata"]["status"] == "failed"
    assert failed["properties"]["style"]["color"] == "#ef4444"
    assert [[8.0, 47.0], [8.2, 47.2]] in lines
    assert [[8.0, 47.0], [8.1, 47.1]] not in lines
    assert [[8.1, 47.1], [8.2, 47.2]] not in lines
