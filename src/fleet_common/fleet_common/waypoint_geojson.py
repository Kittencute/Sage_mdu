import json

GREEN = "#22c55e"
ORANGE = "#f97316"
BLUE = "#3b82f6"
RED = "#ef4444"
GRAY = "#9ca3af"
DASH = "4 4"


def build_waypoints_geojson(
    waypoints,
    current_index,
    failed=False,
    start_position=None,
    return_position=None,
    active=False,
    return_active=False,
    mission_complete=False,
    waypoint_statuses=None,
    waypoint_command_ids=None,
):
    features = []
    coords = [[wp["lon"], wp["lat"]] for wp in waypoints]

    statuses = [
        (
            waypoint_statuses[i]
            if waypoint_statuses is not None and i < len(waypoint_statuses)
            else _point_status(i, current_index, failed, active)
        )
        for i in range(len(waypoints))
    ]

    # Failed destinations remain visible as red points but are removed from the
    # route geometry. Connect the surrounding reachable destinations directly.
    route_points = []
    if start_position is not None:
        route_points.append(("start", [start_position["lon"], start_position["lat"]]))
    route_points.extend(
        (statuses[index], coords[index])
        for index in range(len(coords))
        if statuses[index] != "failed"
    )
    if return_position is not None:
        route_points.append(("home", [return_position["lon"], return_position["lat"]]))

    for route_index, ((start_status, start), (end_status, end)) in enumerate(
        zip(route_points, route_points[1:])
    ):
        if start == end:
            continue
        if mission_complete:
            color = GREEN
        elif return_active and end_status == "home":
            color = ORANGE
        elif end_status == "completed":
            color = GREEN
        elif end_status == "active":
            color = ORANGE
        elif end_status == "pending":
            color = BLUE
        elif start_status == "active":
            color = ORANGE
        else:
            color = BLUE
        features.append(_line([start, end], color))

    for i, wp in enumerate(waypoints):
        status = statuses[i]
        color = {
            "completed": GREEN,
            "failed": RED,
            "skipped": GRAY,
            "active": ORANGE,
            "pending": BLUE,
        }.get(status, BLUE)
        features.append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [wp["lon"], wp["lat"]]},
            "properties": {
                "name": f"WP {i + 1}",
                "metadata": {
                    "status": status,
                    "lat": str(wp["lat"]),
                    "lon": str(wp["lon"]),
                    **({"command_id": str(waypoint_command_ids[i])}
                       if waypoint_command_ids is not None and i < len(waypoint_command_ids) else {}),
                },
                "style": {"color": color},
            },
        })
    if return_position is not None:
        home_status = (
            "completed" if mission_complete
            else "active" if return_active
            else "pending"
        )
        home_color = {"completed": GREEN, "active": ORANGE, "pending": BLUE}[home_status]
        features.append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [return_position["lon"], return_position["lat"]]},
            "properties": {
                "name": "Home",
                "metadata": {
                    "status": home_status,
                    "lat": str(return_position["lat"]),
                    "lon": str(return_position["lon"]),
                },
                "style": {"color": home_color},
            },
        })
    return json.dumps({"type": "FeatureCollection", "features": features})


def _line(coords, color):
    return {
        "type": "Feature",
        "geometry": {"type": "LineString", "coordinates": coords},
        "properties": {"style": {"color": color, "dashArray": DASH}},
    }


def _point_color(index, current_index, failed, active=False):
    if failed:
        if index < current_index:
            return GREEN
        if index == current_index:
            return RED
        return GRAY
    if index < current_index:
        return GREEN
    if index == current_index and (active or current_index > 0):
        return ORANGE
    return BLUE


def _point_status(index, current_index, failed, active=False):
    if failed:
        if index < current_index:
            return "completed"
        if index == current_index:
            return "failed"
        return "skipped"
    if index < current_index:
        return "completed"
    if index == current_index and (active or current_index > 0):
        return "active"
    return "pending"
