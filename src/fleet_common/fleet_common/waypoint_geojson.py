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
):
    features = []
    coords = [[wp["lon"], wp["lat"]] for wp in waypoints]

    # Draw the initial leg independently so waypoint indices remain unchanged.
    if start_position is not None and coords:
        start = [start_position["lon"], start_position["lat"]]
        if start != coords[0]:
            color = GREEN if current_index > 0 else RED if failed else ORANGE if active else BLUE
            features.append(_line([start, coords[0]], color))

    if len(coords) >= 2:
        if failed:
            if current_index > 1:
                features.append(_line(coords[:current_index], GREEN))
            if current_index > 0:
                features.append(_line(coords[current_index - 1:current_index + 1], RED))
            if current_index + 1 < len(coords) - 1:
                features.append(_line(coords[current_index + 1:], GRAY))
        elif current_index == 0:
            features.append(_line(coords, BLUE))
        elif current_index >= len(coords):
            features.append(_line(coords, GREEN))
        else:
            if current_index > 1:
                features.append(_line(coords[:current_index], GREEN))
            features.append(_line(coords[current_index - 1:current_index + 1], ORANGE))
            if current_index < len(coords) - 1:
                features.append(_line(coords[current_index:], BLUE))

    if return_position is not None and coords:
        finish = [return_position["lon"], return_position["lat"]]
        if coords[-1] != finish:
            color = (
                GREEN if mission_complete
                else ORANGE if return_active
                else GRAY if failed
                else BLUE
            )
            features.append(_line([coords[-1], finish], color))

    for i, wp in enumerate(waypoints):
        status = (
            waypoint_statuses[i]
            if waypoint_statuses is not None and i < len(waypoint_statuses)
            else _point_status(i, current_index, failed, active)
        )
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
                },
                "style": {"color": color},
            },
        })
    if return_position is not None:
        home_color = (
            GREEN if mission_complete
            else ORANGE if return_active
            else RED if failed
            else BLUE
        )
        features.append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [return_position["lon"], return_position["lat"]]},
            "properties": {
                "name": "Home",
                "metadata": {
                    "status": "completed" if mission_complete else "active" if return_active else "failed" if failed else "pending",
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
