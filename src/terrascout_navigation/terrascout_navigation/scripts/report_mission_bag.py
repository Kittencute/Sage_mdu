#!/usr/bin/env python3
"""Create an auditable mission-result bundle from a ROS 2 MCAP recording.

The report deliberately leaves MIS-20 unevaluated: mission state and odometry alone
cannot reproduce the specification's Task 1 scoring geometry without approved map
coordinates, vehicle footprint, and scoring-distance conversion.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
from zoneinfo import ZoneInfo

import rosbag2_py
from geometry_msgs.msg import Quaternion
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

MISSION_STATUS = {
    0: "IDLE", 1: "RUNNING", 2: "PAUSED", 3: "FINISHED", 4: "FAILED", 5: "ABORTED"
}
COMMAND_STATUS = {
    0: "WAITING", 1: "RUNNING", 2: "PAUSED", 3: "FINISHED", 4: "FAILED", 5: "ABORTED"
}
DEFAULT_FOOTPRINT = ((0.465, 0.35), (0.465, -0.35), (-0.465, -0.35), (-0.465, 0.35))
SWEDISH_TIMEZONE = ZoneInfo("Europe/Stockholm")


def time_ns(message):
    return int(message.sec) * 1_000_000_000 + int(message.nanosec)


def recording_datetime_tag(bag: Path):
    match = re.search(r"rosbag2_(\d{4})_(\d{2})_(\d{2})-(\d{2})_(\d{2})_(\d{2})", str(bag))
    if not match:
        return datetime.now(SWEDISH_TIMEZONE).strftime("%Y%m%dT%H%M%S")
    start = datetime(*(int(value) for value in match.groups()), tzinfo=timezone.utc)
    return start.astimezone(SWEDISH_TIMEZONE).strftime("%Y%m%dT%H%M%S")


def load_states(bag: Path, topic: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in types:
        raise RuntimeError(f"Mission-state topic {topic!r} not recorded; available topics: {sorted(types)}")
    msg_type = get_message(types[topic])
    states = []
    while reader.has_next():
        name, serialized, received_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(serialized, msg_type)
        # Use rosbag receive time consistently with optional telemetry streams.
        states.append((received_ns, msg))
    states.sort(key=lambda item: item[0])
    if not states:
        raise RuntimeError(f"No messages found on {topic}")
    return states


def load_metric_samples(bag: Path, topics: list[str]):
    """Read optional recorded topics; absent streams are reported as unavailable."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    requested = {topic: type_map[topic] for topic in topics if topic in type_map}
    message_types = {}
    for topic, msg_type in requested.items():
        try:
            message_types[topic] = get_message(msg_type)
        except (ImportError, AttributeError, ModuleNotFoundError):
            # foxglove_msgs/GeoJSON has the same single-string CDR layout as std_msgs/String.
            if msg_type == "foxglove_msgs/msg/GeoJSON":
                message_types[topic] = String
            else:
                raise
    samples = {topic: [] for topic in requested}
    counts = {topic: 0 for topic in topics}
    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic not in samples:
            continue
        message = deserialize_message(serialized, message_types[topic])
        samples[topic].append((bag_time_ns, message))
        counts[topic] += 1
    return samples, {topic: type_map.get(topic) for topic in topics}, counts


def filter_samples_to_window(samples, start_ns, end_ns):
    return {
        topic: [(stamp, message) for stamp, message in messages
                if start_ns <= stamp <= end_ns]
        for topic, messages in samples.items()
    }


def validate_timestamp_streams(samples, required_groups, start_ns, end_ns,
                               expected_no_overlap_groups=()):
    """Validate nonempty streams and temporal overlap on the bag receive-time axis."""
    streams = {}
    missing_groups = []
    no_overlap = []
    for group_name, topic_alternatives in required_groups.items():
        available = []
        for topic in topic_alternatives:
            entries = samples.get(topic, [])
            if entries:
                stamps = [stamp for stamp, _ in entries]
                available.append({"topic": topic, "count": len(stamps),
                                  "start_ns": min(stamps), "end_ns": max(stamps)})
        streams[group_name] = available
        if not available:
            missing_groups.append(group_name)
        elif not any(
            item["end_ns"] >= start_ns and item["start_ns"] <= end_ns
            and any(start_ns <= stamp <= end_ns for stamp, _ in samples[item["topic"]])
            for item in available
        ):
            no_overlap.append(group_name)
    expected_no_overlap = sorted(set(no_overlap) & set(expected_no_overlap_groups))
    unexpected_no_overlap = sorted(set(no_overlap) - set(expected_no_overlap_groups))
    status = "PASS" if not missing_groups and not unexpected_no_overlap else "FAIL"
    if status == "PASS" and expected_no_overlap:
        status = "PASS_WITH_EXPECTED_NO_OVERLAP"
    return {
        "time_base": "rosbag_receive_time",
        "mission_window_start_ns": start_ns,
        "mission_window_end_ns": end_ns,
        "required_streams_have_samples": not missing_groups,
        "required_streams_within_recording_window": not unexpected_no_overlap,
        "missing_stream_groups": missing_groups,
        "streams_without_mission_window_overlap": no_overlap,
        "expected_no_overlap_groups": expected_no_overlap,
        "unexpected_no_overlap_groups": unexpected_no_overlap,
        "streams": streams,
        "status": status,
    }


def destination_failure_reason(destination, outcomes_by_command, mission_failure_reason=""):
    command_id = destination.get("command_id")
    outcome = outcomes_by_command.get(command_id)
    if not outcome or outcome.get("status") not in ("FAILED", "ABORTED"):
        return "Destination failed; no matching command failure reason was recorded."
    return outcome.get("failure_reason") or mission_failure_reason or (
        "Destination failed; its command has no detailed failure reason."
    )


def point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.hypot(point[0] - projection[0], point[1] - projection[1])


def point_polygon_distance(point, polygon):
    """Distance from a point to a polygon boundary; zero if inside the footprint."""
    inside = False
    x, y = point
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[i - 1]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
    if inside:
        return 0.0
    return min(point_segment_distance(point, polygon[i], polygon[(i + 1) % len(polygon)])
               for i in range(len(polygon)))


def quaternion_yaw(q: Quaternion):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def extract_geojson_destinations(messages, end_ns=None):
    """Return the latest waypoint snapshot no later than the selected mission end."""
    if not messages:
        return []
    eligible = [item for item in messages if end_ns is None or item[0] <= end_ns]
    if not eligible:
        return []
    _, msg = max(eligible, key=lambda item: item[0])
    raw = msg.data if hasattr(msg, "data") else msg.geojson
    collection = json.loads(raw)
    destinations = []
    for feature in collection.get("features", []):
        geometry = feature.get("geometry", {})
        if geometry.get("type") != "Point":
            continue
        properties = feature.get("properties", {})
        metadata = properties.get("metadata", {})
        coordinates = geometry.get("coordinates", [])
        if len(coordinates) < 2:
            continue
        destinations.append({
            "name": properties.get("name", f"Destination {len(destinations) + 1}"),
            "latitude": float(metadata.get("lat", coordinates[1])),
            "longitude": float(metadata.get("lon", coordinates[0])),
            "status": metadata.get("status", "unknown"),
            "command_id": int(metadata["command_id"]) if "command_id" in metadata else None,
        })
    return destinations


def summarize_initial_ordering(metadata, requested, selected):
    """Describe the submitted route decision without implying a runtime reorder."""
    mode = str(metadata.get("ordering_mode", "unknown"))
    requested_points = requested.get("waypoints", []) if isinstance(requested, dict) else []
    requested_names = [str(point.get("name", f"WP{index + 1}")) for index, point in enumerate(requested_points)]
    selected_names = []
    unused = list(requested_points)
    for point in selected:
        match_index = next((index for index, candidate in enumerate(unused)
                            if len(candidate.get("coordinates", [])) >= 2
                            and abs(float(candidate["coordinates"][0]) - point["latitude"]) < 1e-7
                            and abs(float(candidate["coordinates"][1]) - point["longitude"]) < 1e-7), None)
        if match_index is not None:
            selected_names.append(str(unused.pop(match_index).get("name", f"WP{len(selected_names) + 1}")))
    if mode == "unordered":
        algorithm = "exhaustive shortest closed tour using straight-line geographic distances"
        reason = (
            "At submission, evaluate every waypoint permutation by straight-line distance "
            "from the robot start through the destinations and back to mission start. "
            "This is not Nav2 planned-path length or obstacle-aware reachability."
        )
    elif mode == "ordered":
        algorithm = "input order"
        reason = "Preserve the waypoint order provided by the operator."
    else:
        algorithm = "unknown"
        reason = "Ordering mode was not available in the mission bundle metadata."
    return {
        "ordering_mode": mode,
        "ordering_algorithm": algorithm,
        "requested_waypoints": requested_names,
        "selected_waypoints": selected_names or None,
        "selection_reason": reason,
        "order_changes": [],
        "note": "This is the initial submission decision; runtime order changes are reported separately.",
    }


def minimum_footprint_distances(destinations, ground_truth, datum, footprint=DEFAULT_FOOTPRINT,
                                map_yaw_offset_deg=0.0, start_ns=None, end_ns=None):
    """Score destinations in local map metres using recorded GT and Nav2 footprint."""
    if not destinations or not ground_truth or datum is None:
        return None
    datum_lat, datum_lon, datum_alt = datum
    datum_ecef = geodetic_to_ecef(datum_lat, datum_lon, datum_alt)
    datum_lat_rad, datum_lon_rad = math.radians(datum_lat), math.radians(datum_lon)
    yaw_offset = math.radians(map_yaw_offset_deg)
    cos_offset, sin_offset = math.cos(yaw_offset), math.sin(yaw_offset)
    target_xy = {}
    for destination in destinations:
        point_ecef = geodetic_to_ecef(destination["latitude"], destination["longitude"], datum_alt)
        dx, dy, dz = (point_ecef[i] - datum_ecef[i] for i in range(3))
        east = -math.sin(datum_lon_rad) * dx + math.cos(datum_lon_rad) * dy
        north = (-math.sin(datum_lat_rad) * math.cos(datum_lon_rad) * dx
             - math.sin(datum_lat_rad) * math.sin(datum_lon_rad) * dy
             + math.cos(datum_lat_rad) * dz)
        target_xy[destination["name"]] = (
            cos_offset * east + sin_offset * north,
            -sin_offset * east + cos_offset * north,
        )
    distances = {name: float("inf") for name in target_xy}
    sample_count = 0
    for stamp, msg in ground_truth:
        if start_ns is not None and stamp < start_ns:
            continue
        if end_ns is not None and stamp > end_ns:
            continue
        pose = msg.pose.pose
        px, py = pose.position.x, pose.position.y
        yaw = quaternion_yaw(pose.orientation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        world_footprint = tuple((px + cosine * x - sine * y, py + sine * x + cosine * y)
                                for x, y in footprint)
        for name, target in target_xy.items():
            distances[name] = min(distances[name], point_polygon_distance(target, world_footprint))
        sample_count += 1
    if sample_count == 0:
        return None
    return {name: distance for name, distance in distances.items()}, sample_count


def geodetic_to_ecef(latitude, longitude, altitude):
    """Convert WGS84 geodetic coordinates to ECEF metres (no external GIS package)."""
    semi_major = 6378137.0
    eccentricity_sq = 6.69437999014e-3
    lat, lon = math.radians(latitude), math.radians(longitude)
    normal = semi_major / math.sqrt(1.0 - eccentricity_sq * math.sin(lat) ** 2)
    return (
        (normal + altitude) * math.cos(lat) * math.cos(lon),
        (normal + altitude) * math.cos(lat) * math.sin(lon),
        (normal * (1.0 - eccentricity_sq) + altitude) * math.sin(lat),
    )


def summarize_motion(samples, odom_topic, cmd_vel_topic, teleop_topic, safety_topic):
    """Calculate only metrics directly observable in the recorded streams."""
    available = []
    missing = []
    summary = {
        "distance_driven_m": None,
        "stop_events": None,
        "teleop_takeover_events": None,
        "safety_stop_events": None,
    }
    if odom_topic in samples:
        poses = []
        for stamp, msg in samples[odom_topic]:
            p = msg.pose.pose.position
            poses.append((stamp, float(p.x), float(p.y)))
        poses.sort()
        summary["distance_driven_m"] = sum(
            ((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
            for a, b in zip(poses, poses[1:])
        )
        available.append("distance_driven_m (integrated odometry path length)")
    else:
        missing.append("distance_driven_m: odometry topic missing")

    if cmd_vel_topic in samples:
        moving = False
        stop_events = 0
        for _, msg in sorted(samples[cmd_vel_topic], key=lambda item: item[0]):
            twist = msg.twist if hasattr(msg, "twist") else msg
            linear = twist.linear
            angular = twist.angular
            is_moving = (linear.x * linear.x + linear.y * linear.y) ** 0.5 > 0.02 or abs(angular.z) > 0.02
            if moving and not is_moving:
                stop_events += 1
            moving = is_moving
        summary["stop_events"] = stop_events
        available.append("stop_events (moving-to-zero cmd_vel transitions)")
    else:
        missing.append("stop_events: cmd_vel topic missing")

    for topic, key, label in (
        (teleop_topic, "teleop_takeover_events", "teleop_takeover"),
        (safety_topic, "safety_stop_events", "platform/safety_stop"),
    ):
        if topic not in samples:
            missing.append(f"{key}: {label} topic missing")
            continue
        messages = sorted(samples[topic], key=lambda item: item[0])
        previous = False
        count = 0
        for _, msg in messages:
            active = bool(msg.data)
            if active and not previous:
                count += 1
            previous = active
        summary[key] = count
        available.append(f"{key} (rising Bool transitions)")
    return summary, available, missing


def count_rising_edges(samples, topic):
    messages = sorted(samples.get(topic, []), key=lambda item: item[0])
    previous = False
    count = 0
    for _, message in messages:
        active = bool(message.data)
        count += int(active and not previous)
        previous = active
    return count


def summarize_faults(samples, rosout_topic):
    """Count recorded ROS error/fatal log entries; not a substitute for fault taxonomy."""
    if rosout_topic not in samples:
        return None, f"fault_events: {rosout_topic} missing"
    faults = []
    for stamp, message in sorted(samples[rosout_topic], key=lambda item: item[0]):
        if int(getattr(message, "level", 0)) >= 40:
            faults.append({
                "timestamp_rosbag_ns": stamp,
                "name": str(getattr(message, "name", "")),
                "message": str(getattr(message, "msg", "")),
                "level": int(message.level),
            })
    return faults, None


def write_events(states, path: Path, mission_id: int | None):
    fields = ["timestamp_rosbag_ns", "mission_id", "mission_status", "current_command_id",
              "selected_order", "command_id", "command_status", "command_failure_reason",
              "failure_reason", "order_change_reason", "ready", "not_ready_reason", "event"]
    events = []
    previous = None
    for stamp, msg in states:
        if mission_id is not None and msg.mission_id != mission_id:
            continue
        current_status = {int(cid): int(status) for cid, status in zip(msg.command_ids, msg.command_status)}
        selected_order = [int(cid) for cid in msg.command_ids]
        failure_reasons = list(getattr(msg, "command_failure_reasons", []))
        reasons_by_command = {
            int(cid): str(failure_reasons[index])
            for index, cid in enumerate(msg.command_ids)
            if index < len(failure_reasons)
        }
        if previous is None or msg.mission_id != previous[0]:
            events.append({"timestamp_rosbag_ns": stamp, "mission_id": msg.mission_id,
                           "mission_status": MISSION_STATUS.get(msg.mission_status, str(msg.mission_status)),
                           "current_command_id": msg.current_command_id,
                           "selected_order": json.dumps(selected_order), "command_id": "", "command_status": "",
                           "command_failure_reason": "", "failure_reason": getattr(msg, "failure_reason", ""),
                           "order_change_reason": getattr(msg, "order_change_reason", ""),
                           "ready": msg.ready, "not_ready_reason": msg.not_ready_reason, "event": "mission_snapshot"})
        elif msg.mission_status != previous[1]:
            events.append({"timestamp_rosbag_ns": stamp, "mission_id": msg.mission_id,
                           "mission_status": MISSION_STATUS.get(msg.mission_status, str(msg.mission_status)),
                           "current_command_id": msg.current_command_id,
                           "selected_order": json.dumps(selected_order), "command_id": "", "command_status": "",
                           "command_failure_reason": "", "failure_reason": getattr(msg, "failure_reason", ""),
                           "order_change_reason": getattr(msg, "order_change_reason", ""),
                           "ready": msg.ready, "not_ready_reason": msg.not_ready_reason, "event": "mission_status_changed"})
        if previous is not None and msg.mission_id == previous[0] and tuple(selected_order) != previous[3]:
            events.append({"timestamp_rosbag_ns": stamp, "mission_id": msg.mission_id,
                           "mission_status": MISSION_STATUS.get(msg.mission_status, str(msg.mission_status)),
                           "current_command_id": msg.current_command_id,
                           "selected_order": json.dumps(selected_order), "command_id": "", "command_status": "",
                           "command_failure_reason": "", "failure_reason": getattr(msg, "failure_reason", ""),
                           "order_change_reason": getattr(msg, "order_change_reason", ""),
                           "ready": msg.ready, "not_ready_reason": msg.not_ready_reason,
                           "event": "order_changed"})
        for cid, status in current_status.items():
            if previous is None or msg.mission_id != previous[0] or previous[2].get(cid) != status:
                events.append({"timestamp_rosbag_ns": stamp, "mission_id": msg.mission_id,
                               "mission_status": MISSION_STATUS.get(msg.mission_status, str(msg.mission_status)),
                               "current_command_id": msg.current_command_id,
                               "selected_order": json.dumps(selected_order), "command_id": cid,
                               "command_status": COMMAND_STATUS.get(status, str(status)),
                               "command_failure_reason": reasons_by_command.get(cid, ""),
                               "failure_reason": getattr(msg, "failure_reason", ""),
                               "order_change_reason": getattr(msg, "order_change_reason", ""), "ready": msg.ready,
                               "not_ready_reason": msg.not_ready_reason, "event": "command_status_changed"})
        previous = (msg.mission_id, msg.mission_status, current_status, tuple(selected_order))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(events)
    return events


def split_mission_instances(states):
    """Group each mission ID's states into runs, including interleaved missions."""
    rows_by_id = {}
    for row in states:
        stamp, msg = row
        mission_id = int(msg.mission_id)
        status = int(msg.mission_status)
        runs = rows_by_id.setdefault(mission_id, [])
        starts_new = runs and int(runs[-1][-1][1].mission_status) in (3, 4, 5) and status == 1
        if not runs or starts_new:
            runs.append([])
        runs[-1].append(row)
    instances = []
    for mission_id, runs in rows_by_id.items():
        for instance, run_states in enumerate(runs, start=1):
            instances.append({"mission_id": mission_id, "instance": instance, "states": run_states})
    return sorted(instances, key=lambda entry: entry["states"][0][0])


def render_readable_report(report):
    """Render a concise Markdown summary for operators; JSON remains machine-readable."""
    mis22 = report["mis22"]
    mis21 = report["mis21"]
    metrics = mis22["metrics"]
    title = report.get("mission_label") or f"Mission {report['mission_id']}"
    lines = [
        f"# {title}",
        "",
        f"**Outcome:** {report['final_mission_status']}  ",
        f"**Duration:** {report['duration_s']:.1f} s  ",
        "",
        "## Mission results",
        "",
        "| Measure | Result |",
        "|---|---:|",
        f"| Distance driven | {metrics.get('distance_driven_m', 'not available')} m |",
        f"| Stop events | {metrics.get('stop_events', 'not available')} |",
        f"| Operator takeovers | {metrics.get('teleop_takeover_events', 'not available')} |",
        f"| Safety-stop events | {metrics.get('safety_stop_events', 'not available')} |",
        "",
        "## Destination outcomes",
        "",
        "| Destination | Outcome | Minimum distance | Explanation |",
        "|---|---|---:|---|",
    ]
    distances = {item.get("command_id"): item.get("minimum_vehicle_edge_distance_m")
                 for item in mis22.get("minimum_destination_edge_distances", [])}
    for item in mis22.get("destination_outcomes", []):
        distance = distances.get(item.get("command_id"))
        distance_text = f"{distance:.2f} m" if isinstance(distance, (int, float)) else "not available"
        explanation = str(item.get("outcome_reason", "—")).replace("|", "\\|").replace("\n", " ")
        lines.append(f"| {item.get('name', 'Destination')} | {item.get('status', 'unknown')} | {distance_text} | {explanation} |")
    lines.extend(["", "## Route selection and changes", ""])
    initial = mis22.get("initial_ordering_decision") or {}
    lines.append(f"- Initial choice: {initial.get('ordering_algorithm', 'not recorded')} — {initial.get('selection_reason', 'No reason recorded.')}")
    order_changes = mis22.get("order_changes", [])
    if order_changes:
        for change in order_changes:
            order = ", ".join(str(value) for value in change.get("selected_order", [])) or "not recorded"
            lines.append(f"- At bag time {change.get('timestamp_rosbag_ns', 'unknown')} ns, selected order became {order}. Reason: {change.get('reason', 'No reason recorded.')}")
    else:
        lines.append("- No runtime order changes recorded.")
    lines.extend(["", "## Odometry and motion plots", ""])
    artifacts = report.get("artifacts", {})
    plot_links = [("Trajectory comparison", artifacts.get("trajectory_comparison")),
                  ("Position error", artifacts.get("position_error")),
                  ("Orientation error", artifacts.get("orientation_error"))]
    available_plots = [(label, path) for label, path in plot_links if path]
    if available_plots:
        lines.extend(f"- [{label}]({path})" for label, path in available_plots)
    else:
        lines.append("No odometry comparison plots were generated for this recording.")
    lines.extend(["", "## Recording and faults", ""])
    validation = mis21.get("timestamp_validation", {})
    lines.append(f"- Required data capture check: {validation.get('status', 'not evaluated')}.")
    expected_streams = validation.get("expected_no_overlap_groups", [])
    if expected_streams:
        lines.append("- No messages were expected during this run for: " + ", ".join(expected_streams) + ".")
    lines.append(f"- Fault records: {len(mis22.get('fault_events', []))}.")
    for fault in mis22.get("fault_events", []):
        message = str(fault.get("message", "Fault details unavailable.")).replace("|", "\\|").replace("\n", " ")
        lines.append(f"  - {message}")
    caveats = report.get("caveats", [])
    if caveats:
        lines.extend(["", "## Notes", ""])
        lines.extend(f"- {note}" for note in caveats)
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 MCAP bag directory")
    parser.add_argument("--out-dir", help="Existing mission result bundle directory; defaults to the newest tester-created mission folder")
    parser.add_argument("--state-topic", default="/terrascout1/mission_state")
    parser.add_argument("--mission-id", type=int, help="Select a mission ID; defaults to the latest mission in the bag")
    parser.add_argument("--mission-instance", type=int, help="Select a run instance for that mission ID; defaults to the latest instance")
    parser.add_argument("--route", type=Path, help="Copy the route/waypoint JSON into the bundle")
    parser.add_argument("--odom-topic", default="/terrascout1/odom")
    parser.add_argument("--cmd-vel-topic", default="/terrascout1/cmd_vel")
    parser.add_argument("--teleop-topic", default="/terrascout1/teleop_takeover")
    parser.add_argument("--safety-topic", default="/terrascout1/platform/safety_stop")
    parser.add_argument("--autonomy-lock-topic", default="/terrascout1/autonomy_lock")
    parser.add_argument("--estop-topic", default="/terrascout1/estop")
    parser.add_argument("--estop-lock-topic", default="/terrascout1/estop_lock")
    parser.add_argument("--rosout-topic", default="/rosout")
    parser.add_argument("--ground-truth-topic", default="/terrascout1/odom_gt")
    parser.add_argument("--datum-topic", default="/terrascout1/fixposition/datum")
    parser.add_argument("--waypoints-topic", default="/terrascout1/waypoints_geo")
    parser.add_argument("--map-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument(
        "--footprint", type=json.loads, default=DEFAULT_FOOTPRINT,
        help='JSON polygon in base_footprint metres, default matches Nav2: [[0.465,0.35],[0.465,-0.35],[-0.465,-0.35],[-0.465,0.35]]',
    )
    args = parser.parse_args()
    if args.out_dir is not None and not args.out_dir.strip():
        parser.error("--out-dir cannot be empty; pass a mission result folder")
    if args.out_dir is None:
        workspace_root = Path(__file__).resolve().parents[4]
        results_root = workspace_root / "src" / "terrascout_mission_tester" / "mission_results"
        candidates = sorted(
            (path for path in results_root.glob("mission_*") if path.is_dir()),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        if not candidates:
            args.out_dir = results_root / f"mission_{recording_datetime_tag(args.bag)}"
            print(f"No existing mission bundle found; creating one from recording timestamp: {args.out_dir}")
        else:
            args.out_dir = candidates[0]
    else:
        args.out_dir = Path(args.out_dir)
    states = load_states(args.bag, args.state_topic)
    metrics, topic_types, topic_counts = load_metric_samples(
        args.bag, [args.odom_topic, args.cmd_vel_topic, args.teleop_topic, args.safety_topic,
                   args.state_topic, args.autonomy_lock_topic, args.estop_topic,
                   args.estop_lock_topic, args.rosout_topic,
                   args.ground_truth_topic, args.datum_topic, args.waypoints_topic,
                   "/terrascout1/keepout_obstacle_points", "/terrascout1/keepout_zone_marker",
                   "/terrascout1/plan", "/terrascout1/trajectories",
                   "/terrascout1/fixposition/odometry_enu",
                   "/terrascout1/fixposition/odometry_llh", "/terrascout1/fast_lio/odometry",
                   "/terrascout1/obstacle_points", "/terrascout1/ground_points"]
    )
    mission_instances = split_mission_instances(states)
    available_ids = sorted({entry["mission_id"] for entry in mission_instances if entry["mission_id"] > 0})
    if not available_ids:
        raise RuntimeError("No positive mission IDs found in recorded state")
    mission_id = args.mission_id if args.mission_id is not None else available_ids[-1]
    if mission_id not in available_ids:
        raise RuntimeError(f"Mission ID {mission_id} not found; recorded IDs: {available_ids}")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if args.route and not (args.out_dir / "waypoints.json").exists():
        (args.out_dir / "waypoints.json").write_bytes(args.route.read_bytes())
    matching_instances = [entry for entry in mission_instances if entry["mission_id"] == mission_id]
    if args.mission_instance is not None:
        selected_instance = next((entry for entry in matching_instances
                                  if entry["instance"] == args.mission_instance), None)
        if selected_instance is None:
            raise RuntimeError(
                f"Mission {mission_id} instance {args.mission_instance} not found; "
                f"available instances: {[entry['instance'] for entry in matching_instances]}"
            )
    else:
        selected_instance = matching_instances[-1]
    selected = selected_instance["states"]
    events = write_events(selected, args.out_dir / "mission_events.csv", mission_id)
    first_stamp, last_stamp = selected[0][0], selected[-1][0]
    final = selected[-1][1]
    final_failure_reasons = list(getattr(final, "command_failure_reasons", []))
    command_outcomes = []
    for index, (cid, status) in enumerate(zip(final.command_ids, final.command_status)):
        command_outcomes.append({
            "command_id": int(cid),
            "status": COMMAND_STATUS.get(status, str(status)),
            "failure_reason": final_failure_reasons[index]
            if index < len(final_failure_reasons) else "",
        })
    failure_text = " ".join([
        str(getattr(final, "failure_reason", "")),
        *[str(item.get("failure_reason", "")) for item in command_outcomes],
    ]).lower()
    expected_empty_navigation_streams = (
        final.mission_status == 4
        and "compute_path_to_pose" in failure_text
        and "FAILED" in [item["status"] for item in command_outcomes]
    )
    expected_no_overlap_groups = (
        ("navigation path", "commands") if expected_empty_navigation_streams else ()
    )
    timestamp_validation = validate_timestamp_streams(metrics, {
        "mission state": (args.state_topic,),
        "navigation path": ("/terrascout1/plan", "/terrascout1/trajectories"),
        "keepout areas": ("/terrascout1/keepout_obstacle_points", "/terrascout1/keepout_zone_marker"),
        "commands": (args.cmd_vel_topic,),
        "mode/intervention": (args.autonomy_lock_topic, args.teleop_topic),
        "fault logs": (args.rosout_topic,),
        "position": (args.odom_topic, args.ground_truth_topic),
        "localization": ("/terrascout1/fixposition/odometry_enu", "/terrascout1/fast_lio/odometry"),
        "obstacle data": ("/terrascout1/obstacle_points", "/terrascout1/ground_points"),
        "e-stop state": (args.estop_topic, args.estop_lock_topic),
    }, first_stamp, last_stamp, expected_no_overlap_groups)
    mission_metrics = filter_samples_to_window(metrics, first_stamp, last_stamp)
    motion_summary, available_metrics, missing_metrics = summarize_motion(
        mission_metrics, args.odom_topic, args.cmd_vel_topic, args.teleop_topic, args.safety_topic
    )
    completed = [int(cid) for cid, status in zip(final.command_ids, final.command_status) if status == 3]
    destinations = extract_geojson_destinations(
        mission_metrics.get(args.waypoints_topic, []), end_ns=last_stamp
    )
    bundle_waypoints = {}
    bundle_metadata = {}
    waypoints_path = args.out_dir / "waypoints.json"
    metadata_path = args.out_dir / "metadata.json"
    if waypoints_path.exists():
        try:
            bundle_waypoints = json.loads(waypoints_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            bundle_waypoints = {}
    if metadata_path.exists():
        try:
            bundle_metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            bundle_metadata = {}
    initial_ordering = summarize_initial_ordering(bundle_metadata, bundle_waypoints, destinations)
    if destinations and not (args.out_dir / "waypoints.json").exists():
        route_snapshot = {
            "source": args.waypoints_topic,
            "coordinate_reference": "WGS84 latitude/longitude",
            "destinations": destinations,
        }
        (args.out_dir / "waypoints.json").write_text(json.dumps(route_snapshot, indent=2) + "\n", encoding="utf-8")
    completed_destinations = [destination for destination in destinations
                              if destination["status"].lower() == "completed"]
    datum_msgs = metrics.get(args.datum_topic, [])
    datum = None
    if datum_msgs:
        datum_msg = min(datum_msgs, key=lambda item: item[0])[1]
        if math.isfinite(datum_msg.latitude) and math.isfinite(datum_msg.longitude):
            datum = (datum_msg.latitude, datum_msg.longitude, datum_msg.altitude)
    ground_truth = mission_metrics.get(args.ground_truth_topic, [])
    footprint_score = minimum_footprint_distances(
        completed_destinations, ground_truth, datum, args.footprint, args.map_yaw_offset_deg,
        first_stamp, last_stamp,
    )
    if footprint_score:
        distances, gt_samples_used = footprint_score
        destination_scores = [
            {**destination, "minimum_vehicle_edge_distance_m": distances[destination["name"]],
             "within_1m": distances[destination["name"]] <= 1.0}
            for destination in completed_destinations
        ]
        spatial_pass = bool(destination_scores) and all(item["within_1m"] for item in destination_scores)
        time_pass = (last_stamp - first_stamp) / 1e9 <= 1200.0
        mis20_pass = final.mission_status == 3 and spatial_pass and time_pass
        mis20 = {
            "pass": mis20_pass,
            "status": "PASS" if mis20_pass else "FAIL",
            "duration_limit_s": 1200.0,
            "time_pass": time_pass,
            "all_completed_destinations_within_1m": spatial_pass,
            "destination_scores": destination_scores,
            "ground_truth_samples_used": gt_samples_used,
            "vehicle_footprint_base_link_xy_m": args.footprint,
            "map_yaw_offset_deg": args.map_yaw_offset_deg,
            "distance_method": "minimum planar Euclidean distance from destination point to rotated vehicle footprint polygon, zero when point lies inside polygon",
            "coordinate_method": "WGS84 destination converted to local ENU at recorded datum, then rotated into map coordinates; assumes /odom_gt odom axes share the map origin and yaw convention",
            "trial_scope": "simulation scoring check; not an assessed-trial or physical-robot pass",
        }
    else:
        why = []
        if not completed_destinations:
            why.append("No completed waypoint points were available in the recorded waypoint GeoJSON.")
        if datum is None:
            why.append(f"No valid datum was recorded on {args.datum_topic}.")
        if not ground_truth:
            why.append(f"No ground-truth poses were recorded on {args.ground_truth_topic}.")
        mis20 = {"pass": None, "status": "NOT_EVALUATED", "reason": " ".join(why)}
    all_destination_score = minimum_footprint_distances(
        destinations, ground_truth, datum, args.footprint, args.map_yaw_offset_deg,
        first_stamp, last_stamp,
    )
    if all_destination_score:
        all_distances, _ = all_destination_score
        destination_distances = [
            {**destination,
             "minimum_vehicle_edge_distance_m": all_distances[destination["name"]]}
            for destination in destinations
        ]
    else:
        destination_distances = []
    outcomes_by_command = {item["command_id"]: item for item in command_outcomes}
    fault_events, fault_missing = summarize_faults(mission_metrics, args.rosout_topic)
    autonomy_lock_events = count_rising_edges(mission_metrics, args.autonomy_lock_topic)
    estop_events = count_rising_edges(mission_metrics, args.estop_topic)
    estop_lock_events = count_rising_edges(mission_metrics, args.estop_lock_topic)
    order_events = [event for event in events if event["event"] == "order_changed"]
    order_changes = [{
        "timestamp_rosbag_ns": event["timestamp_rosbag_ns"],
        "selected_order": json.loads(event["selected_order"]),
        "reason": event["order_change_reason"] or "No reason was published for this order change.",
    } for event in order_events]
    for index, destination in enumerate(destinations):
        status = str(destination.get("status", "unknown")).lower()
        if status == "completed":
            reason = "Destination reached successfully."
        elif status == "failed":
            reason = destination_failure_reason(
                destination, outcomes_by_command, getattr(final, "failure_reason", "")
            )
        elif status == "active":
            reason = getattr(final, "failure_reason", "") or "Destination was active when this recording ended."
        else:
            reason = "Destination was not reached before this recording ended."
        destination["outcome_reason"] = reason
    if fault_missing:
        missing_metrics.append(fault_missing)
    available_metrics.extend([
        "selected-order history from mission-state snapshots",
        "autonomy lock transitions" if args.autonomy_lock_topic in metrics else "autonomy lock stream unavailable",
        "e-stop activations" if args.estop_topic in metrics else "e-stop stream unavailable",
        "e-stop lock transitions" if args.estop_lock_topic in metrics else "e-stop lock stream unavailable",
    ])
    required_mis21_topics = {
        args.state_topic,
        args.cmd_vel_topic, args.autonomy_lock_topic, args.rosout_topic,
        args.teleop_topic, args.safety_topic, args.odom_topic, args.ground_truth_topic,
        args.datum_topic, "/terrascout1/fixposition/odometry_enu",
        "/terrascout1/fixposition/odometry_llh", "/terrascout1/fast_lio/odometry",
    }
    actually_recorded_topics = {topic for topic, count in topic_counts.items() if count > 0}
    missing_mis21_topics = sorted(required_mis21_topics - actually_recorded_topics)
    if not topic_counts.get("/terrascout1/plan", 0) and not topic_counts.get("/terrascout1/trajectories", 0):
        missing_mis21_topics.append("navigation path data: neither /terrascout1/plan nor /terrascout1/trajectories had messages")
    if not topic_counts.get("/terrascout1/keepout_obstacle_points", 0) and not topic_counts.get("/terrascout1/keepout_zone_marker", 0):
        missing_mis21_topics.append("keepout area data: neither keepout obstacle points nor markers had messages")
    if not topic_counts.get("/terrascout1/obstacle_points", 0) and not topic_counts.get("/terrascout1/ground_points", 0):
        missing_mis21_topics.append("obstacle data: neither obstacle_points nor ground_points had messages")
    if args.estop_topic not in actually_recorded_topics and args.estop_lock_topic not in actually_recorded_topics:
        missing_mis21_topics.append(
            f"e-stop state missing: neither {args.estop_topic} nor {args.estop_lock_topic} was recorded"
        )
    if missing_mis21_topics:
        missing_metrics.extend(f"MIS-21 required topic missing: {topic}" for topic in missing_mis21_topics)
    timestamp_validation["topic_message_counts"] = {
        topic: topic_counts.get(topic, 0) for topic in sorted(topic_counts)
    }
    timestamp_validation["required_streams_have_samples"] = (
        timestamp_validation["required_streams_have_samples"] and not missing_mis21_topics
    )
    if not timestamp_validation["required_streams_have_samples"]:
        timestamp_validation["status"] = "FAIL"
    elif not timestamp_validation["required_streams_within_recording_window"]:
        timestamp_validation["status"] = "FAIL"
    if timestamp_validation["status"] == "FAIL":
        missing_metrics.append("MIS-21 timestamp/sample validation failed")
    terminal = final.mission_status in (3, 4, 5)
    outcome_reasons_present = bool(destinations) and all(
        destination.get("outcome_reason") for destination in destinations
    )
    mis22_complete = terminal and not missing_metrics and outcome_reasons_present and bool(destination_distances)
    report = {
        "schema_version": 1,
        "mission_id": mission_id,
        "mission_instance": selected_instance["instance"],
        "mission_run_id": f"mission-{mission_id}-run-{selected_instance['instance']}-start-{first_stamp}ns",
        "mission_label": f"Mission {mission_id} — run {selected_instance['instance']} (bag time {first_stamp} ns)",
        "recording": str(args.bag.resolve()),
        "state_topic": args.state_topic,
        "recorded_mission_ids": available_ids,
        "started_at_rosbag_ns": first_stamp,
        "ended_at_rosbag_ns": last_stamp,
        "time_base": "rosbag_receive_time; timestamps are not necessarily Unix epoch time",
        "duration_s": (last_stamp - first_stamp) / 1e9,
        "final_mission_status": MISSION_STATUS.get(final.mission_status, str(final.mission_status)),
        "final_commands": command_outcomes,
        "mission_failure_reason": getattr(final, "failure_reason", ""),
        "completed_command_ids": completed,
        "mission_event_rows": len(events),
        "artifacts": {
            key: matches[0].name if (matches := sorted(args.out_dir.glob(pattern))) else None
            for key, pattern in {
                "trajectory_comparison": "*_trajectory_comparison.png",
                "position_error": "*_position_error.png",
                "orientation_error": "*_orientation_error.png",
            }.items()
        },
        "mis20": mis20,
        "mis21": {
            "status": "PARTIAL" if timestamp_validation["status"] == "FAIL" else "RECORDED_STREAMS_PRESENT_UNVERIFIED",
            "bag_time_base": "rosbag receive timestamps used consistently for analyzed streams; values may be simulation-relative, not Unix time",
            "timestamp_validation": timestamp_validation,
            "recorded_topics_used": sorted(actually_recorded_topics | {args.state_topic}),
            "missing_required_topics": missing_mis21_topics,
            "mission_order_and_state_events": "mission_events.csv",
            "caveat": "Recorder presence confirms data capture, not sensor accuracy or clock synchronization quality.",
        },
        "mis22": {"status": "COMPLETE_SUMMARY" if mis22_complete else "INCOMPLETE_SUMMARY",
              "available": ["mission state transitions", "command outcomes and failure reasons",
                            "selected order history", "destination outcomes", "mission duration",
                            *available_metrics],
              "metrics": motion_summary,
              "order_changes": order_changes,
              "initial_ordering_decision": initial_ordering,
              "destination_outcomes": destinations,
              "command_outcomes": command_outcomes,
              "minimum_destination_edge_distances": destination_distances,
              "fault_events": fault_events,
              "interventions": {
                  "teleop_takeover_rising_edges": motion_summary["teleop_takeover_events"],
                  "autonomy_lock_rising_edges": autonomy_lock_events,
                  "soft_estop_rising_edges": estop_events,
                  "estop_lock_rising_edges": estop_lock_events,
                  "platform_safety_stop_rising_edges": motion_summary["safety_stop_events"],
              },
              "not_recorded_or_not_computed": [
                  *([] if destination_distances else ["minimum destination edge distances: required route, datum, ground truth and footprint data missing"]),
                  *([] if terminal else [f"mission was not terminal at recording end (status: {MISSION_STATUS.get(final.mission_status, final.mission_status)})"]),
                  *([] if order_changes else ["no order changes occurred in this mission; order-change reason handling was not exercised"]),
                  *missing_metrics,
              ]},
        "caveats": ["Topic presence and a shared bag receive-time axis do not prove source-clock synchronization accuracy.", "ROS error/fatal log entries are a proxy for faults, not a structured fault taxonomy.", "MIS-20 and destination edge distances assume /odom_gt odom axes align with map axes and use the configured Nav2 footprint polygon.", "A simulation scoring pass is not an assessed-trial or physical-robot pass."],
    }
    (args.out_dir / "mission_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (args.out_dir / "mission_report.md").write_text(render_readable_report(report), encoding="utf-8")
    print(f"Wrote {args.out_dir / 'mission_events.csv'}, {args.out_dir / 'mission_report.json'}, and {args.out_dir / 'mission_report.md'}")


if __name__ == "__main__":
    main()
