#!/usr/bin/env python3
"""Create an auditable mission-result bundle from a ROS 2 MCAP recording.

The report deliberately leaves MIS-20 unevaluated: mission state and odometry alone
cannot reproduce the specification's Task 1 scoring geometry without approved map
coordinates, vehicle footprint, and scoring-distance conversion.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
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


def workspace_provenance(workspace: Path):
    """Capture processing-time git and config fingerprints, not historical guarantees."""
    def git_value(*args):
        result = subprocess.run(
            ["git", "-C", str(workspace), *args], capture_output=True, text=True, check=False
        )
        return result.stdout.strip() if result.returncode == 0 else None

    config_files = {}
    for relative in ("src/terrascout_navigation/config/fast_lio_sim.yaml",
                     "src/terrascout_navigation/config/nav2.yaml"):
        path = workspace / relative
        if path.is_file():
            config_files[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": git_value("rev-parse", "HEAD"),
        "git_branch": git_value("branch", "--show-current"),
        "working_tree_dirty": bool(git_value("status", "--porcelain")),
        "configuration_sha256_at_processing_time": config_files,
        "caveat": "These identify the workspace at report-processing time; they do not prove the same revision/configuration was active during recording.",
    }


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


def summarize_keepout_events(samples, rosout_topic):
    """Extract keepout server zone lifecycle logs when rosout captured them."""
    events = []
    for stamp, message in sorted(samples.get(rosout_topic, []), key=lambda item: item[0]):
        name = str(getattr(message, "name", ""))
        text = str(getattr(message, "msg", ""))
        lowered = f"{name} {text}".lower()
        if "keepout" not in lowered and not re.search(r"\bzone\b", lowered):
            continue
        if not re.search(r"\b(created|updated|deleted|restored|restore|removed|clear)\b", lowered):
            continue
        events.append({
            "timestamp_rosbag_ns": int(stamp), "component": name,
            "severity": int(getattr(message, "level", 0)), "message": text,
        })
    return events


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


def summarize_error_csv(path: Path | None, elapsed_window: tuple[float, float] | None = None):
    """Summarize an existing FAST-LIO comparison CSV without inventing data."""
    if path is None or not path.is_file():
        return None
    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                numeric = {key: float(value) for key, value in row.items()
                           if key != "datetime" and value not in (None, "")}
                if elapsed_window is None or elapsed_window[0] <= numeric.get("elapsed_s", -math.inf) <= elapsed_window[1]:
                    rows.append(numeric)
            except (TypeError, ValueError):
                continue
    if not rows:
        return None

    def stats(column):
        values = [row[column] for row in rows if column in row and math.isfinite(row[column])]
        if not values:
            return None
        ordered = sorted(values)

        def percentile(percent):
            index = (len(ordered) - 1) * percent / 100.0
            low, high = math.floor(index), math.ceil(index)
            return ordered[low] if low == high else ordered[low] * (high - index) + ordered[high] * (index - low)

        return {
            "count": len(values), "mean": statistics.fmean(values),
            "median": statistics.median(values),
            "rmse": math.sqrt(statistics.fmean(value * value for value in values)),
            "stddev": statistics.pstdev(values), "min": ordered[0], "max": ordered[-1],
            "p90": percentile(90), "p95": percentile(95), "p99": percentile(99),
        }

    columns = ("error_x_m", "error_y_m", "error_z_m", "error_xy_m", "error_3d_m",
               "error_roll_deg", "error_pitch_deg", "error_yaw_deg", "error_rotation_deg",
               "nearest_gt_stamp_dt_s")
    summary = {column: stats(column) for column in columns}
    elapsed = [row["elapsed_s"] for row in rows if "elapsed_s" in row]
    sync_ms = [value * 1000.0 for value in
               (row["nearest_gt_stamp_dt_s"] for row in rows if "nearest_gt_stamp_dt_s" in row)]
    trajectory_lengths = {}
    coordinate_sets = {
        "ground_truth": ("gt_imu_x_m", "gt_imu_y_m", "gt_imu_z_m"),
        "fast_lio_aligned": ("lio_aligned_x_m", "lio_aligned_y_m", "lio_aligned_z_m"),
    }
    for label, axes in coordinate_sets.items():
        if all(all(axis in row for axis in axes) for row in rows):
            trajectory_lengths[label] = sum(
                math.sqrt(sum((second[axis] - first[axis]) ** 2 for axis in axes))
                for first, second in zip(rows, rows[1:])
            )
    final_drift = ({key: rows[-1][key] for key in
                    ("error_xy_m", "error_3d_m", "error_rotation_deg")}
                   if all(key in rows[-1] for key in
                          ("error_xy_m", "error_3d_m", "error_rotation_deg")) else None)
    gt_length = trajectory_lengths.get("ground_truth")
    summary.update({
        "csv_path": str(path), "sample_count": len(rows),
        "elapsed_window_s": list(elapsed_window) if elapsed_window else None,
        "elapsed_duration_s": max(elapsed) - min(elapsed) if len(elapsed) > 1 else 0.0,
        "mean_update_frequency_hz": (len(elapsed) - 1) / (max(elapsed) - min(elapsed))
        if len(elapsed) > 1 and max(elapsed) > min(elapsed) else None,
        "trajectory_lengths_m": trajectory_lengths or None,
        "final_drift": final_drift,
        "horizontal_drift_per_ground_truth_meter": (
            rows[-1]["error_xy_m"] / gt_length
            if gt_length and "error_xy_m" in rows[-1] else None
        ),
        "timestamp_offset_ms": {
            "mean": statistics.fmean(sync_ms), "max": max(sync_ms),
            "p95": sorted(sync_ms)[min(len(sync_ms) - 1, math.ceil(.95 * len(sync_ms)) - 1)],
        } if sync_ms else None,
    })
    return summary


def summarize_command_timing(events):
    """Derive command elapsed time from recorded command-status transitions."""
    started = {}
    results = {}
    for event in events:
        if event.get("event") != "command_status_changed" or not event.get("command_id"):
            continue
        command_id = int(event["command_id"])
        status = event.get("command_status")
        stamp = int(event["timestamp_rosbag_ns"])
        if status == "RUNNING":
            started.setdefault(command_id, stamp)
        elif status in ("FINISHED", "FAILED", "ABORTED"):
            begin = started.get(command_id)
            results[command_id] = {
                "command_id": command_id, "status": status,
                "start_rosbag_ns": begin, "end_rosbag_ns": stamp,
                "elapsed_s": (stamp - begin) / 1e9 if begin is not None else None,
                "failure_reason": event.get("command_failure_reason", ""),
            }
    return [results[key] for key in sorted(results)]


def summarize_leg_metrics(command_timing, odometry, cmd_vel, plans, destinations, start_ns):
    """Estimate leg distance and speed from odometry and velocity samples."""
    results = []
    destination_by_id = {item.get("command_id"): item for item in destinations
                         if item.get("command_id") is not None}
    odometry_rows = sorted(odometry, key=lambda item: item[0])
    velocities = sorted(cmd_vel, key=lambda item: item[0])
    plan_stamps = [stamp for stamp, _ in plans]
    for timing in command_timing:
        begin, end = timing.get("start_rosbag_ns"), timing["end_rosbag_ns"]
        if begin is None:
            continue
        pose_rows = [(stamp, msg) for stamp, msg in odometry_rows if begin <= stamp <= end]
        distance = 0.0
        for (_, first), (_, second) in zip(pose_rows, pose_rows[1:]):
            p1, p2 = first.pose.pose.position, second.pose.pose.position
            distance += math.hypot(p2.x - p1.x, p2.y - p1.y)
        duration = max(0.0, (end - begin) / 1e9)
        velocity_rows = [(stamp, msg) for stamp, msg in velocities if begin <= stamp <= end]
        moving_samples = []
        stopped_samples = 0
        moving_duration_s = 0.0
        stopped_duration_s = 0.0
        for _, msg in velocity_rows:
            twist = msg.twist if hasattr(msg, "twist") else msg
            speed = math.hypot(float(twist.linear.x), float(twist.linear.y))
            if speed > 0.02:
                moving_samples.append(speed)
            else:
                stopped_samples += 1
        for (stamp_a, msg_a), (stamp_b, _) in zip(velocity_rows, velocity_rows[1:]):
            dt = max(0.0, min((stamp_b - stamp_a) / 1e9, 0.25))
            twist = msg_a.twist if hasattr(msg_a, "twist") else msg_a
            if (math.hypot(float(twist.linear.x), float(twist.linear.y)) > 0.02
                    or abs(float(twist.angular.z)) > 0.02):
                moving_duration_s += dt
            else:
                stopped_duration_s += dt
        leg_plans = [(stamp, msg) for stamp, msg in plans if begin <= stamp <= end]
        planned_distance = None
        if leg_plans:
            path = leg_plans[-1][1]
            path_poses = path.poses
            if len(path_poses) > 1:
                planned_distance = sum(
                    math.hypot(b.pose.position.x - a.pose.position.x,
                               b.pose.position.y - a.pose.position.y)
                    for a, b in zip(path_poses, path_poses[1:])
                )
        start_pose = pose_rows[0][1].pose.pose.position if pose_rows else None
        end_pose = pose_rows[-1][1].pose.pose.position if pose_rows else None
        results.append({
            **timing,
            "target": destination_by_id.get(timing["command_id"]),
            "actual_distance_m": distance if len(pose_rows) > 1 else None,
            "planned_distance_m": planned_distance,
            "start_position_xy_m": [start_pose.x, start_pose.y] if start_pose else None,
            "end_position_xy_m": [end_pose.x, end_pose.y] if end_pose else None,
            "mean_speed_mps": statistics.fmean(moving_samples) if moving_samples else None,
            "max_speed_mps": max(moving_samples) if moving_samples else None,
            "moving_velocity_samples": len(moving_samples),
            "stopped_velocity_samples": stopped_samples,
            "moving_duration_s_estimate": moving_duration_s,
            "stopped_duration_s_estimate": stopped_duration_s,
            "replan_path_messages": sum(begin <= stamp <= end for stamp in plan_stamps),
        })
    return results


def summarize_velocity_stream(samples, intervals=None):
    """Summarize recorded velocity-message cadence and values in selected windows."""
    selected = sorted(
        ((stamp, msg) for stamp, msg in samples
         if intervals is None or any(begin <= stamp <= end for begin, end in intervals)),
        key=lambda item: item[0],
    )
    stamps = [stamp for stamp, _ in selected]
    gaps_ms = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
    gaps_sorted = sorted(gaps_ms)
    values = []
    zero_count = 0
    for _, msg in selected:
        twist = msg.twist if hasattr(msg, "twist") else msg
        linear = math.hypot(float(twist.linear.x), float(twist.linear.y))
        angular = abs(float(twist.angular.z))
        values.append((linear, angular))
        zero_count += linear <= 0.02 and angular <= 0.02
    return {
        "message_count": len(selected),
        "mean_frequency_hz": ((len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)
                               if len(stamps) > 1 and stamps[-1] > stamps[0] else None),
        "median_interval_ms": statistics.median(gaps_ms) if gaps_ms else None,
        "p95_interval_ms": (gaps_sorted[math.ceil(0.95 * len(gaps_sorted)) - 1]
                             if gaps_sorted else None),
        "max_interval_ms": max(gaps_ms) if gaps_ms else None,
        "gaps_over_100ms": sum(gap > 100.0 for gap in gaps_ms),
        "gaps_over_200ms": sum(gap > 200.0 for gap in gaps_ms),
        "mean_absolute_linear_velocity_mps": statistics.fmean(v[0] for v in values) if values else None,
        "max_absolute_linear_velocity_mps": max((v[0] for v in values), default=None),
        "mean_absolute_angular_velocity_radps": statistics.fmean(v[1] for v in values) if values else None,
        "max_absolute_angular_velocity_radps": max((v[1] for v in values), default=None),
        "zero_velocity_percentage": 100.0 * zero_count / len(values) if values else None,
    }


def summarize_controller_diagnostics(samples, rosout_topic):
    """Count controller-related warning/error/recovery log messages, if recorded."""
    if rosout_topic not in samples:
        return {"frequency_warnings": None, "controller_failures": None,
                "controller_recoveries": None}
    counts = {"frequency_warnings": 0, "controller_failures": 0,
              "controller_recoveries": 0}
    for _, message in samples[rosout_topic]:
        text = f"{getattr(message, 'name', '')} {getattr(message, 'msg', '')}".lower()
        if not any(token in text for token in ("controller", "mppi", "follow_path")):
            continue
        if "frequency" in text and ("warn" in text or getattr(message, "level", 0) >= 30):
            counts["frequency_warnings"] += 1
        if any(token in text for token in ("failed", "failure", "exception")):
            counts["controller_failures"] += 1
        if "recover" in text:
            counts["controller_recoveries"] += 1
    return counts


def render_readable_report(report):
    """Render the analysis, data coverage, and caveats for operators."""
    mis22 = report["mis22"]
    mis21 = report["mis21"]
    metrics = mis22["metrics"]
    validation = mis21.get("timestamp_validation", {})
    mis20 = report.get("mis20", {})
    title = report.get("mission_label") or f"Mission {report['mission_id']}"
    lines = [
        f"# {title}",
        "",
        f"**Outcome:** {report['final_mission_status']}  ",
        f"**Duration:** {report['duration_s']:.1f} s  ",
        f"**Mission run:** {report.get('mission_run_id', 'not recorded')}  ",
        f"**Bag time window:** {report.get('started_at_rosbag_ns', 'unknown')}–{report.get('ended_at_rosbag_ns', 'unknown')} ns  ",
        f"**Time base:** {report.get('time_base', 'not recorded')}  ",
        f"**Recording:** `{report.get('recording', 'not recorded')}`  ",
        f"**Mission-state topic:** `{report.get('state_topic', 'not recorded')}`  ",
        f"**Failure reason:** {report.get('mission_failure_reason') or 'none recorded'}  ",
        "",
        "## Experiment metadata",
        "",
        f"- Mission name: {report.get('bundle_metadata', {}).get('mission_name', 'Not collected')}",
        f"- Preset: {report.get('bundle_metadata', {}).get('mission_preset', 'Not collected')}",
        f"- Namespace: {report.get('bundle_metadata', {}).get('namespace', 'Not collected')}",
        f"- Ordering mode: {report.get('bundle_metadata', {}).get('ordering_mode', 'Not collected')}",
        f"- Final destination option: {report.get('bundle_metadata', {}).get('final_destination_option', 'Not collected')}",
        f"- Bundle created: {report.get('bundle_metadata', {}).get('bundle_created_at', 'Not collected')}",
        f"- Mission client submission status: {report.get('bundle_metadata', {}).get('mission_client_status', 'Not collected')}",
        f"- Final mission server status: {report.get('final_mission_status', 'Not collected')}",
        f"- Git branch / commit at report processing: {report.get('workspace_provenance', {}).get('git_branch') or 'Not collected'} / {report.get('workspace_provenance', {}).get('git_commit') or 'Not collected'}",
        f"- Configuration hashes at processing: {json.dumps(report.get('workspace_provenance', {}).get('configuration_sha256_at_processing_time', {}), sort_keys=True) or 'Not collected'}",
        "- Wall-clock mission duration: Not collected (bundle creation time is not mission end time).",
        "- Robot/sensor model, world, software versions, and random seed: Not collected unless supplied in bundle metadata.",
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
    commands = report.get("final_commands", mis22.get("command_outcomes", []))
    status_counts = {status: sum(command.get("status") == status for command in commands)
                     for status in ("FINISHED", "FAILED", "ABORTED", "WAITING", "RUNNING", "PAUSED")}
    lines.extend(["", "### Command summary", "",
                  f"- Total commands: {len(commands)}; completed: {status_counts['FINISHED']}; "
                  f"failed: {status_counts['FAILED']}; aborted: {status_counts['ABORTED']}."])
    lines.extend(["", "## Command outcomes", "", "| Command ID | Status | Failure reason |", "|---:|---|---|"])
    for command in commands:
        reason = str(command.get("failure_reason") or "—").replace("|", "\\|").replace("\n", " ")
        lines.append(f"| {command.get('command_id', 'unknown')} | {command.get('status', 'unknown')} | {reason} |")
    if not commands:
        lines.append("| — | No command outcomes recorded | — |")
    command_timing = report.get("command_timing", [])
    if command_timing:
        lines.extend(["", "### Command elapsed times", "", "| Command ID | Status | Start | End | Elapsed |", "|---:|---|---:|---:|---:|"])
        for item in command_timing:
            start_s = (item["start_rosbag_ns"] - report["started_at_rosbag_ns"]) / 1e9 if item.get("start_rosbag_ns") is not None else None
            end_s = (item["end_rosbag_ns"] - report["started_at_rosbag_ns"]) / 1e9
            elapsed = f"{item['elapsed_s']:.3f} s" if item.get("elapsed_s") is not None else "not available"
            lines.append(f"| {item['command_id']} | {item['status']} | {f'{start_s:.3f} s' if start_s is not None else 'unknown'} | {end_s:.3f} s | {elapsed} |")
    leg_metrics = report.get("per_command_leg_metrics", [])
    if leg_metrics:
        lines.extend(["", "### Per-waypoint leg metrics", "",
                      "Distance is integrated from odometry. Moving/stopped durations are estimates from cmd_vel samples (gaps capped at 0.25 s). Plan-message count is a replan proxy, not a confirmed planner-replan event.",
                      "| Command / target | Duration | Planned distance | Actual distance | Moving / stopped | Mean / max speed | Plan messages |",
                      "|---|---:|---:|---:|---:|---:|---:|"])
        for leg in leg_metrics:
            target = leg.get("target") or {}
            coords = (f" ({target['latitude']:.6f}, {target['longitude']:.6f})"
                      if target.get("latitude") is not None and target.get("longitude") is not None else "")
            duration = f"{leg['elapsed_s']:.2f} s" if leg.get("elapsed_s") is not None else "—"
            distance = f"{leg['actual_distance_m']:.2f} m" if leg.get("actual_distance_m") is not None else "—"
            planned = f"{leg['planned_distance_m']:.2f} m" if leg.get("planned_distance_m") is not None else "—"
            mean_speed = f"{leg['mean_speed_mps']:.2f}" if leg.get("mean_speed_mps") is not None else "—"
            max_speed = f"{leg['max_speed_mps']:.2f}" if leg.get("max_speed_mps") is not None else "—"
            label = f"Command {leg['command_id']}{coords}"
            moving = f"{leg['moving_duration_s_estimate']:.1f} / {leg['stopped_duration_s_estimate']:.1f} s"
            lines.append(f"| {label} | {duration} | {planned} | {distance} | {moving} | {mean_speed} / {max_speed} m/s | {leg['replan_path_messages']} |")
    else:
        lines.extend(["", "### Per-waypoint leg metrics", "", "Not available: required command timing or odometry samples were not recorded."])
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
    lines.append(f"- Data-capture validation: **{validation.get('status', 'not evaluated')}**.")
    lines.append(f"- MIS-21 summary: {mis21.get('status', 'not evaluated')}.")
    lines.append(f"- MIS-22 summary: {mis22.get('status', 'not evaluated')}.")
    expected_streams = validation.get("expected_no_overlap_groups", [])
    if expected_streams:
        lines.append("- No messages were expected during this run for: " + ", ".join(expected_streams) + ".")
    lines.append(f"- Fault records: {len(mis22.get('fault_events', []))}.")
    for fault in mis22.get("fault_events", []):
        message = str(fault.get("message", "Fault details unavailable.")).replace("|", "\\|").replace("\n", " ")
        lines.append(f"  - {message}")
    keepout_events = report.get("keepout_events", [])
    lines.append(f"- Keepout-zone lifecycle events found in rosout: {len(keepout_events)}.")
    for event in keepout_events:
        relative_s = (event["timestamp_rosbag_ns"] - report["started_at_rosbag_ns"]) / 1e9
        lines.append(f"  - {relative_s:.3f} s [{event.get('component', 'unknown')}, severity {event.get('severity', 'unknown')}]: {event.get('message', '')}")
    if not keepout_events:
        lines.append("  - No matching keepout lifecycle logs found (not proof that no zone changed).")
    lines.extend(["", "### Recorded topic message counts", ""])
    topic_counts = validation.get("topic_message_counts", {})
    if topic_counts:
        lines.extend(["| Topic | Messages |", "|---|---:|"])
        lines.extend(f"| `{topic}` | {count} |" for topic, count in sorted(topic_counts.items()))
    else:
        lines.append("No topic message counts were available.")
    missing = mis21.get("missing_required_topics", [])
    lines.extend(["", "### Missing required data", ""])
    if missing:
        lines.extend(f"- {item}" for item in missing)
    else:
        lines.append("- None reported.")
    lines.extend(["", "## Interventions and scoring", ""])
    interventions = mis22.get("interventions", {})
    lines.extend(f"- {label.replace('_', ' ').capitalize()}: {count if count is not None else 'unavailable'}"
                 for label, count in interventions.items())
    lines.append(f"- MIS-20: {mis20.get('status', 'not evaluated')}" +
                 (f" — {mis20.get('reason')}" if mis20.get("reason") else ""))
    if mis20.get("destination_scores"):
        lines.extend(["", "| Scored destination | Minimum edge distance | Within 1 m |", "|---|---:|---|"])
        for score in mis20["destination_scores"]:
            lines.append(f"| {score.get('name', 'Destination')} | {score.get('minimum_vehicle_edge_distance_m', 'unavailable')} m | {score.get('within_1m', 'unavailable')} |")
    if mis22.get("not_recorded_or_not_computed"):
        lines.extend(["", "### Not recorded or not computed", ""])
        lines.extend(f"- {item}" for item in mis22["not_recorded_or_not_computed"])
    localization = report.get("fast_lio_localization")
    lines.extend(["", "## FAST-LIO localization statistics", ""])
    if localization:
        frequency = localization.get("mean_update_frequency_hz")
        summary = (f"; mean update frequency: {frequency:.2f} Hz" if frequency else "")
        lines.append(f"- Mission-window error CSV samples: {localization['sample_count']}; "
                     f"duration: {localization['elapsed_duration_s']:.2f} s{summary}.")
        lines.extend(["", "| Error metric | Mean | Median | RMSE | Std dev | Min | Max | P90 | P95 | P99 |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"])
        for key, label, unit in (("error_x_m", "X position", "m"), ("error_y_m", "Y position", "m"),
                                 ("error_z_m", "Z position", "m"), ("error_xy_m", "Horizontal position", "m"),
                                 ("error_3d_m", "3D position", "m"), ("error_roll_deg", "Roll", "deg"),
                                 ("error_pitch_deg", "Pitch", "deg"), ("error_yaw_deg", "Yaw", "deg"),
                                 ("error_rotation_deg", "Rotation angle", "deg")):
            values = localization.get(key)
            if values:
                cells = [f"{values[name]:.4f} {unit}" for name in
                         ("mean", "median", "rmse", "stddev", "min", "max", "p90", "p95", "p99")]
                lines.append(f"| {label} | " + " | ".join(cells) + " |")
        sync = localization.get("timestamp_offset_ms")
        if sync:
            lines.append(f"- FAST-LIO/ground-truth timestamp offset: {sync['mean']:.2f} ms mean, {sync['p95']:.2f} ms P95, {sync['max']:.2f} ms max.")
        else:
            lines.append("- FAST-LIO/ground-truth timestamp offset: Not collected.")
        lengths = localization.get("trajectory_lengths_m") or {}
        if lengths:
            lines.append("- Trajectory lengths: " + ", ".join(
                f"{name.replace('_', ' ')} {length:.2f} m" for name, length in lengths.items()
            ) + ".")
        drift = localization.get("final_drift")
        if drift:
            lines.append(f"- Final aligned drift: horizontal {drift['error_xy_m']:.4f} m, "
                         f"3D {drift['error_3d_m']:.4f} m, rotation {drift['error_rotation_deg']:.4f} deg.")
        drift_per_m = localization.get("horizontal_drift_per_ground_truth_meter")
        lines.append(f"- Horizontal drift per ground-truth meter: {drift_per_m:.6f} m/m." if drift_per_m is not None
                     else "- Horizontal drift per ground-truth meter: Not available.")
    else:
        lines.append("Mission-window FAST-LIO error statistics unavailable (CSV missing or no aligned samples).")
    full_localization = report.get("fast_lio_full_recording")
    if full_localization:
        lines.append(f"- Full-recording comparison: {full_localization['sample_count']} samples over "
                     f"{full_localization['elapsed_duration_s']:.2f} s; CSV `{Path(full_localization['csv_path']).name}`.")
    lines.extend(["", "## Navigation, obstacle, and system performance data", "",
                  "- Navigation planning/controller timings, replan/recovery counts, velocity distributions, and collision-monitor interventions: Not collected as structured metrics in this report.",
                  "- Keepout update history, cloud age/latency, and costmap update delays: Not collected as structured metrics in this report.",
                  "- Per-process CPU/memory, host utilization, swap, real-time factor, and process restarts: Not collected in the rosbag; these require concurrent system sampling during the run."])
    lines.extend(["", "## Mission timeline", ""])
    timeline = report.get("mission_timeline", [])
    if timeline:
        lines.extend(["| Bag time | Event | Command | Status | Details |", "|---:|---|---:|---|---|"])
        for event in timeline:
            time_s = (event["timestamp_rosbag_ns"] - report["started_at_rosbag_ns"]) / 1e9
            details = str(event.get("detail", "—")).replace("|", "\\|").replace("\n", " ")
            lines.append(f"| {time_s:.3f} s | {event.get('event', 'event')} | {event.get('command_id') or '—'} | {event.get('status') or '—'} | {details} |")
    else:
        lines.append("No mission event rows were available.")
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
    parser.add_argument("--mppi-output-topic", help="Recorded raw MPPI output topic; omit if unknown or not recorded")
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
    requested_topics = [args.odom_topic, args.cmd_vel_topic, args.teleop_topic, args.safety_topic,
                   args.state_topic, args.autonomy_lock_topic, args.estop_topic,
                   args.estop_lock_topic, args.rosout_topic,
                   args.ground_truth_topic, args.datum_topic, args.waypoints_topic,
                   "/terrascout1/keepout_obstacle_points", "/terrascout1/keepout_zone_marker",
                   "/terrascout1/plan", "/terrascout1/trajectories",
                   "/terrascout1/fixposition/odometry_enu",
                   "/terrascout1/fixposition/odometry_llh", "/terrascout1/fast_lio/odometry",
                   "/terrascout1/obstacle_points", "/terrascout1/ground_points"]
    if args.mppi_output_topic:
        requested_topics.append(args.mppi_output_topic)
    metrics, topic_types, topic_counts = load_metric_samples(
        args.bag, requested_topics
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
    keepout_events = summarize_keepout_events(mission_metrics, args.rosout_topic)
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
    error_csv_candidates = sorted(args.out_dir.glob("*_fast_lio_error.csv"), key=lambda path: path.stat().st_mtime)
    error_csv = error_csv_candidates[-1] if error_csv_candidates else None
    lio_samples = metrics.get("/terrascout1/fast_lio/odometry", [])
    first_lio_stamp_ns = time_ns(lio_samples[0][1].header.stamp) if lio_samples else None
    mission_elapsed_window = (
        (max(0.0, (first_stamp - first_lio_stamp_ns) / 1e9),
         max(0.0, (last_stamp - first_lio_stamp_ns) / 1e9))
        if first_lio_stamp_ns is not None else None
    )
    fast_lio_localization = summarize_error_csv(error_csv, mission_elapsed_window)
    fast_lio_full_recording = summarize_error_csv(error_csv)
    mission_timeline = []
    for event in events:
        detail = event.get("command_failure_reason") or event.get("failure_reason") or event.get("order_change_reason") or event.get("not_ready_reason") or ""
        mission_timeline.append({
            "timestamp_rosbag_ns": int(event["timestamp_rosbag_ns"]),
            "event": event.get("event", ""),
            "command_id": event.get("command_id", ""),
            "status": event.get("command_status") or event.get("mission_status", ""),
            "detail": detail,
        })
    command_timing = summarize_command_timing(events)
    per_command_leg_metrics = summarize_leg_metrics(
        command_timing,
        mission_metrics.get(args.odom_topic, []),
        mission_metrics.get(args.cmd_vel_topic, []),
        mission_metrics.get("/terrascout1/plan", []),
        destinations,
        first_stamp,
    )
    destination_command_ids = {item.get("command_id") for item in destinations
                               if item.get("command_id") is not None}
    navigation_intervals = [
        (item["start_rosbag_ns"], item["end_rosbag_ns"])
        for item in command_timing
        if item.get("command_id") in destination_command_ids
        and item.get("start_rosbag_ns") is not None
    ]
    final_velocity = summarize_velocity_stream(
        mission_metrics.get(args.cmd_vel_topic, []), navigation_intervals
    )
    mppi_velocity = (summarize_velocity_stream(
        mission_metrics.get(args.mppi_output_topic, []), navigation_intervals
    ) if args.mppi_output_topic and args.mppi_output_topic in mission_metrics else None)
    per_command_by_id = {item["command_id"]: item for item in per_command_leg_metrics}
    per_command_velocity = {}
    for command_id in destination_command_ids:
        timing = per_command_by_id.get(command_id)
        if timing and timing.get("start_rosbag_ns") is not None:
            per_command_velocity[command_id] = summarize_velocity_stream(
                mission_metrics.get(args.cmd_vel_topic, []),
                [(timing["start_rosbag_ns"], timing["end_rosbag_ns"])],
            )
    for item in per_command_leg_metrics:
        velocity = per_command_velocity.get(item["command_id"])
        item["controller_metrics"] = ({
            "cmd_vel_count": velocity["message_count"],
            "cmd_vel_mean_frequency_hz": velocity["mean_frequency_hz"],
            "cmd_vel_p95_interval_ms": velocity["p95_interval_ms"],
            "cmd_vel_max_interval_ms": velocity["max_interval_ms"],
            "cmd_vel_gaps_over_100ms": velocity["gaps_over_100ms"],
            "cmd_vel_zero_velocity_percentage": velocity["zero_velocity_percentage"],
        } if velocity else None)
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
        "mission_timeline": mission_timeline,
        "command_timing": command_timing,
        "per_command_leg_metrics": per_command_leg_metrics,
        "controller_performance": {
            "controller_name": "nav2_mppi_controller::MPPIController",
            "configured_frequency_hz": 20.0,
            "expected_period_ms": 50.0,
            "mppi_output": {"topic": args.mppi_output_topic, **mppi_velocity}
                if mppi_velocity is not None else {"topic": args.mppi_output_topic,
                    "message_count": None, "mean_frequency_hz": None,
                    "median_interval_ms": None, "p95_interval_ms": None,
                    "max_interval_ms": None, "gaps_over_100ms": None,
                    "gaps_over_200ms": None},
            "final_cmd_vel": {"topic": args.cmd_vel_topic, **final_velocity},
            "controller_diagnostics": summarize_controller_diagnostics(
                mission_metrics, args.rosout_topic
            ),
        },
        "bundle_metadata": bundle_metadata,
        "workspace_provenance": workspace_provenance(Path(__file__).resolve().parents[4]),
        "fast_lio_localization": fast_lio_localization,
        "fast_lio_full_recording": fast_lio_full_recording,
        "keepout_events": keepout_events,
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
