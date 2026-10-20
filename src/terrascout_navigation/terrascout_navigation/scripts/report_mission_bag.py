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
        stamp = time_ns(msg.stamp) or received_ns
        states.append((stamp, msg))
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
    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic not in samples:
            continue
        message = deserialize_message(serialized, message_types[topic])
        stamp = bag_time_ns
        if hasattr(message, "header") and time_ns(message.header.stamp):
            stamp = time_ns(message.header.stamp)
        samples[topic].append((stamp, message))
    return samples, {topic: type_map.get(topic) for topic in topics}


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


def extract_geojson_destinations(messages):
    """Return the most recent mission waypoint points stored by the mission server."""
    if not messages:
        return []
    _, msg = max(messages, key=lambda item: item[0])
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
        })
    return destinations


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


def write_events(states, path: Path, mission_id: int | None):
    fields = ["timestamp_unix_ns", "mission_id", "mission_status", "current_command_id", "command_id", "command_status", "ready", "not_ready_reason", "event"]
    events = []
    previous = None
    for stamp, msg in states:
        if mission_id is not None and msg.mission_id != mission_id:
            continue
        current_status = {int(cid): int(status) for cid, status in zip(msg.command_ids, msg.command_status)}
        if previous is None or msg.mission_id != previous[0]:
            events.append({"timestamp_unix_ns": stamp, "mission_id": msg.mission_id,
                           "mission_status": MISSION_STATUS.get(msg.mission_status, str(msg.mission_status)),
                           "current_command_id": msg.current_command_id, "command_id": "", "command_status": "",
                           "ready": msg.ready, "not_ready_reason": msg.not_ready_reason, "event": "mission_snapshot"})
        elif msg.mission_status != previous[1]:
            events.append({"timestamp_unix_ns": stamp, "mission_id": msg.mission_id,
                           "mission_status": MISSION_STATUS.get(msg.mission_status, str(msg.mission_status)),
                           "current_command_id": msg.current_command_id, "command_id": "", "command_status": "",
                           "ready": msg.ready, "not_ready_reason": msg.not_ready_reason, "event": "mission_status_changed"})
        for cid, status in current_status.items():
            if previous is None or msg.mission_id != previous[0] or previous[2].get(cid) != status:
                events.append({"timestamp_unix_ns": stamp, "mission_id": msg.mission_id,
                               "mission_status": MISSION_STATUS.get(msg.mission_status, str(msg.mission_status)),
                               "current_command_id": msg.current_command_id, "command_id": cid,
                               "command_status": COMMAND_STATUS.get(status, str(status)), "ready": msg.ready,
                               "not_ready_reason": msg.not_ready_reason, "event": "command_status_changed"})
        previous = (msg.mission_id, msg.mission_status, current_status)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(events)
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 MCAP bag directory")
    parser.add_argument("--out-dir", help="Existing mission result bundle directory; defaults to the newest tester-created mission folder")
    parser.add_argument("--state-topic", default="/terrascout1/mission_state")
    parser.add_argument("--mission-id", type=int, help="Select a mission ID; defaults to the latest mission in the bag")
    parser.add_argument("--route", type=Path, help="Copy the route/waypoint JSON into the bundle")
    parser.add_argument("--odom-topic", default="/terrascout1/odom")
    parser.add_argument("--cmd-vel-topic", default="/terrascout1/cmd_vel")
    parser.add_argument("--teleop-topic", default="/terrascout1/teleop_takeover")
    parser.add_argument("--safety-topic", default="/terrascout1/platform/safety_stop")
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
    metrics, _topic_types = load_metric_samples(
        args.bag, [args.odom_topic, args.cmd_vel_topic, args.teleop_topic, args.safety_topic,
                   args.ground_truth_topic, args.datum_topic, args.waypoints_topic]
    )
    motion_summary, available_metrics, missing_metrics = summarize_motion(
        metrics, args.odom_topic, args.cmd_vel_topic, args.teleop_topic, args.safety_topic
    )
    available_ids = sorted({msg.mission_id for _, msg in states if msg.mission_id > 0})
    if not available_ids:
        raise RuntimeError("No positive mission IDs found in recorded state")
    mission_id = args.mission_id if args.mission_id is not None else available_ids[-1]
    if mission_id not in available_ids:
        raise RuntimeError(f"Mission ID {mission_id} not found; recorded IDs: {available_ids}")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if args.route and not (args.out_dir / "waypoints.json").exists():
        (args.out_dir / "waypoints.json").write_bytes(args.route.read_bytes())
    events = write_events(states, args.out_dir / "mission_events.csv", mission_id)
    selected = [(stamp, msg) for stamp, msg in states if msg.mission_id == mission_id]
    first_stamp, last_stamp = selected[0][0], selected[-1][0]
    final = selected[-1][1]
    completed = [int(cid) for cid, status in zip(final.command_ids, final.command_status) if status == 3]
    destinations = extract_geojson_destinations(metrics.get(args.waypoints_topic, []))
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
    ground_truth = metrics.get(args.ground_truth_topic, [])
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
    report = {
        "schema_version": 1,
        "mission_id": mission_id,
        "recording": str(args.bag.resolve()),
        "state_topic": args.state_topic,
        "recorded_mission_ids": available_ids,
        "started_at_unix_ns": first_stamp,
        "ended_at_unix_ns": last_stamp,
        "duration_s": (last_stamp - first_stamp) / 1e9,
        "final_mission_status": MISSION_STATUS.get(final.mission_status, str(final.mission_status)),
        "final_commands": [{"command_id": int(cid), "status": COMMAND_STATUS.get(status, str(status))}
                           for cid, status in zip(final.command_ids, final.command_status)],
        "completed_command_ids": completed,
        "mission_event_rows": len(events),
        "mis20": mis20,
        "mis22": {"status": "PARTIAL", "available": ["mission state transitions", "command outcomes", "mission duration", *available_metrics],
              "metrics": motion_summary,
              "order_changes": "Not reported by the recorded MissionState schema.",
              "not_recorded_or_not_computed": ["order-change reasons", "minimum destination edge distances", "faults", *missing_metrics]},
        "caveats": ["Mission state topic must be included in the bag.", "State snapshots do not provide rejection/failure reason text for each command.", "MIS-20 assumes /odom_gt odom axes align with map axes and uses the configured Nav2 footprint polygon.", "A simulation scoring pass is not an assessed-trial or physical-robot pass."],
    }
    (args.out_dir / "mission_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {args.out_dir / 'mission_events.csv'} and {args.out_dir / 'mission_report.json'}")


if __name__ == "__main__":
    main()
