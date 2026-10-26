#!/usr/bin/env python3
"""Compare FAST-LIO2 odometry with Gazebo ground truth from an MCAP ROS 2 bag.

FAST-LIO2 is aligned to the ground-truth IMU pose at the first matched sample.
The reported errors measure relative trajectory drift, not absolute localization.
"""

import argparse
import csv
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
import re
from zoneinfo import ZoneInfo

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp
from std_msgs.msg import String

SWEDISH_TIMEZONE = ZoneInfo("Europe/Stockholm")


def read_odometry_bag(bag_path: Path, topics: list[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics if topic not in type_map]
    if missing:
        raise RuntimeError(f"Missing topics {missing}; available: {sorted(type_map)}")
    expected_type = "nav_msgs/msg/Odometry"
    for topic in topics:
        if type_map[topic] != expected_type:
            raise RuntimeError(f"{topic}: expected {expected_type}, got {type_map[topic]}")

    message_types = {topic: get_message(type_map[topic]) for topic in topics}
    rows = {topic: [] for topic in topics}
    frames = {topic: set() for topic in topics}
    fallback_counts = {topic: 0 for topic in topics}
    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic not in rows:
            continue
        message = deserialize_message(serialized, message_types[topic])
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns == 0:
            stamp_ns = received_ns
            fallback_counts[topic] += 1
        pose = message.pose.pose
        position = pose.position
        orientation = pose.orientation
        xyz = np.array([position.x, position.y, position.z], dtype=float)
        quat = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w], dtype=float
        )
        norm = np.linalg.norm(quat)
        if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quat)) or norm < 1e-8:
            continue
        rows[topic].append((stamp_ns * 1e-9, xyz, quat / norm))
        frames[topic].add((message.header.frame_id, message.child_frame_id))

    for topic in topics:
        rows[topic].sort(key=lambda row: row[0])
        # Keep the last message for duplicate timestamps.
        unique = {row[0]: row for row in rows[topic]}
        rows[topic] = list(unique.values())
        print(f"{topic}: {len(rows[topic])} samples, frames={frames[topic]}")
        if fallback_counts[topic]:
            print(f"WARNING: {fallback_counts[topic]} zero-stamped messages used bag receipt time")
        if not rows[topic]:
            raise RuntimeError(f"No usable odometry samples for {topic}")
    return rows, frames


def read_plan_bag(bag_path: Path, topic: str, start_time: float, end_time: float):
    """Read paths and their frame IDs in the selected odometry time interval."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in type_map:
        print(f"WARNING: planned path topic {topic} is not in the bag")
        return []
    if type_map[topic] != "nav_msgs/msg/Path":
        print(f"WARNING: {topic} has type {type_map[topic]}, expected nav_msgs/msg/Path")
        return []

    try:
        message_type = get_message(type_map[topic])
    except (ImportError, AttributeError, ModuleNotFoundError):
        # foxglove_msgs/GeoJSON has the same single-string CDR layout as String.
        if type_map[topic] != "foxglove_msgs/msg/GeoJSON":
            raise
        message_type = String
    paths = []
    while reader.has_next():
        current_topic, serialized, received_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_type)
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        path_time = stamp_ns * 1e-9 if stamp_ns else received_ns * 1e-9
        if not start_time <= path_time <= end_time:
            continue
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
            if np.isfinite(pose.pose.position.x) and np.isfinite(pose.pose.position.y)
        ]
        if points:
            paths.append((path_time, message.header.frame_id, np.asarray(points, dtype=float)))
    paths.sort(key=lambda item: item[0])
    # Keep one representative/latest plan per destination leg, avoiding a stack
    # of near-identical replans for that waypoint.
    latest_by_leg = {}
    for item in paths:
        endpoint = item[2][-1]
        leg_key = tuple(np.round(endpoint, decimals=1))
        latest_by_leg[leg_key] = item
    selected = sorted(latest_by_leg.values(), key=lambda item: item[0])
    print(
        f"{topic}: {len(paths)} path messages in the plotted interval; "
        f"showing {len(selected)} latest plan(s), one per destination endpoint"
    )
    return selected


def read_waypoint_snapshot(bag_path: Path, topic: str, end_time: float):
    """Read the last GeoJSON waypoint snapshot up to the plotted mission end."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in type_map:
        print(f"WARNING: waypoint topic {topic} is not in the bag")
        return [], None
    try:
        message_type = get_message(type_map[topic])
    except (ImportError, AttributeError, ModuleNotFoundError):
        # foxglove_msgs/GeoJSON has the same single-string CDR layout as String.
        if type_map[topic] != "foxglove_msgs/msg/GeoJSON":
            raise
        message_type = String
    snapshots = []
    while reader.has_next():
        current_topic, serialized, received_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_type)
        # GeoJSON is a single-string message without a header; consistently use
        # rosbag receive time for selecting its snapshot against bag timestamps.
        message_time = received_ns * 1e-9
        if message_time > end_time:
            continue
        raw = message.data if hasattr(message, "data") else message.geojson
        try:
            collection = json.loads(raw)
        except (TypeError, ValueError, json.JSONDecodeError):
            continue
        points = []
        for feature in collection.get("features", []):
            geometry = feature.get("geometry", {})
            coords = geometry.get("coordinates", [])
            if geometry.get("type") != "Point" or len(coords) < 2:
                continue
            properties = feature.get("properties", {})
            metadata = properties.get("metadata", {})
            # This topic's GeoJSON coordinates are longitude/latitude, converted
            # to the same local ENU basis used by the ground-truth odometry.
            status = str(metadata.get("status", "pending")).lower()
            style = properties.get("style", {})
            color = str(style.get("color", ""))
            points.append((
                float(metadata.get("lon", coords[0])),
                float(metadata.get("lat", coords[1])),
                str(properties.get("name", f"WP{len(points) + 1}")),
                status,
                color,
            ))
        if points:
            snapshots.append((message_time, points))
    if not snapshots:
        print(f"WARNING: no waypoint snapshot found on {topic}")
        return [], None
    print(f"{topic}: using waypoint snapshot at bag time {max(snapshots, key=lambda x: x[0])[0]:.3f}s")
    selected = max(snapshots, key=lambda item: item[0])
    return selected[1], selected[0]


def local_enu_xy(longitude, latitude, origin_longitude, origin_latitude):
    """Convert nearby WGS84 coordinates to local east/north metres."""
    earth_radius_m = 6_378_137.0
    lat0 = np.deg2rad(origin_latitude)
    east = earth_radius_m * np.deg2rad(longitude - origin_longitude) * np.cos(lat0)
    north = earth_radius_m * np.deg2rad(latitude - origin_latitude)
    return np.array([east, north], dtype=float)


def read_datum(bag_path: Path, topic: str):
    """Return the latest recorded NavSatFix datum (latitude, longitude)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in type_map:
        print(f"WARNING: datum topic {topic} is not in the bag; waypoint markers omitted")
        return None
    message_type = get_message(type_map[topic])
    datum = None
    while reader.has_next():
        current_topic, serialized, _ = reader.read_next()
        if current_topic == topic:
            message = deserialize_message(serialized, message_type)
            datum = (float(message.latitude), float(message.longitude))
    return datum


def read_map_odom_transform(bag_path: Path, namespace: str):
    """Read recorded map/odom TF and return a transform function map XY -> odom XY."""
    tf_topics = (f"/{namespace}/tf", f"/{namespace}/tf_static", "/tf", "/tf_static")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    available = [topic for topic in tf_topics if topic in type_map]
    if not available:
        print("WARNING: no TF topics recorded; cannot transform map-frame plan")
        return None
    tf_type = get_message("tf2_msgs/msg/TFMessage")
    candidates = []
    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic not in available:
            continue
        is_static = topic.endswith("tf_static")
        message = deserialize_message(serialized, tf_type)
        for stamped in message.transforms:
            parent = stamped.header.frame_id.lstrip("/")
            child = stamped.child_frame_id.lstrip("/")
            if (parent, child) not in (("map", "odom"), (f"{namespace}/map", f"{namespace}/odom")):
                continue
            stamp = stamped.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            candidates.append((is_static, stamp_ns or received_ns, parent, child, stamped.transform))
    if not candidates:
        print("WARNING: recorded TF contains no direct map/odom transform")
        return None
    # Static TF is invariant; otherwise use the latest recorded transform.
    static_candidates = [candidate for candidate in candidates if candidate[0]]
    chosen = max(static_candidates or candidates, key=lambda candidate: candidate[1])
    _, _, parent, child, transform = chosen
    q = transform.rotation
    yaw = 2.0 * np.arctan2(q.z, q.w)
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    translation = np.array([transform.translation.x, transform.translation.y])
    print(
        f"Using recorded {'static' if chosen[0] else 'dynamic'} TF "
        f"{parent} -> {child}: x={translation[0]:.3f}, y={translation[1]:.3f}, yaw={np.rad2deg(yaw):.3f} deg"
    )
    if child.endswith("odom"):
        # parent=map, child=odom: invert map<-odom to obtain odom<-map.
        return lambda points: (points - translation) @ rotation
    # parent=odom, child=map already maps map coordinates into odom.
    return lambda points: points @ rotation.T + translation


def read_keepout_zones(bag_path: Path, topic: str, end_time: float, map_to_odom=None):
    """Read the last active cube markers and convert their footprints to odom XY."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in type_map:
        print(f"WARNING: keepout marker topic {topic} is not in the bag")
        return []
    if type_map[topic] != "visualization_msgs/msg/Marker":
        print(f"WARNING: {topic} has type {type_map[topic]}, expected visualization_msgs/msg/Marker")
        return []
    marker_type = get_message(type_map[topic])
    latest = {}
    while reader.has_next():
        current_topic, serialized, received_ns = reader.read_next()
        if current_topic != topic or received_ns * 1e-9 > end_time:
            continue
        marker = deserialize_message(serialized, marker_type)
        if marker.action == marker.DELETEALL:
            latest.clear()
        elif marker.action == marker.DELETE:
            latest.pop((marker.ns, marker.id), None)
        elif marker.type == marker.CUBE:
            latest[(marker.ns, marker.id)] = marker

    zones = []
    for marker in latest.values():
        frame = marker.header.frame_id.lstrip("/")
        center = np.array([marker.pose.position.x, marker.pose.position.y])
        yaw = 2.0 * np.arctan2(marker.pose.orientation.z, marker.pose.orientation.w)
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        local = np.array([
            [-marker.scale.x / 2, -marker.scale.y / 2],
            [ marker.scale.x / 2, -marker.scale.y / 2],
            [ marker.scale.x / 2,  marker.scale.y / 2],
            [-marker.scale.x / 2,  marker.scale.y / 2],
        ])
        polygon = local @ rotation.T + center
        if frame == "map" or frame.endswith("/map"):
            if map_to_odom is None:
                print("WARNING: keepout markers are in map frame but no map-to-odom transform was recorded; overlay omitted")
                continue
            polygon = map_to_odom(polygon)
        elif frame not in ("odom", "") and not frame.endswith("/odom"):
            print(f"WARNING: keepout marker frame {frame!r} is unsupported; overlay omitted")
            continue
        zones.append({"label": marker.text or f"Keepout {marker.id}", "points": polygon})
    print(f"{topic}: plotting {len(zones)} recorded keepout zone(s)")
    return zones


def arrays(rows):
    return (
        np.array([row[0] for row in rows], dtype=float),
        np.stack([row[1] for row in rows]),
        np.stack([row[2] for row in rows]),
    )


def match_and_interpolate(lio_rows, gt_rows, max_dt: float):
    lio_t, lio_p, lio_q = arrays(lio_rows)
    gt_t, gt_p, gt_q = arrays(gt_rows)
    if len(gt_t) < 2:
        raise RuntimeError("At least two ground-truth samples are required")
    lio_indices = np.flatnonzero((lio_t >= gt_t[0]) & (lio_t <= gt_t[-1]))
    matched_t = lio_t[lio_indices]
    upper = np.clip(np.searchsorted(gt_t, matched_t), 1, len(gt_t) - 1)
    lower = upper - 1
    nearest_dt = np.minimum(abs(matched_t - gt_t[lower]), abs(gt_t[upper] - matched_t))
    valid = nearest_dt <= max_dt
    lio_indices, matched_t, nearest_dt = lio_indices[valid], matched_t[valid], nearest_dt[valid]
    if len(lio_indices) < 2:
        raise RuntimeError(
            f"Only {len(lio_indices)} timestamps matched within {max_dt:.3f}s; "
            "check simulation time and bag overlap"
        )
    gt_position = np.column_stack([np.interp(matched_t, gt_t, gt_p[:, axis]) for axis in range(3)])
    gt_rotation = Slerp(gt_t, Rotation.from_quat(gt_q))(matched_t)
    print(
        f"Matched {len(lio_indices)}/{len(lio_t)} FAST-LIO2 samples; "
        f"nearest GT timestamp mean={np.mean(nearest_dt) * 1000:.1f} ms"
    )
    return (
        matched_t,
        lio_p[lio_indices],
        Rotation.from_quat(lio_q[lio_indices]),
        gt_position,
        gt_rotation,
        nearest_dt,
    )


def print_stats(name: str, values, unit: str):
    values = np.asarray(values)
    print(
        f"{name:14s} mean={np.mean(values): .5f} {unit} "
        f"MAE={np.mean(np.abs(values)):.5f} {unit} "
        f"RMSE={np.sqrt(np.mean(values**2)):.5f} {unit} "
        f"max={np.max(np.abs(values)):.5f} {unit}"
    )


def save_plot(path: Path, seconds, series, ylabel: str, title: str):
    if plt is None:
        return
    plt.figure(figsize=(10, 5))
    for name, values in series.items():
        plt.plot(seconds, values, label=name)
    plt.xlabel("Time since first matched sample [s]")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_trajectory_plot(
    path: Path, gt_position, lio_position, planned_paths=(), waypoints=(),
    keepout_zones=(), show=False
):
    """Save an XY overlay and optionally display it interactively."""
    if plt is None:
        return
    plt.figure(figsize=(8, 8))
    plt.plot(gt_position[:, 0], gt_position[:, 1], label="Ground truth", linewidth=2)
    plt.plot(
        lio_position[:, 0], lio_position[:, 1],
        label="FAST-LIO2", linewidth=1.5,
    )
    for zone in keepout_zones:
        polygon = np.vstack((zone["points"], zone["points"][0]))
        plt.fill(polygon[:, 0], polygon[:, 1], color="#ef4444", alpha=0.2, zorder=1)
        plt.plot(polygon[:, 0], polygon[:, 1], color="#dc2626", linewidth=1.5, zorder=2)
        center = np.mean(zone["points"], axis=0)
        plt.annotate(zone["label"], center, color="#991b1b", fontsize=8, ha="center")
    if keepout_zones:
        plt.plot([], [], color="#dc2626", linewidth=1.5, label="Keepout zone")
    status_colors = {
        "pending": "#3b82f6", "completed": "#22c55e", "failed": "#ef4444",
        "active": "#f97316", "skipped": "#9ca3af",
    }
    # Color each path by the waypoint leg it approaches. Repeated replans for
    # that same leg share its status color; only one legend entry per status.
    destination_status = {name: status for _, _, name, status, _ in waypoints}
    labelled_statuses = set()
    for _, _, plan in planned_paths:
        if len(plan) == 0:
            continue
        endpoint = plan[-1]
        if waypoints:
            closest = min(waypoints, key=lambda wp: np.hypot(wp[0] - endpoint[0], wp[1] - endpoint[1]))
            status = closest[3]
        else:
            status = "pending"
        color = status_colors.get(status, "#3b82f6")
        plt.plot(
            plan[:, 0], plan[:, 1], "--", linewidth=1.3, color=color, alpha=0.65,
            label=f"Nav2 path: {status}" if status not in labelled_statuses else None,
        )
        labelled_statuses.add(status)
    labelled_statuses = set()
    for x, y, name, status, feature_color in waypoints:
        color = feature_color if feature_color.startswith("#") else status_colors.get(status, "#3b82f6")
        status_label = f"Waypoint: {status}"
        plt.scatter(
            x, y, color=color, marker="o", s=55, edgecolors="white", linewidths=0.8,
            zorder=5, label=status_label if status not in labelled_statuses else None,
        )
        labelled_statuses.add(status)
        plt.annotate(name, (x, y), xytext=(5, 5), textcoords="offset points", color=color)
    plt.scatter(*gt_position[0, :2], marker="o", label="Start")
    plt.scatter(*gt_position[-1, :2], marker="x", label="Ground-truth end")
    plt.xlabel("East / X [m]")
    plt.ylabel("North / Y [m]")
    plt.title("FAST-LIO2 trajectory vs ground truth")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    if show:
        backend = matplotlib.get_backend().lower()
        if "agg" in backend:
            print(f"Plot saved to {path}; backend {matplotlib.get_backend()} has no GUI to display it.")
        else:
            plt.show()
    plt.close()


def recording_start_datetime(bag_path: Path, override: str | None) -> datetime | None:
    if override:
        parsed = datetime.fromisoformat(override.replace("Z", "+00:00"))
        return parsed.replace(tzinfo=timezone.utc) if parsed.tzinfo is None else parsed
    pattern = re.compile(r"rosbag2_(\d{4}_\d{2}_\d{2}-\d{2}_\d{2}_\d{2})")
    for candidate in (bag_path.name, bag_path.parent.name):
        match = pattern.search(candidate)
        if match:
            # rosbag2 folder timestamps are UTC; convert them to Swedish local time for output.
            return datetime.strptime(match.group(1), "%Y_%m_%d-%H_%M_%S").replace(
                tzinfo=timezone.utc
            )
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 MCAP bag directory")
    parser.add_argument("--lio-topic", default="/terrascout1/fast_lio/odometry")
    parser.add_argument("--ground-truth-topic", default="/terrascout1/odom_gt")
    parser.add_argument("--plan-topic", default="/terrascout1/plan")
    parser.add_argument("--waypoints-topic", default="/terrascout1/waypoints_geo")
    parser.add_argument("--datum-topic", default="/terrascout1/fixposition/datum")
    parser.add_argument("--keepout-marker-topic", default="/terrascout1/keepout_zone_marker")
    parser.add_argument(
        "--plan-to-odom-x", type=float,
        help="known map-to-odom translation X [m]; omit unless calibrated",
    )
    parser.add_argument(
        "--plan-to-odom-y", type=float,
        help="known map-to-odom translation Y [m]; omit unless calibrated",
    )
    parser.add_argument(
        "--plan-to-odom-yaw-deg", type=float, default=0.0,
        help="known map-to-odom yaw [deg] for transforming map-frame paths into odom",
    )
    parser.add_argument("--max-dt", type=float, default=0.03)
    parser.add_argument("--imu-height", type=float, default=0.48)
    parser.add_argument(
        "--out-dir",
        help="existing mission result bundle directory; defaults to the newest tester-created mission folder",
    )
    parser.add_argument(
        "--mission", choices=("1", "2", "3", "4"),
        help="mission preset used for this recording (included in output filenames)",
    )
    parser.add_argument(
        "--mode", choices=("ordered", "unordered"),
        help="mission ordering mode used for this recording (included in output filenames)",
    )
    parser.add_argument(
        "--show-trajectory", action="store_true",
        help="display the FAST-LIO/ground-truth route overlay after saving it",
    )
    parser.add_argument(
        "--recording-start",
        help="UTC recording start (ISO-8601); otherwise inferred from rosbag2_YYYY_MM_DD-HH_MM_SS",
    )
    args = parser.parse_args()
    if args.out_dir is not None and not args.out_dir.strip():
        parser.error("--out-dir cannot be empty; pass a mission result folder")

    topics = [args.lio_topic, args.ground_truth_topic]
    data, frames = read_odometry_bag(args.bag, topics)
    if frames[args.ground_truth_topic] != {("odom", "terrascout1/base_footprint")}:
        raise RuntimeError(
            "Ground-truth frames must be ('odom', 'terrascout1/base_footprint'); "
            f"got {frames[args.ground_truth_topic]}"
        )
    if frames[args.lio_topic] != {("camera_init", "body")}:
        print(f"WARNING: unexpected FAST-LIO2 frames: {frames[args.lio_topic]}")

    t, lio_p, lio_r, gt_base_p, gt_base_r, nearest_dt = match_and_interpolate(
        data[args.lio_topic], data[args.ground_truth_topic], args.max_dt
    )
    gt_imu_p = gt_base_p + gt_base_r.apply(np.array([0.0, 0.0, args.imu_height]))
    align_r = gt_base_r[0] * lio_r[0].inv()
    align_t = gt_imu_p[0] - align_r.apply(lio_p[0])
    lio_p_aligned = align_r.apply(lio_p) + align_t
    lio_r_aligned = align_r * lio_r
    position_error = lio_p_aligned - gt_imu_p
    rotation_error = gt_base_r.inv() * lio_r_aligned
    rpy_error_deg = rotation_error.as_euler("xyz", degrees=True)
    rotation_error_deg = np.rad2deg(rotation_error.magnitude())
    horizontal_error = np.linalg.norm(position_error[:, :2], axis=1)
    spatial_error = np.linalg.norm(position_error, axis=1)
    seconds = t - t[0]
    planned_paths = read_plan_bag(args.bag, args.plan_topic, float(t[0]), float(t[-1]))
    plan_frames = {frame_id for _, frame_id, _ in planned_paths}
    if len(plan_frames) > 1:
        raise RuntimeError(f"Planned path changes frame in the plot interval: {sorted(plan_frames)}")
    plan_frame = next(iter(plan_frames), None)
    ground_truth_frame = next(iter(frames[args.ground_truth_topic]))[0]
    map_to_odom = read_map_odom_transform(
        args.bag, args.plan_topic.strip("/").split("/")[0]
    )
    if plan_frame and plan_frame != ground_truth_frame:
        if plan_frame == "map":
            if map_to_odom is not None:
                planned_paths = [
                    (stamp, frame_id, map_to_odom(points))
                    for stamp, frame_id, points in planned_paths
                ]
                print("Transformed recorded map-frame Nav2 plan into odom for plotting")
            elif args.plan_to_odom_x is not None and args.plan_to_odom_y is not None:
                yaw = np.deg2rad(args.plan_to_odom_yaw_deg)
                rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
                translation = np.array([args.plan_to_odom_x, args.plan_to_odom_y])
                planned_paths = [
                    (stamp, frame_id, (points - translation) @ rotation)
                    for stamp, frame_id, points in planned_paths
                ]
                print("Transformed map-frame Nav2 paths to odom using the supplied calibration")
            else:
                print(
                    f"WARNING: path frame {plan_frame!r} differs from plotted ground-truth frame; "
                    "overlay hidden because no recorded or supplied map-to-odom transform was found."
                )
                planned_paths = []
        else:
            print(
                f"WARNING: path frame {plan_frame!r} differs from plotted ground-truth frame "
                f"{ground_truth_frame!r}; overlay hidden."
            )
            planned_paths = []
    waypoint_geo, waypoint_snapshot_time = read_waypoint_snapshot(
        args.bag, args.waypoints_topic, float(t[-1])
    )
    keepout_zones = read_keepout_zones(
        args.bag, args.keepout_marker_topic, float(t[-1]), map_to_odom
    )
    waypoint_points = []
    if waypoint_geo:
        datum = read_datum(args.bag, args.datum_topic)
        if datum is not None:
            origin_lat, origin_lon = datum
            for lon, lat, name, status, color in waypoint_geo:
                waypoint_points.append(
                    (*local_enu_xy(lon, lat, origin_lon, origin_lat), name, status, color)
                )
        else:
            print("Waypoint markers omitted because they cannot be placed in the odometry frame safely")
    home_markers = [point for point in waypoint_points if point[2].lower() == "home"]
    if home_markers:
        home = home_markers[-1]
        final_home_distance = float(np.linalg.norm(gt_imu_p[-1, :2] - np.array(home[:2])))
        all_waypoints_done = all(
            status == "completed" for _, _, name, status, _ in waypoint_points
            if name.lower() != "home"
        )
        if all_waypoints_done and final_home_distance <= 1.0:
            waypoint_points = [
                (x, y, name, "completed" if name.lower() == "home" else status, color)
                for x, y, name, status, color in waypoint_points
            ]
            print(
                f"Home marker shown completed: all waypoints completed and final "
                f"ground-truth position is {final_home_distance:.2f} m from Home"
            )
    recording_start = recording_start_datetime(args.bag, args.recording_start)
    if recording_start is None:
        file_datetime = datetime.now(SWEDISH_TIMEZONE)
        datetime_tag = file_datetime.strftime("%Y%m%dT%H%M%S")
    else:
        datetime_tag = recording_start.astimezone(SWEDISH_TIMEZONE).strftime(
            "%Y%m%dT%H%M%S"
        )
    if args.out_dir is None:
        workspace_root = Path(__file__).resolve().parents[4]
        results_root = workspace_root / "src" / "terrascout_mission_tester" / "mission_results"
        candidates = sorted(
            (path for path in results_root.glob("mission_*") if path.is_dir()),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        if not candidates:
            args.out_dir = results_root / f"mission_{datetime_tag}"
            print(f"No existing mission bundle found; creating one from recording timestamp: {args.out_dir}")
        else:
            args.out_dir = candidates[0]
    else:
        args.out_dir = Path(args.out_dir)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    output_prefix = f"{datetime_tag}_"

    print("Relative trajectory drift after first-pose alignment:")
    for axis, name in enumerate(("x", "y", "z")):
        print_stats(name, position_error[:, axis], "m")
    print_stats("horizontal", horizontal_error, "m")
    print_stats("3D", spatial_error, "m")
    for axis, name in enumerate(("roll", "pitch", "yaw")):
        print_stats(name, rpy_error_deg[:, axis], "deg")
    print_stats("rotation angle", rotation_error_deg, "deg")

    headers = (
        "datetime,elapsed_s,gt_imu_x_m,gt_imu_y_m,gt_imu_z_m,lio_aligned_x_m,"
        "lio_aligned_y_m,lio_aligned_z_m,error_x_m,error_y_m,error_z_m,error_xy_m,"
        "error_3d_m,error_roll_deg,error_pitch_deg,error_yaw_deg,error_rotation_deg,"
        "nearest_gt_stamp_dt_s"
    ).split(",")
    with (args.out_dir / f"{output_prefix}fast_lio_error.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        for index, elapsed in enumerate(seconds):
            timestamp = (
                (recording_start + timedelta(seconds=float(t[index])))
                .astimezone(SWEDISH_TIMEZONE)
                .isoformat(timespec="milliseconds")
                if recording_start is not None
                else ""
            )
            writer.writerow([
                timestamp, elapsed, *gt_imu_p[index], *lio_p_aligned[index],
                *position_error[index], horizontal_error[index], spatial_error[index],
                *rpy_error_deg[index], rotation_error_deg[index], nearest_dt[index],
            ])
    save_plot(
        args.out_dir / f"{output_prefix}position_error.png", seconds,
        {"x": position_error[:, 0], "y": position_error[:, 1], "z": position_error[:, 2],
         "XY norm": horizontal_error},
        "Position error [m]", "FAST-LIO2 vs ground truth: position error",
    )
    save_plot(
        args.out_dir / f"{output_prefix}orientation_error.png", seconds,
        {"roll": rpy_error_deg[:, 0], "pitch": rpy_error_deg[:, 1], "yaw": rpy_error_deg[:, 2]},
        "Relative rotation error [deg]", "FAST-LIO2 vs ground truth: orientation error",
    )
    save_trajectory_plot(
        args.out_dir / f"{output_prefix}trajectory_comparison.png", gt_imu_p, lio_p_aligned,
        planned_paths=planned_paths, waypoints=waypoint_points, keepout_zones=keepout_zones,
        show=args.show_trajectory,
    )
    if recording_start is None:
        print("WARNING: no bag start datetime found; datetime is blank (use --recording-start).")
    else:
        print(
            "CSV datetime is shown in Europe/Stockholm time, based on recording start "
            f"{recording_start.astimezone(SWEDISH_TIMEZONE).isoformat()}"
        )
    print(f"Saved results to {args.out_dir.resolve()}")


if __name__ == "__main__":
    main()
