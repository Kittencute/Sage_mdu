#!/usr/bin/env python3
"""Compare FAST-LIO2 odometry with Gazebo ground truth from an MCAP ROS 2 bag.

FAST-LIO2 is aligned to the ground-truth IMU pose at the first matched sample.
The reported errors measure relative trajectory drift, not absolute localization.
"""

import argparse
from pathlib import Path

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 MCAP bag directory")
    parser.add_argument("--lio-topic", default="/terrascout1/fast_lio/odometry")
    parser.add_argument("--ground-truth-topic", default="/terrascout1/odom_gt")
    parser.add_argument("--max-dt", type=float, default=0.03)
    parser.add_argument("--imu-height", type=float, default=0.48)
    parser.add_argument("--out-dir", type=Path, default=Path("fast_lio_error_results"))
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

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

    print("Relative trajectory drift after first-pose alignment:")
    for axis, name in enumerate(("x", "y", "z")):
        print_stats(name, position_error[:, axis], "m")
    print_stats("horizontal", horizontal_error, "m")
    print_stats("3D", spatial_error, "m")
    for axis, name in enumerate(("roll", "pitch", "yaw")):
        print_stats(name, rpy_error_deg[:, axis], "deg")
    print_stats("rotation angle", rotation_error_deg, "deg")

    columns = np.column_stack(
        [seconds, gt_imu_p, lio_p_aligned, position_error, horizontal_error, spatial_error,
         rpy_error_deg, rotation_error_deg, nearest_dt]
    )
    np.savetxt(
        args.out_dir / "fast_lio_error.csv", columns, delimiter=",", comments="",
        header=("time_s,gt_imu_x_m,gt_imu_y_m,gt_imu_z_m,lio_aligned_x_m,lio_aligned_y_m,"
                "lio_aligned_z_m,error_x_m,error_y_m,error_z_m,error_xy_m,error_3d_m,"
                "error_roll_deg,error_pitch_deg,error_yaw_deg,error_rotation_deg,"
                "nearest_gt_stamp_dt_s"),
    )
    save_plot(
        args.out_dir / "position_error.png", seconds,
        {"x": position_error[:, 0], "y": position_error[:, 1], "z": position_error[:, 2],
         "XY norm": horizontal_error},
        "Position error [m]", "FAST-LIO2 vs ground truth: position error",
    )
    save_plot(
        args.out_dir / "orientation_error.png", seconds,
        {"roll": rpy_error_deg[:, 0], "pitch": rpy_error_deg[:, 1], "yaw": rpy_error_deg[:, 2]},
        "Relative rotation error [deg]", "FAST-LIO2 vs ground truth: orientation error",
    )
    print(f"Saved results to {args.out_dir.resolve()}")


if __name__ == "__main__":
    main()
