#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2

from fleet_common import signals
from fleet_common.params import declare

SENSOR_DATA_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

MIN_RANGE_M = 0.5
_LIVOX_TAG_OFFSET = 16

_LIVOX_NOISE_MASK = 0x3F


def _valid_mask(
    data: bytes,
    point_step: int,
    min_range_sq: float = 0.25,
    filter_noise_tag: bool = True,
    xyz_offsets: tuple[int, int, int] = (0, 4, 8),
    tag_offset: int | None = _LIVOX_TAG_OFFSET,
) -> np.ndarray:
    n_points = len(data) // point_step if point_step > 0 else 0
    if n_points == 0:
        return np.array([], dtype=bool)

    fields: dict = {
        "names": ["x", "y", "z"],
        "formats": [np.float32, np.float32, np.float32],
        "offsets": list(xyz_offsets),
        "itemsize": point_step,
    }
    has_tag = filter_noise_tag and tag_offset is not None and point_step > tag_offset
    if has_tag:
        fields["names"].append("tag")
        fields["formats"].append(np.uint8)
        fields["offsets"].append(tag_offset)

    pts = np.frombuffer(data, dtype=np.dtype(fields))

    finite = np.isfinite(pts["x"]) & np.isfinite(pts["y"]) & np.isfinite(pts["z"])
    nonzero = (pts["x"] != 0.0) | (pts["y"] != 0.0) | (pts["z"] != 0.0)

    range_sq = pts["x"] ** 2 + pts["y"] ** 2 + pts["z"] ** 2
    beyond_blind = range_sq > min_range_sq

    mask = finite & nonzero & beyond_blind

    if has_tag:
        mask &= (pts["tag"] & _LIVOX_NOISE_MASK) == 0

    return mask


class LidarFilter(Node):
    def __init__(self):
        super().__init__("lidar_filter")
        self._filter_noise_tag = declare(
            self,
            "filter_noise_tag",
            rclpy.Parameter.Type.BOOL,
            "Whether to drop points whose per-point tag marks them as noise",
            constraints="true requires the cloud to carry a tag field",
        )
        self.get_logger().info(
            f"min_range={MIN_RANGE_M:.2f}m, filter_noise_tag={self._filter_noise_tag}"
        )
        self.pub = self.create_publisher(PointCloud2, "cloud_out", SENSOR_DATA_QOS)
        self.sub = self.create_subscription(
            PointCloud2,
            "cloud_in",
            self._callback,
            SENSOR_DATA_QOS,
        )

    def _callback(self, msg: PointCloud2):
        n_points = msg.width * msg.height
        if n_points == 0:
            self.pub.publish(msg)
            return

        offsets = {field.name: field.offset for field in msg.fields}
        missing = [name for name in ("x", "y", "z") if name not in offsets]
        if missing:
            raise RuntimeError(
                f"filter lidar cloud failed: cause: the cloud has no field {missing[0]}"
            )
        if self._filter_noise_tag and "tag" not in offsets:
            raise RuntimeError(
                "filter lidar cloud failed: cause: filter_noise_tag is set but the cloud has no tag field"
            )
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.row_step)
        raw = np.ascontiguousarray(rows[:, : msg.width * msg.point_step]).reshape(
            n_points, msg.point_step
        )
        mask = _valid_mask(
            raw.tobytes(),
            msg.point_step,
            MIN_RANGE_M**2,
            self._filter_noise_tag,
            (offsets["x"], offsets["y"], offsets["z"]),
            offsets.get("tag"),
        )
        filtered = raw[mask]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(mask.sum())
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.width * msg.point_step
        out.data = filtered.tobytes()
        out.is_dense = True
        self.pub.publish(out)


def main(args=None):
    signals.init(args=args)
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
