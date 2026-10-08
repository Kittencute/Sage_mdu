#!/usr/bin/env python3

from __future__ import annotations

import sys
import struct
from dataclasses import dataclass

import rclpy
from fleet_interfaces.srv import GetKeepoutZone, SetKeepoutZone
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField

from fleet_common import signals
from fleet_common.geodesy import llh_to_enu
from fleet_common.params import declare
from fleet_common.qos import latched_qos
from terrascout_mission.keepout_geometry import build_rectangle, finite, positive


@dataclass
class KeepoutZone:
    label: str
    latitude: float
    longitude: float
    x_length: float
    y_length: float
    yaw_deg: float
    polygon: list[tuple[float, float]]
    points: list[tuple[float, float, float]]


class KeepoutServer(Node):
    def __init__(self) -> None:
        super().__init__("keepout_server")
        self._global_frame = declare(
            self,
            "global_frame",
            Parameter.Type.STRING,
            "Frame in which keepout polygons are published",
            constraints="a tf frame id",
        )
        self._datum_topic = declare(
            self,
            "datum_topic",
            Parameter.Type.STRING,
            "Topic that publishes the geodetic origin of the map frame",
            constraints="a ROS topic name",
        )
        self._zone_topic = declare(
            self,
            "zone_topic",
            Parameter.Type.STRING,
            "Topic used by Nav2 collision monitor for stop polygons",
            constraints="a ROS topic name",
        )
        self._obstacle_topic = declare(
            self,
            "obstacle_topic",
            Parameter.Type.STRING,
            "PointCloud2 topic used by Nav2 costmaps for keepout obstacles",
            constraints="a ROS topic name",
        )
        self._publish_period_s = declare(
            self,
            "publish_period_s",
            Parameter.Type.DOUBLE,
            "How often to republish keepout geometry for late subscribers, in seconds",
            0.1,
            5.0,
        )
        self._datum: NavSatFix | None = None
        self._zones: dict[str, KeepoutZone] = {}
        self._active_label: str | None = None
        self._active_points: list[tuple[float, float, float]] = []

        self._zone_pub = self.create_publisher(PolygonStamped, self._zone_topic, latched_qos())
        self._obstacle_pub = self.create_publisher(PointCloud2, self._obstacle_topic, latched_qos())
        self.create_subscription(NavSatFix, self._datum_topic, self._on_datum, latched_qos())
        self.create_service(SetKeepoutZone, "set_keepout_zone", self._on_set_keepout_zone)
        self.create_service(GetKeepoutZone, "get_keepout_zone", self._on_get_keepout_zone)
        self.create_timer(self._publish_period_s, self._republish_active_zone)

    def _on_datum(self, msg: NavSatFix) -> None:
        if self._datum is None:
            self._datum = msg
            self.get_logger().info(
                f"datum locked from {self._datum_topic}: {msg.latitude:.7f},{msg.longitude:.7f}"
            )

    def _on_set_keepout_zone(self, request, response):
        try:
            if request.clear:
                zone = self._zones.pop(request.label, None)
                if zone is None:
                    raise ValueError(f"zone '{request.label}' not found")
                if self._active_label == request.label:
                    self._active_label = None
                    self._active_points = []
                    self._zone_pub.publish(PolygonStamped(header=self._header()))
                    self._obstacle_pub.publish(self._pointcloud([]))
                response.accepted = True
                response.reason = f"zone '{request.label}' deleted"
                self.get_logger().info(response.reason)
                return response

            if self._datum is None:
                raise ValueError(f"no datum on {self._datum_topic}")
            finite(request.latitude, "latitude")
            finite(request.longitude, "longitude")
            positive(request.x_length, "x_length")
            positive(request.y_length, "y_length")
            finite(request.yaw_deg, "yaw_deg")

            east, north, _ = llh_to_enu(
                request.latitude,
                request.longitude,
                0.0,
                self._datum.latitude,
                self._datum.longitude,
                self._datum.altitude,
            )
            polygon = build_rectangle(
                center_x=east,
                center_y=north,
                x_length=request.x_length,
                y_length=request.y_length,
                yaw_deg=request.yaw_deg,
            )
            points = _sample_zone(polygon)
            existed = request.label in self._zones
            self._zones[request.label] = KeepoutZone(
                label=request.label,
                latitude=request.latitude,
                longitude=request.longitude,
                x_length=request.x_length,
                y_length=request.y_length,
                yaw_deg=request.yaw_deg,
                polygon=polygon,
                points=points,
            )
            self._active_label = request.label
            self._active_points = points

            msg = PolygonStamped()
            msg.header = self._header()
            msg.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in polygon]
            self._zone_pub.publish(msg)
            self._obstacle_pub.publish(self._pointcloud(points))

            response.accepted = True
            action = "updated" if existed else "created"
            response.reason = (
                f"zone '{request.label}' {action} at {request.latitude:.7f},{request.longitude:.7f} "
                f"size {request.x_length:.2f}x{request.y_length:.2f} m yaw {request.yaw_deg:.1f} deg"
            )
            self.get_logger().info(response.reason)
            return response
        except ValueError as exc:
            response.accepted = False
            response.reason = str(exc)
            return response

    def _on_get_keepout_zone(self, request, response):
        zone = self._zones.get(request.label)
        if zone is None:
            response.found = False
            response.reason = f"zone '{request.label}' not found"
            return response

        response.found = True
        response.reason = f"zone '{request.label}' found"
        response.latitude = zone.latitude
        response.longitude = zone.longitude
        response.x_length = zone.x_length
        response.y_length = zone.y_length
        response.yaw_deg = zone.yaw_deg
        response.active = self._active_label == zone.label
        return response

    def _republish_active_zone(self) -> None:
        self._obstacle_pub.publish(self._pointcloud(self._active_points))

    def _pointcloud(self, points: list[tuple[float, float, float]]) -> PointCloud2:
        msg = PointCloud2()
        msg.header = self._header()
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", x, y, z) for x, y, z in points)
        return msg

    def _header(self):
        header = PolygonStamped().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._global_frame
        return header


def main() -> None:
    signals.init()
    node = KeepoutServer()
    try:
        node.get_logger().info("ready for keepout zones")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _sample_zone(polygon: list[tuple[float, float]], spacing: float = 0.4) -> list[tuple[float, float, float]]:
    min_x = min(x for x, _ in polygon)
    max_x = max(x for x, _ in polygon)
    min_y = min(y for _, y in polygon)
    max_y = max(y for _, y in polygon)
    edges = list(zip(polygon, polygon[1:] + polygon[:1], strict=True))
    points: list[tuple[float, float, float]] = []
    y = min_y
    while y <= max_y + 1e-9:
        x = min_x
        while x <= max_x + 1e-9:
            if _inside_convex_polygon(x, y, edges):
                points.append((x, y, 0.0))
            x += spacing
        y += spacing
    return points


def _inside_convex_polygon(
    x: float,
    y: float,
    edges: list[tuple[tuple[float, float], tuple[float, float]]],
) -> bool:
    sign: float | None = None
    for (x1, y1), (x2, y2) in edges:
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) < 1e-9:
            continue
        current_sign = cross > 0.0
        if sign is None:
            sign = current_sign
        elif sign != current_sign:
            return False
    return True


if __name__ == "__main__":
    sys.exit(main())
