#!/usr/bin/env python3
"""Collect Foxglove map clicks and submit them as a TerraScout mission."""
from __future__ import annotations

import math

from fleet_common.geodesy import enu_to_llh, enu_to_map, llh_to_enu, map_to_enu
from fleet_common.qos import latched_qos
from fleet_common.send_mission import build_plan_items, order_waypoints
from fleet_interfaces.action import ExecuteMission
from fleet_interfaces.msg import MissionCommand, MissionPlan
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class FoxgloveMissionInput:
    """Attach waypoint collection services and subscriptions to an existing ROS node."""

    def __init__(self, node: Node, map_yaw_offset_rad: float = 0.0) -> None:
        self._node = node
        self._frame = node.get_parameter("global_frame").value
        self._map_yaw_offset_rad = map_yaw_offset_rad
        self._datum: NavSatFix | None = None
        self._position: NavSatFix | None = None
        self._points: list[tuple[float, float, float]] = []
        self._submission_pending = False
        self._action = ActionClient(node, ExecuteMission, "execute_mission")
        self._markers = node.create_publisher(MarkerArray, "mission_waypoint_markers", latched_qos())
        node.create_subscription(PointStamped, "clicked_point", self._on_click, 10)
        node.create_subscription(
            NavSatFix, node.get_parameter("datum_topic").value, self._on_datum, latched_qos()
        )
        node.create_subscription(
            NavSatFix, node.get_parameter("position_topic").value, self._on_position, 10
        )
        node.create_service(Trigger, "mission_waypoints/clear", self._on_clear)
        node.create_service(Trigger, "mission_waypoints/send_ordered", self._on_ordered)
        node.create_service(Trigger, "mission_waypoints/send_unordered", self._on_unordered)

    def __getattr__(self, name):
        return getattr(self._node, name)

    def _on_datum(self, message: NavSatFix) -> None:
        self._datum = message

    def _on_position(self, message: NavSatFix) -> None:
        if message.status.status >= NavSatStatus.STATUS_FIX:
            self._position = message

    def _on_click(self, message: PointStamped) -> None:
        if message.header.frame_id and message.header.frame_id.lstrip("/") != self._frame:
            self.get_logger().warning(
                f"ignored clicked point in frame '{message.header.frame_id}'; expected '{self._frame}'"
            )
            return
        if self._datum is None:
            self.get_logger().warning("ignored clicked point: GPS datum is not available")
            return
        point = message.point
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            self.get_logger().warning("ignored clicked point: coordinates are not finite")
            return
        east, north = map_to_enu(point.x, point.y, self._map_yaw_offset_rad)
        lat, lon, _ = enu_to_llh(
            east, north, point.z,
            self._datum.latitude, self._datum.longitude, self._datum.altitude,
        )
        self._points.append((lat, lon, 0.0))
        self.get_logger().info(
            f"collected waypoint {len(self._points)}: {lat:.8f},{lon:.8f}"
        )
        self._publish_markers()

    def _publish_markers(self) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.header = Header(frame_id=self._frame)
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for index, (lat, lon, _) in enumerate(self._points, start=1):
            if self._datum is None:
                break
            east, north, z = llh_to_enu(
                lat, lon, 0.0,
                self._datum.latitude, self._datum.longitude, self._datum.altitude,
            )
            x, y = enu_to_map(east, north, self._map_yaw_offset_rad)
            marker = Marker()
            marker.header = Header(frame_id=self._frame)
            marker.ns = "foxglove_mission_waypoints"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z + 0.4
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 1.0
            marker.color.a = 0.95
            marker.text = str(index)
            array.markers.append(marker)
        self._markers.publish(array)

    def _on_clear(self, request, response):
        self._points.clear()
        self._publish_markers()
        response.success = True
        response.message = "cleared collected mission waypoints"
        return response

    def _on_ordered(self, request, response):
        return self._submit(request, response, unordered=False)

    def _on_unordered(self, request, response):
        return self._submit(request, response, unordered=True)

    def _submit(self, request, response, unordered: bool):
        if not self._points:
            response.success = False
            response.message = "no clicked waypoints collected"
            return response
        if self._submission_pending:
            response.success = False
            response.message = "a waypoint mission submission is already pending"
            return response
        if self._position is None:
            response.success = False
            response.message = "robot GPS position is not available"
            return response
        if not self._action.server_is_ready():
            response.success = False
            response.message = "mission action server is unavailable"
            return response

        coords = list(self._points)
        if unordered:
            coords = order_waypoints(
                coords,
                "unordered",
                (self._position.latitude, self._position.longitude),
            )
        plan = MissionPlan()
        plan.id = int(self.get_clock().now().nanoseconds % 2_000_000_000)
        plan.home = GeoPoint(
            latitude=self._position.latitude,
            longitude=self._position.longitude,
            altitude=0.0,
        )
        plan.order_mode = "unordered" if unordered else "ordered"
        commands = []
        for item in build_plan_items(coords, altitude=10.0):
            command = MissionCommand()
            command.id = item["id"]
            command.kind = item["kind"]
            command.latitude = item.get("latitude", 0.0)
            command.longitude = item.get("longitude", 0.0)
            command.altitude_agl = item.get("altitude_agl", 0.0)
            command.hold_s = item.get("hold_s", 0.0)
            command.yaw_deg = float("nan")
            commands.append(command)
        plan.commands = commands
        goal = ExecuteMission.Goal()
        goal.plan = plan
        self._submission_pending = True
        self._action.send_goal_async(goal).add_done_callback(self._goal_sent)
        mode = "unordered" if unordered else "ordered"
        response.success = True
        response.message = f"submitted {len(coords)} {mode} waypoints; returning to mission start"
        return response

    def _goal_sent(self, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error("mission server rejected Foxglove waypoint mission")
            else:
                self.get_logger().info("Foxglove waypoint mission accepted")
                self._points.clear()
                self._publish_markers()
        except Exception as exc:  # Log asynchronous action transport errors.
            self.get_logger().error(f"failed to submit Foxglove waypoint mission: {exc}")
        finally:
            self._submission_pending = False
