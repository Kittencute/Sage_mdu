#!/usr/bin/env python3

from __future__ import annotations

import json

import rclpy
from fleet_interfaces.msg import MissionCommand, MissionPlan
from foxglove_msgs.msg import GeoJSON
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from fleet_common import signals
from fleet_common.geodesy import llh_to_enu
from fleet_common.qos import latched_qos
from fleet_common.waypoint_geojson import build_waypoints_geojson

_ODOM_FRAME = "odom"


class MissionProgress(Node):
    def __init__(self):
        super().__init__("mission_progress")

        self._geo_pub = self.create_publisher(GeoJSON, "waypoints_geo", latched_qos())
        self._path_pub = self.create_publisher(Path, "mission_path", latched_qos())

        self.create_subscription(MissionPlan, "mission_plan", self._plan_cb, latched_qos())
        self.create_subscription(
            String,
            "mission_progress",
            self._progress_cb,
            10,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self._local_position_cb,
            qos_profile_sensor_data,
        )

        self._waypoints: list[dict[str, float]] = []
        self._ref_lat: float | None = None
        self._ref_lon: float | None = None

    def _plan_cb(self, plan: MissionPlan):
        self._waypoints = []
        for command in plan.commands:
            if command.kind == MissionCommand.WAYPOINT:
                self._waypoints.append(
                    {
                        "lat": command.latitude,
                        "lon": command.longitude,
                        "alt": command.altitude_agl,
                    }
                )
            elif command.kind == MissionCommand.HOME:
                self._waypoints.append(
                    {
                        "lat": plan.home.latitude,
                        "lon": plan.home.longitude,
                        "alt": command.altitude_agl,
                    }
                )
        self.get_logger().info(f"Loaded {len(self._waypoints)} waypoints from mission {plan.id}")
        self._publish_geojson(0, failed=False)
        self._publish_path()

    def _local_position_cb(self, msg: VehicleLocalPosition):
        if not (msg.xy_global and msg.z_global):
            return
        new_ref = (float(msg.ref_lat), float(msg.ref_lon))
        if new_ref != (self._ref_lat, self._ref_lon):
            self._ref_lat, self._ref_lon = new_ref
            self._publish_path()

    def _progress_cb(self, msg: String):
        if not self._waypoints:
            return

        try:
            progress = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"parse mission progress failed: cause: {exc}") from exc

        index = progress["index"]
        state = progress["state"]
        failed = state == "failed"

        current_index = max(0, index) if not failed else index
        self._publish_geojson(current_index, failed)

    def _publish_geojson(self, current_index: int, failed: bool):
        geo_msg = GeoJSON()
        geo_msg.geojson = build_waypoints_geojson(self._waypoints, current_index, failed=failed)
        self._geo_pub.publish(geo_msg)

    def _publish_path(self):
        if self._ref_lat is None or self._ref_lon is None or not self._waypoints:
            return
        stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = _ODOM_FRAME
        for wp in self._waypoints:
            east, north, _ = llh_to_enu(
                wp["lat"], wp["lon"], 0.0, self._ref_lat, self._ref_lon, 0.0
            )
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = _ODOM_FRAME
            pose.pose.position.x = east
            pose.pose.position.y = north
            pose.pose.position.z = wp["alt"]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)


def main():
    signals.init()
    node = MissionProgress()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
