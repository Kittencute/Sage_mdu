#!/usr/bin/env python3

from __future__ import annotations

import sys

import rclpy
from px4_msgs.msg import HealthReport
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from fleet_common import signals

HEALTH_TOPIC = "fmu/out/health_report"
SENSOR_COMPONENTS = {
    1 << 1: "absolute_pressure",
    1 << 3: "gps",
    1 << 27: "magnetometer",
    1 << 28: "accel",
    1 << 29: "gyro",
}


class Px4HealthWatch(Node):
    def __init__(self) -> None:
        super().__init__("px4_health_watch")
        self.failure: str | None = None
        self._reported: tuple[int, int, int] | None = None
        self._healthy = 0
        self.create_subscription(
            HealthReport, HEALTH_TOPIC, self._on_report, qos_profile_sensor_data
        )

    def _on_report(self, report: HealthReport) -> None:
        flags = (
            report.health_is_present_flags,
            report.health_error_flags,
            report.health_warning_flags,
        )
        if flags != self._reported:
            self._reported = flags
            self.get_logger().info(
                f"PX4 health: present flags {flags[0]:#x}, error flags {flags[1]:#x}, "
                f"warning flags {flags[2]:#x}"
            )
        self._healthy |= report.health_is_present_flags & ~report.health_error_flags
        failed = [
            name
            for bit, name in SENSOR_COMPONENTS.items()
            if self._healthy & bit and report.health_error_flags & bit
        ]
        if failed:
            self.failure = f"PX4 reports failed sensors: {', '.join(failed)}"


def main() -> int:
    signals.init()
    node = Px4HealthWatch()
    try:
        while node.failure is None:
            rclpy.spin_once(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(
        f"px4 health watch failed: operation: watch {HEALTH_TOPIC}; cause: {node.failure}",
        file=sys.stderr,
        flush=True,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
