#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from fleet_common import signals
from fleet_common.params import declare

SLOTS = ("left", "right")
FULL_CAPACITY_PCT = 100.0


class SimulatedBatteryNode(Node):
    def __init__(self):
        super().__init__("simulated_battery")

        publish_period = declare(
            self,
            "publish_period",
            Parameter.Type.DOUBLE,
            "Interval between battery state messages, in seconds",
            0.001,
            3600.0,
        )
        self._installed_packs = list(
            declare(
                self,
                "installed_packs",
                Parameter.Type.STRING_ARRAY,
                "Battery slots that hold a pack, each reported as its own battery state",
                constraints="each entry is left or right, listed at most once, at least one entry",
            )
        )
        self._safety_stop_period = declare(
            self,
            "safety_stop_period",
            Parameter.Type.DOUBLE,
            "Interval between safety stop messages and capacity updates, in seconds",
            0.001,
            60.0,
        )
        self._capacity_pct = declare(
            self,
            "initial_capacity_pct",
            Parameter.Type.DOUBLE,
            "Simulated pack charge at start, in percent of full capacity",
            0.0,
            100.0,
        )
        self._drain_pct_per_s = declare(
            self,
            "drain_pct_per_s",
            Parameter.Type.DOUBLE,
            "Simulated charge lost per second, in percent of full capacity; zero never drains",
            0.0,
            100.0,
        )
        self._min_capacity_pct = declare(
            self,
            "min_capacity_pct",
            Parameter.Type.INTEGER,
            "Charge below which the safety stop is raised, in percent of full capacity",
            0,
            100,
        )

        if publish_period <= 0.0 or self._safety_stop_period <= 0.0:
            raise ValueError(
                "configure simulated battery failed: cause: publish_period and "
                f"safety_stop_period must be positive, got {publish_period} and {self._safety_stop_period}"
            )
        if (
            not self._installed_packs
            or len(set(self._installed_packs)) != len(self._installed_packs)
            or not set(self._installed_packs) <= set(SLOTS)
        ):
            raise ValueError(
                "configure simulated battery failed: cause: installed_packs must list each installed pack "
                f"once from {SLOTS}, got {self._installed_packs}"
            )
        if not 0.0 <= self._capacity_pct <= FULL_CAPACITY_PCT:
            raise ValueError(
                "configure simulated battery failed: cause: initial_capacity_pct must be "
                f"between 0 and {FULL_CAPACITY_PCT}, got {self._capacity_pct}"
            )
        if self._drain_pct_per_s < 0.0:
            raise ValueError(
                "configure simulated battery failed: cause: drain_pct_per_s must not be "
                f"negative, got {self._drain_pct_per_s}"
            )

        self.pub_battery = self.create_publisher(BatteryState, "battery_state", 10)
        self.pub_safety_stop = self.create_publisher(Bool, "safety_stop", 10)
        self.create_timer(publish_period, self._publish_battery)
        self.create_timer(self._safety_stop_period, self._drain_and_publish_safety_stop)

    def _publish_battery(self):
        for location in self._installed_packs:
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = self._capacity_pct / FULL_CAPACITY_PCT
            msg.present = True
            msg.location = location
            self.pub_battery.publish(msg)

    def _drain_and_publish_safety_stop(self):
        self._capacity_pct = max(
            0.0, self._capacity_pct - self._drain_pct_per_s * self._safety_stop_period
        )
        self.pub_safety_stop.publish(Bool(data=self._capacity_pct < self._min_capacity_pct))


def main(args=None):
    signals.init(args=args)
    node = SimulatedBatteryNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
