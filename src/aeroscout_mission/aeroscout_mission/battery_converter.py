#!/usr/bin/env python3

import math

import rclpy
from px4_msgs.msg import BatteryStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from fleet_common import signals

PX4_INVALID_REMAINING = -1.0
PX4_INVALID_VOLTAGE = 0.0
PX4_INVALID_CURRENT = -1.0
MAH_PER_AH = 1000.0


class BatteryConverter(Node):
    def __init__(self):
        super().__init__("battery_converter")
        self.create_subscription(
            BatteryStatus, "fmu/out/battery_status", self._battery_cb, qos_profile_sensor_data
        )
        self._publisher = self.create_publisher(BatteryState, "battery_state", 10)

    def _battery_cb(self, status: BatteryStatus):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.present = status.connected
        msg.location = str(status.id)
        msg.percentage = math.nan if status.remaining == PX4_INVALID_REMAINING else status.remaining
        msg.voltage = math.nan if status.voltage_v == PX4_INVALID_VOLTAGE else status.voltage_v
        msg.current = math.nan if status.current_a == PX4_INVALID_CURRENT else -status.current_a
        msg.temperature = status.temperature
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = status.capacity / MAH_PER_AH if status.capacity else math.nan
        msg.cell_voltage = [float(volts) for volts in status.voltage_cell_v[: status.cell_count]]
        self._publisher.publish(msg)


def main(args=None):
    signals.init(args=args)
    node = BatteryConverter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
