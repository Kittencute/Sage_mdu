#!/usr/bin/env python3

import threading
import time

import can
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from fleet_common import signals
from fleet_common.params import declare
from terrascout_hardware.tattu_bms import BatteryManager


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__("battery_monitor")

        can_interface = declare(
            self,
            "can_interface",
            Parameter.Type.STRING,
            "Name of the CAN network interface the batteries are connected to",
            constraints="a Linux network interface name",
        )
        can_bitrate = declare(
            self,
            "can_bitrate",
            Parameter.Type.INTEGER,
            "CAN bus bitrate in bits per second",
            10000,
            1000000,
        )
        can_recv_timeout = declare(
            self,
            "can_recv_timeout",
            Parameter.Type.DOUBLE,
            "Longest wait for one CAN frame before checking again, in seconds",
            0.001,
            10.0,
        )
        publish_period = declare(
            self,
            "publish_period",
            Parameter.Type.DOUBLE,
            "Time between battery state messages, in seconds",
            0.01,
            3600.0,
        )
        installed_packs = declare(
            self,
            "installed_packs",
            Parameter.Type.STRING_ARRAY,
            "Battery slots that hold a pack",
            constraints="each entry is left or right, listed once, at least one",
        )
        left_battery_can_id = declare(
            self,
            "left_battery_can_id",
            Parameter.Type.INTEGER,
            "CAN identifier of the left battery",
            0,
            536870911,
        )
        right_battery_can_id = declare(
            self,
            "right_battery_can_id",
            Parameter.Type.INTEGER,
            "CAN identifier of the right battery",
            0,
            536870911,
        )
        design_capacity_mah = declare(
            self,
            "design_capacity_mah",
            Parameter.Type.INTEGER,
            "Rated charge capacity of one battery pack, in milliampere-hours",
            1,
            65535,
        )
        stale_timeout = declare(
            self,
            "stale_timeout",
            Parameter.Type.DOUBLE,
            "Age after which a battery's last frame counts as missing and triggers a safety stop, in seconds",
            0.01,
            60.0,
        )
        safety_stop_period = declare(
            self,
            "safety_stop_period",
            Parameter.Type.DOUBLE,
            "Time between safety stop messages, in seconds",
            0.01,
            10.0,
        )
        max_pack_voltage_mv = declare(
            self,
            "max_pack_voltage_mv",
            Parameter.Type.INTEGER,
            "Highest allowed pack voltage, in millivolts",
            30000,
            52200,
        )
        min_temp_c = declare(
            self,
            "min_temp_c",
            Parameter.Type.INTEGER,
            "Lowest allowed pack temperature, in degrees Celsius",
            -40,
            85,
        )
        max_temp_c = declare(
            self,
            "max_temp_c",
            Parameter.Type.INTEGER,
            "Highest allowed pack temperature, in degrees Celsius",
            -40,
            85,
        )
        max_current_ma = declare(
            self,
            "max_current_ma",
            Parameter.Type.INTEGER,
            "Highest allowed pack current in either direction, in milliamperes",
            0,
            330000,
        )
        min_cell_mv = declare(
            self,
            "min_cell_mv",
            Parameter.Type.INTEGER,
            "Lowest allowed cell voltage, in millivolts",
            2500,
            4350,
        )
        max_cell_mv = declare(
            self,
            "max_cell_mv",
            Parameter.Type.INTEGER,
            "Highest allowed cell voltage, in millivolts",
            2500,
            4350,
        )
        min_capacity_pct = declare(
            self,
            "min_capacity_pct",
            Parameter.Type.INTEGER,
            "Lowest allowed remaining charge, in percent",
            0,
            100,
        )
        min_health_pct = declare(
            self,
            "min_health_pct",
            Parameter.Type.INTEGER,
            "Lowest allowed battery health, in percent",
            0,
            100,
        )

        slot_can_ids = {"left": left_battery_can_id, "right": right_battery_can_id}
        installed_packs = list(installed_packs)
        if not installed_packs or len(set(installed_packs)) != len(installed_packs):
            raise ValueError(
                "configure battery monitor failed: cause: installed_packs must list each installed pack once, "
                f"got {installed_packs}"
            )
        unknown_packs = [name for name in installed_packs if name not in slot_can_ids]
        if unknown_packs:
            raise ValueError(
                f"configure battery monitor failed: cause: installed_packs names unknown slots {unknown_packs}, "
                f"expected names from {sorted(slot_can_ids)}"
            )
        self._pack_names = {slot_can_ids[name]: name for name in installed_packs}
        self._design_capacity_ah = design_capacity_mah / 1000.0
        self._can_recv_timeout = can_recv_timeout

        self.bus = can.Bus(channel=can_interface, interface="socketcan", bitrate=can_bitrate)
        self.manager = BatteryManager(
            installed_can_ids=self._pack_names,
            stale_timeout=stale_timeout,
            max_pack_voltage_mv=max_pack_voltage_mv,
            min_temp_c=min_temp_c,
            max_temp_c=max_temp_c,
            max_current_ma=max_current_ma,
            min_cell_mv=min_cell_mv,
            max_cell_mv=max_cell_mv,
            min_capacity_pct=min_capacity_pct,
            min_health_pct=min_health_pct,
        )
        self._lock = threading.Lock()
        self._sample_stamps: dict[int, float] = {}
        self._can_error = None
        self._can_error_guard = self.create_guard_condition(self._raise_can_error)

        self.pub_battery = self.create_publisher(BatteryState, "battery_state", 10)
        self.pub_safety_stop = self.create_publisher(Bool, "safety_stop", 10)
        self.create_timer(publish_period, self._publish_battery)
        self.create_timer(safety_stop_period, self._publish_safety_stop)

        self._reader_thread = threading.Thread(target=self._can_reader, daemon=True)
        self._reader_thread.start()
        self.get_logger().info(
            f"Battery monitor started on {can_interface} for packs {installed_packs}"
        )

    def _can_reader(self):
        try:
            while rclpy.ok():
                msg = self.bus.recv(timeout=self._can_recv_timeout)
                if msg is None:
                    continue
                with self._lock:
                    if self.manager.process_frame(msg.arbitration_id, bytes(msg.data)):
                        self._sample_stamps[msg.arbitration_id] = msg.timestamp
        except Exception as exc:
            self._can_error = exc
            self.get_logger().fatal(f"receive battery CAN frame failed: cause: {exc}")
            self._can_error_guard.trigger()

    def _raise_can_error(self):
        error = self._can_error
        if error is None:
            raise RuntimeError(
                "raise battery CAN receive failure failed: cause: "
                "guard condition triggered without a stored error"
            )
        raise RuntimeError(f"receive battery CAN frame failed: cause: {error}") from error

    def _publish_battery(self):
        now = time.monotonic()
        with self._lock:
            for can_id, battery in self.manager.batteries.items():
                msg = BatteryState()
                msg.header.stamp = Time(seconds=self._sample_stamps[can_id]).to_msg()
                msg.voltage = battery.pack_voltage_mv / 1000.0
                msg.current = battery.current_ma / 1000.0
                msg.temperature = float(battery.temperature_c)
                msg.percentage = battery.capacity_pct / 100.0
                msg.capacity = battery.remaining_capacity_mah / 1000.0
                msg.design_capacity = self._design_capacity_ah
                msg.cell_voltage = [mv / 1000.0 for mv in battery.cell_voltages_mv]
                msg.present = not battery.is_stale(now)
                msg.location = self._pack_names[can_id]

                self.pub_battery.publish(msg)

    def _publish_safety_stop(self):
        with self._lock:
            self.pub_safety_stop.publish(Bool(data=not self.manager.all_safe(time.monotonic())))

    def destroy_node(self):
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    signals.init(args=args)
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
