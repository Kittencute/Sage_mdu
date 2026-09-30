from __future__ import annotations

import logging
import os
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from fleet_interfaces.msg import BoundaryEntry, UnitDescription
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from fleet_common import signals
from fleet_common.params import declare
from fleet_unit import state as unit_state
from fleet_unit.description import Description, load_description
from fleet_unit.errors import UnitError
from fleet_unit.manifest import Manifest, load_manifest

UNIT_DIR_VARIABLE = "SAGE_UNIT_DIR"
IMAGE_DIGEST_VARIABLE = "SAGE_IMAGE_DIGEST"
DESCRIPTION_TOPIC = "unit/description"
HEALTH_TOPIC = "unit/health"
STATION_KIND = "station"
LOG_FORMAT = "%(asctime)s %(levelname)s %(name)s %(message)s"

OK_STATES = frozenset({unit_state.READY})
WARN_STATES = frozenset(
    {
        unit_state.PENDING,
        unit_state.WAITING,
        unit_state.STARTING,
        unit_state.STOPPED,
    }
)


def _publisher_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _boundary_messages(entries: tuple[Any, ...]) -> list[BoundaryEntry]:
    messages: list[BoundaryEntry] = []
    for entry in entries:
        message = BoundaryEntry()
        message.topic = entry.topic
        message.type = entry.type
        message.max_hz = float(entry.max_hz)
        messages.append(message)
    return messages


def _process_level(process: dict[str, Any]) -> bytes:
    name = process["state"]
    if name in OK_STATES:
        return DiagnosticStatus.OK
    if name in WARN_STATES:
        return DiagnosticStatus.WARN
    return DiagnosticStatus.ERROR


def _with_detail(state: str, detail: str) -> str:
    return f"{state}: {detail}" if detail else state


class UnitAgent(Node):
    def __init__(self, manifest: Manifest, description: Description) -> None:
        super().__init__("unit_agent")
        self._manifest = manifest
        self._description = description
        self._state_file = manifest.state_file
        self._image_digest = os.environ.get(IMAGE_DIGEST_VARIABLE, "")
        health_period = declare(
            self,
            "health_period_s",
            Parameter.Type.DOUBLE,
            "Interval between unit health publications, in seconds",
            0.01,
            3600.0,
        )
        description_period = declare(
            self,
            "description_period_s",
            Parameter.Type.DOUBLE,
            "Interval between unit description publications, in seconds",
            0.01,
            3600.0,
        )
        self._description_publisher = self.create_publisher(
            UnitDescription, DESCRIPTION_TOPIC, _publisher_qos()
        )
        self._health_publisher = self.create_publisher(
            DiagnosticArray, HEALTH_TOPIC, _publisher_qos()
        )
        self._description_message = self._build_description()
        self._publish_description()
        self._description_timer = self.create_timer(description_period, self._publish_description)
        self._health_received_at: dict[str, float | None] = {}
        self._health_subscriptions: list[Any] = []
        if manifest.kind == STATION_KIND:
            self._health_max_age = declare(
                self,
                "health_max_age_s",
                Parameter.Type.DOUBLE,
                "Longest time since a watched unit's last health message before it is reported stale, in seconds",
                0.01,
                3600.0,
            )
            watched_units = declare(
                self,
                "watched_units",
                Parameter.Type.STRING_ARRAY,
                "Units whose health this station watches",
                constraints="at least one unit id",
            )
            if not watched_units:
                raise UnitError(
                    "read parameter watched_units",
                    "expected at least one unit",
                    self._manifest.unit,
                )
            for unit in watched_units:
                self._watch_health(unit)
        self._health_timer = self.create_timer(health_period, self._publish_health)

    def _watch_health(self, unit: str) -> None:
        self._health_received_at[unit] = None

        def received(_: DiagnosticArray) -> None:
            self._health_received_at[unit] = time.monotonic()

        self._health_subscriptions.append(
            self.create_subscription(
                DiagnosticArray, f"/{unit}/{HEALTH_TOPIC}", received, _publisher_qos()
            )
        )

    def _watched_health(self, unit: str, received_at: float | None) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f"{self._manifest.unit}: {unit} health"
        status.hardware_id = unit
        if received_at is None:
            status.level = DiagnosticStatus.STALE
            status.message = "stale: no health received"
            return status
        age = time.monotonic() - received_at
        status.values = [KeyValue(key="age_s", value=f"{age:.1f}")]
        if age > self._health_max_age:
            status.level = DiagnosticStatus.STALE
            status.message = f"stale: last health {age:.1f} s ago"
        else:
            status.level = DiagnosticStatus.OK
            status.message = f"last health {age:.1f} s ago"
        return status

    def _publish_description(self) -> None:
        self._description_publisher.publish(self._description_message)

    def _build_description(self) -> UnitDescription:
        message = UnitDescription()
        message.id = self._description.id
        message.robot_class = self._description.robot_class
        message.mode = self._description.mode
        message.ros_namespace = self._description.ros_namespace
        message.domain_id = self._description.domain_id
        message.platform_kind = self._description.platform_kind
        message.capabilities = list(self._description.capabilities)
        message.exports = _boundary_messages(self._description.exports)
        message.imports = _boundary_messages(self._description.imports)
        message.image_digest = self._image_digest
        return message

    def _publish_health(self) -> None:
        document = unit_state.read_state(self._state_file)
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        unit = document["unit"]
        for process in document["processes"]:
            status = DiagnosticStatus()
            status.name = f"{unit}: {process['name']}"
            status.hardware_id = unit
            status.level = _process_level(process)
            status.message = _with_detail(process["state"], process["detail"])
            status.values = [
                KeyValue(key="kind", value=str(process["kind"])),
                KeyValue(key="ready", value=str(process["ready"])),
                KeyValue(key="pid", value=str(process["pid"])),
                KeyValue(key="exit_code", value=str(process["exit_code"])),
                KeyValue(key="signal", value=str(process["signal"])),
                KeyValue(key="detail", value=str(process["detail"])),
            ]
            array.status.append(status)
        summary = DiagnosticStatus()
        summary.name = f"{unit}: unit"
        summary.hardware_id = unit
        summary.message = _with_detail(document["state"], document["detail"])
        summary.level = max((status.level for status in array.status), default=DiagnosticStatus.OK)
        summary.values = [
            KeyValue(key="image_digest", value=self._image_digest),
            KeyValue(key="runner_pid", value=str(document["runner_pid"])),
            KeyValue(key="updated_at", value=repr(document["updated_at"])),
        ]
        array.status.insert(0, summary)
        array.status.extend(
            self._watched_health(unit, received_at)
            for unit, received_at in self._health_received_at.items()
        )
        self._health_publisher.publish(array)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(stream=sys.stderr, level=logging.INFO, format=LOG_FORMAT)
    try:
        unit_dir = os.environ.get(UNIT_DIR_VARIABLE)
        if not unit_dir:
            raise UnitError(f"read {UNIT_DIR_VARIABLE}", "the variable is unset")
        manifest = load_manifest(Path(unit_dir))
        description = load_description(manifest.description_path, manifest.unit)
    except UnitError as exc:
        logging.getLogger("fleet_unit.unit_agent").error("%s", exc)
        return 1
    signals.init(args=argv if argv is not None else sys.argv)
    node: UnitAgent | None = None
    try:
        node = UnitAgent(manifest, description)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (UnitError, ParameterException, RuntimeError, ValueError) as exc:
        logging.getLogger("fleet_unit.unit_agent").error("%s", exc)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
