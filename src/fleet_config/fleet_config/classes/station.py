from __future__ import annotations

import json
import re
from typing import Any

import yaml

from fleet_config.common import CLOCK_BRIDGE, ROUTER, clock_bridge, foxglove_bridge, unit_agent
from fleet_config.model import SIMULATION
from fleet_config.paths import FOXGLOVE_BRIDGE_PARAMS_FILE
from fleet_config.unit import (
    GeneratedFile,
    UnitContext,
    description_topic,
    health_topic,
    mission_state_topic,
    position_topic,
    robot_exports,
)
from fleet_unit.manifest import Process

FOXGLOVE_PARAMETERS = {
    "port": "<foxglove_port>",
    "address": "<foxglove_address>",
    "num_threads": 2,
    "send_buffer_limit": 1000000,
    "max_qos_depth": 5,
    "capabilities": ["connectionGraph", "assets"],
}
MAP_ZOOM_LEVEL = 17
FLEET_SPLIT_PERCENTAGE = 50
RAW_MESSAGES = {
    "diffMethod": "custom",
    "diffTopicPath": "",
    "diffEnabled": False,
    "showFullMessageForDiff": False,
}


def processes(unit: UnitContext) -> list[Process]:
    assert unit.station is not None
    simulated = unit.mode == SIMULATION
    entries: list[Process] = []
    if simulated:
        entries.append(clock_bridge(unit))
    entries.append(unit_agent(unit, tuple(robot_id for robot_id, _ in unit.station.robots)))
    entries.append(foxglove_bridge(unit, [CLOCK_BRIDGE] if simulated else [ROUTER]))
    return entries


def foxglove_parameters(unit: UnitContext) -> str:
    assert unit.station is not None
    whitelist = [f"^{re.escape(health_topic(unit.id))}$"] + [
        f"^{re.escape(entry.topic)}$"
        for robot_id, robot_class in unit.station.robots
        for entry in robot_exports(robot_id, robot_class)
    ]
    parameters = {**FOXGLOVE_PARAMETERS, "topic_whitelist": whitelist}
    return yaml.safe_dump({"/**": {"ros__parameters": parameters}}, sort_keys=False)


def split(panels: list[Any], direction: str) -> Any:
    if len(panels) == 1:
        return panels[0]
    return {
        "first": panels[0],
        "second": split(panels[1:], direction),
        "direction": direction,
        "splitPercentage": 100 / len(panels),
    }


def layout(unit: UnitContext) -> GeneratedFile:
    assert unit.station is not None
    robots = unit.station.robots
    panels: dict[str, Any] = {
        "map!fleet": {
            "topics": {
                position_topic(robot_id, robot_class): {"visible": True}
                for robot_id, robot_class in robots
            },
            "zoomLevel": MAP_ZOOM_LEVEL,
        }
    }
    for unit_id in (unit.id, *(robot_id for robot_id, _ in robots)):
        panels[f"DiagnosticSummary!{unit_id}"] = {
            "topicToRender": health_topic(unit_id),
            "hardwareIdFilter": "",
            "pinnedIds": [],
            "sortByLevel": True,
        }
    for robot_id, _ in robots:
        panels[f"RawMessages!{robot_id}_mission"] = {
            "topicPath": mission_state_topic(robot_id),
            **RAW_MESSAGES,
        }
        panels[f"RawMessages!{robot_id}"] = {
            "topicPath": description_topic(robot_id),
            **RAW_MESSAGES,
        }
    rows = [f"DiagnosticSummary!{unit.id}"] + [
        split([f"{prefix}{robot_id}{suffix}" for robot_id, _ in robots], "row")
        for prefix, suffix in (
            ("DiagnosticSummary!", ""),
            ("RawMessages!", "_mission"),
            ("RawMessages!", ""),
        )
    ]
    panels["Tab!station"] = {
        "activeTabIdx": 0,
        "tabs": [
            {
                "title": "Fleet",
                "layout": {
                    "first": "map!fleet",
                    "second": split(rows, "column"),
                    "direction": "row",
                    "splitPercentage": FLEET_SPLIT_PERCENTAGE,
                },
            }
        ],
    }
    document = {
        "configById": panels,
        "layout": "Tab!station",
        "globalVariables": {},
        "playbackConfig": {"speed": 1},
        "userNodes": {},
    }
    return GeneratedFile(f"{unit.id} layout", json.dumps(document, indent=2) + "\n")


def unit_files(unit: UnitContext) -> dict[str, GeneratedFile]:
    return {
        FOXGLOVE_BRIDGE_PARAMS_FILE: GeneratedFile(
            f"{unit.id} {FOXGLOVE_BRIDGE_PARAMS_FILE}", foxglove_parameters(unit)
        )
    }
