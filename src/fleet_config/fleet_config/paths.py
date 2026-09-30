from __future__ import annotations

from pathlib import Path

CONTAINER_RENDER_DIR = "/sage/render"
CONTAINER_RUN_DIR = "/run/sage"
CONTAINER_WORKSPACE_DIR = "/workspace"
CONTAINER_LOG_DIR = f"{CONTAINER_RUN_DIR}/log"
CONTAINER_RECORDINGS_DIR = f"{CONTAINER_RUN_DIR}/recordings"
FLEET_SOCKET_PATH = f"{CONTAINER_RUN_DIR}/fleet.sock"

FLEET_SUMMARY_FILE = "fleet.yaml"
COMPOSE_FILE = "compose.yaml"
FLEET_ROUTER_CONFIG_FILE = "zenoh/fleet-router.json5"
LAYOUTS_DIR = "layouts"
UNITS_DIR = "units"
PARAMS_DIR = "params"

DESCRIPTION_FILE = "description.json"
ROUTER_CONFIG_FILE = "zenoh-router.json5"
SESSION_CONFIG_FILE = "zenoh-session.json5"
BRIDGE_CONFIG_FILE = "bridge.yaml"
FOXGLOVE_BRIDGE_PARAMS_FILE = "foxglove_bridge.yaml"

SOURCE_ROOT = Path(__file__).resolve().parents[2]


def unit_path(unit_id: str) -> str:
    return f"{UNITS_DIR}/{unit_id}"


def unit_dir_of(unit_id: str) -> str:
    return f"{CONTAINER_RENDER_DIR}/{unit_path(unit_id)}"


def unit_run_dir_of(unit_id: str) -> str:
    return f"{CONTAINER_RUN_DIR}/{unit_id}"


def params_path(name: str) -> str:
    return f"{PARAMS_DIR}/{name}"


def layout_path(unit_id: str) -> str:
    return f"{LAYOUTS_DIR}/{unit_id}.json"
