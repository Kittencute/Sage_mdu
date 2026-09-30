from __future__ import annotations

from pathlib import Path
from typing import Any

from fleet_config.errors import RenderError
from fleet_config.model import (
    BROKER_PASS_VARIABLE,
    BROKER_USER_VARIABLE,
    ROBOT,
    SIMULATION,
    WORLD,
    WORLD_UNIT_ID,
    Fleet,
    Instance,
    site_value,
)
from fleet_config.paths import (
    CONTAINER_LOG_DIR,
    CONTAINER_RENDER_DIR,
    CONTAINER_RUN_DIR,
    CONTAINER_WORKSPACE_DIR,
    FLEET_ROUTER_CONFIG_FILE,
    SESSION_CONFIG_FILE,
)
from fleet_config.unit import UnitContext

RENDER_HOST_VARIABLE = "${SAGE_RENDER_HOST}"
RUN_HOST_VARIABLE = "${SAGE_RUN_HOST}"
IMAGE_DIGEST_VARIABLE = "${SAGE_IMAGE_DIGEST}"
USER_VARIABLE = "${DOCKER_UID:-1000}:${DOCKER_GID:-1000}"
DISPLAY_VARIABLE = "${DISPLAY:-:0}"

FLEET_ROUTER_SERVICE = "fleet-router"
DEV_SERVICE = "dev"
MOSQUITTO_SERVICE = "mosquitto"

ZENOHD_EXECUTABLE = "zenohd"
RUNNER_COMMAND = ("python3", "-m", "fleet_unit")

SIMULATION_IMAGE = "sage-sim:latest"
PHYSICAL_IMAGE = "sage-phy:latest"
DEV_IMAGE = "sage-dev:latest"
MOSQUITTO_IMAGE = "eclipse-mosquitto:2"
LOOPBACK_BROKER_CONFIG = "../docker/mosquitto.conf"
AUTHENTICATED_BROKER_CONFIG = "../docker/mosquitto-authenticated.conf"
BROKER_CONFIG_TARGET = "/mosquitto/config/mosquitto.conf"
BROKER_PASSWORD_FILE_TARGET = "/mosquitto/config/password_file"
BROKER_PASSWORD_FILE_KEY = "mosquitto_password_file"

JOYSTICK_DEVICE_MOUNT = "/dev/input:/dev/input"
JOYSTICK_DEVICE_RULE = "c 13:* rmw"
DEVICE_PATH_PREFIX = "/dev/"
X11_SOCKET_MOUNT = "/tmp/.X11-unix:/tmp/.X11-unix:rw"
GPU_DEVICE_MOUNT = "/dev/dri:/dev/dri"

WORKSPACE_MOUNT_NAMES = ("src", "build", "install")

RESTART_POLICY = "no"
UNIT_STOP_GRACE_SECONDS = 60
DEV_UHLC_MAX_DELTA_MS = "5000"

GPU_GROUPS = (
    "${SAGE_VIDEO_GID:?the host has no group video, which the GPU world needs}",
    "${SAGE_RENDER_GID:?the host has no group render, which the GPU world needs}",
)
DEVICE_GROUPS = ("input", "dialout")


def image(mode: str) -> str:
    return SIMULATION_IMAGE if mode == SIMULATION else PHYSICAL_IMAGE


def _logging() -> dict[str, Any]:
    return {"driver": "json-file", "options": {"max-size": "100m", "max-file": "5"}}


def _render_volumes(mode: str) -> list[str]:
    volumes = [
        f"{RENDER_HOST_VARIABLE}:{CONTAINER_RENDER_DIR}:ro",
        f"{RUN_HOST_VARIABLE}:{CONTAINER_RUN_DIR}",
    ]
    if mode == SIMULATION:
        volumes += [
            f"../{name}:{CONTAINER_WORKSPACE_DIR}/{name}:rw" for name in WORKSPACE_MOUNT_NAMES
        ]
    return volumes


def _site_devices(unit: UnitContext) -> list[str]:
    return sorted(
        f"{path}:{path}" for path in unit.site.values() if path.startswith(DEVICE_PATH_PREFIX)
    )


def _broker_credential(variable: str, required: bool) -> str:
    if required:
        return f"${{{variable}:?set {variable} in docker/.env}}"
    return f"${{{variable}:-}}"


def _unit_environment(unit: UnitContext) -> dict[str, str]:
    environment = {
        "SAGE_UNIT_DIR": unit.dir,
        "SAGE_IMAGE_DIGEST": IMAGE_DIGEST_VARIABLE,
        "ROS_DOMAIN_ID": str(unit.domain_id),
        "ROS_LOG_DIR": CONTAINER_LOG_DIR,
        "ZENOH_SESSION_CONFIG_URI": unit.file(SESSION_CONFIG_FILE),
    }
    robot = unit.robot
    if robot is not None and robot.features.madum_enabled:
        for variable in (BROKER_USER_VARIABLE, BROKER_PASS_VARIABLE):
            environment[variable] = _broker_credential(variable, not unit.on_fleet_host)
    return environment


def _display_environment(gpu: bool) -> dict[str, str]:
    environment = {
        "DISPLAY": DISPLAY_VARIABLE,
        "QT_QPA_PLATFORM": "xcb",
        "QT_X11_NO_MITSHM": "1",
    }
    if gpu:
        environment.update(
            {
                "__NV_PRIME_RENDER_OFFLOAD": "1",
                "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
                "NVIDIA_VISIBLE_DEVICES": "all",
                "NVIDIA_DRIVER_CAPABILITIES": "all",
            }
        )
    return environment


def _gpu_settings() -> dict[str, Any]:
    return {
        "devices": [GPU_DEVICE_MOUNT],
        "group_add": list(GPU_GROUPS),
        "runtime": "nvidia",
        "deploy": {
            "resources": {
                "reservations": {
                    "devices": [{"driver": "nvidia", "count": "all", "capabilities": ["gpu"]}]
                }
            }
        },
    }


def _unit_service(unit: UnitContext, dependencies: list[str]) -> dict[str, Any]:
    world = unit.kind == WORLD
    hardware = unit.kind == ROBOT and unit.mode != SIMULATION
    environment = _unit_environment(unit)
    volumes = _render_volumes(unit.mode)
    if world:
        assert unit.simulation is not None
        environment.update(_display_environment(unit.simulation.gpu))
        volumes.append(X11_SOCKET_MOUNT)
    if hardware:
        volumes.append(JOYSTICK_DEVICE_MOUNT)
    service: dict[str, Any] = {
        "image": image(unit.mode),
        "network_mode": "host",
        "init": True,
        "user": USER_VARIABLE,
        "command": [*RUNNER_COMMAND, "run", "--unit-dir", unit.dir],
        "environment": environment,
        "volumes": volumes,
    }
    if hardware:
        service["devices"] = _site_devices(unit)
        service["device_cgroup_rules"] = [JOYSTICK_DEVICE_RULE]
        service["group_add"] = list(DEVICE_GROUPS)
    if world and unit.simulation.gpu:
        service.update(_gpu_settings())
    if dependencies:
        service["depends_on"] = {name: {"condition": "service_started"} for name in dependencies}
    service["stop_grace_period"] = f"{UNIT_STOP_GRACE_SECONDS}s"
    service["restart"] = RESTART_POLICY
    service["logging"] = _logging()
    return service


def _router_service(mode: str) -> dict[str, Any]:
    return {
        "image": image(mode),
        "network_mode": "host",
        "user": USER_VARIABLE,
        "command": [ZENOHD_EXECUTABLE, "-c", f"{CONTAINER_RENDER_DIR}/{FLEET_ROUTER_CONFIG_FILE}"],
        "volumes": _render_volumes(mode),
        "init": True,
        "restart": RESTART_POLICY,
        "logging": _logging(),
    }


def _development_service(units: list[UnitContext], instance: Instance, mode: str) -> dict[str, Any]:
    session = next((unit for unit in units if unit.kind == ROBOT), units[0])
    return {
        "image": DEV_IMAGE if mode == SIMULATION else PHYSICAL_IMAGE,
        "network_mode": "host",
        "user": USER_VARIABLE,
        "stdin_open": True,
        "tty": True,
        "environment": {
            "ROS_DOMAIN_ID": str(instance.domain_id),
            "ROS_LOG_DIR": CONTAINER_LOG_DIR,
            "ZENOH_SESSION_CONFIG_URI": session.file(SESSION_CONFIG_FILE),
            "UHLC_MAX_DELTA_MS": DEV_UHLC_MAX_DELTA_MS,
        },
        "volumes": _render_volumes(mode) + [JOYSTICK_DEVICE_MOUNT],
        "device_cgroup_rules": [JOYSTICK_DEVICE_RULE],
        "group_add": ["input"],
        "logging": _logging(),
        "profiles": [DEV_SERVICE],
    }


def _broker_volumes(fleet: Fleet) -> list[str]:
    if fleet.mode == SIMULATION:
        return [f"{LOOPBACK_BROKER_CONFIG}:{BROKER_CONFIG_TARGET}:ro"]
    assert fleet.station is not None
    password_file = site_value(fleet.site, fleet.station.id, BROKER_PASSWORD_FILE_KEY)
    if not Path(password_file).is_file():
        raise RenderError(
            f"read {BROKER_PASSWORD_FILE_KEY} of unit {fleet.station.id}",
            f"{password_file} is not a file",
        )
    return [
        f"{AUTHENTICATED_BROKER_CONFIG}:{BROKER_CONFIG_TARGET}:ro",
        f"{password_file}:{BROKER_PASSWORD_FILE_TARGET}:ro",
    ]


def _mosquitto_service(fleet: Fleet) -> dict[str, Any]:
    return {
        "image": MOSQUITTO_IMAGE,
        "network_mode": "host",
        "volumes": _broker_volumes(fleet),
        "logging": _logging(),
        "restart": RESTART_POLICY,
    }


def compose(fleet: Fleet, units: list[UnitContext], instance: Instance) -> dict[str, Any]:
    fleet_host = any(unit.on_fleet_host for unit in units)
    has_world = any(unit.kind == WORLD for unit in units)
    services: dict[str, Any] = {}
    if fleet_host:
        services[FLEET_ROUTER_SERVICE] = _router_service(fleet.mode)
    for unit in units:
        dependencies = [FLEET_ROUTER_SERVICE] if fleet_host else []
        if has_world and unit.kind != WORLD:
            dependencies.append(WORLD_UNIT_ID)
        services[unit.service] = _unit_service(unit, dependencies)
    services[DEV_SERVICE] = _development_service(units, instance, fleet.mode)
    if fleet_host and fleet.features.madum_enabled:
        services[MOSQUITTO_SERVICE] = _mosquitto_service(fleet)
    return {"services": services}
