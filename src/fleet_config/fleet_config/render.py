from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

import yaml

from fleet_config.checks import check_layout_topics
from fleet_config.classes import station as station_class
from fleet_config.classes import world as world_class
from fleet_config.compose import UNIT_STOP_GRACE_SECONDS, compose, image
from fleet_config.errors import RenderError
from fleet_config.model import (
    BROKER_PORT,
    SIMULATION,
    SITE_FILE,
    STATION,
    UNIT_PLACEHOLDER,
    WORLD,
    Fleet,
    Instance,
    Simulation,
    SourceFile,
)
from fleet_config.paths import (
    BRIDGE_CONFIG_FILE,
    COMPOSE_FILE,
    CONTAINER_RUN_DIR,
    CONTAINER_WORKSPACE_DIR,
    DESCRIPTION_FILE,
    FLEET_ROUTER_CONFIG_FILE,
    FLEET_SUMMARY_FILE,
    FOXGLOVE_BRIDGE_PARAMS_FILE,
    PARAMS_DIR,
    ROUTER_CONFIG_FILE,
    SESSION_CONFIG_FILE,
    layout_path,
)
from fleet_config.targets import TARGETS
from fleet_config.unit import GeneratedFile, UnitContext, contexts
from fleet_config.validation import check_references, compile_fleet
from fleet_config.zenoh import (
    LOOPBACK_ADDRESS,
    fleet_router,
    fleet_router_address,
    unit_router,
    unit_session,
)
from fleet_unit.manifest import (
    MANIFEST_NAME,
    Manifest,
    Process,
    Router,
    Timeouts,
    load_manifest,
    manifest_document,
)

PLACEHOLDER_PATTERN = re.compile(r"<[a-z][a-z_]*>")
PACKAGE_SHARE_PATTERN = re.compile(r"\$\(find-pkg-share ([A-Za-z0-9_]+)\)")
FOXGLOVE_PORT_PLACEHOLDER = "<foxglove_port>"
FOXGLOVE_ADDRESS_PLACEHOLDER = "<foxglove_address>"
ROUTER_PORT_PLACEHOLDER = "<router_port>"
VEHICLE_ID_PLACEHOLDER = "<vehicle_id>"
MADUM_CLIENT_ID_PLACEHOLDER = "<madum_client_id>"
BROKER_HOST_PLACEHOLDER = "<broker_host>"
BROKER_PORT_PLACEHOLDER = "<broker_port>"
BROKER_TLS_PLACEHOLDER = "<broker_tls>"

WORLD_PLACEHOLDERS = ("@WORLD_NAME@", "@DATUM_LAT@", "@DATUM_LON@", "@ELEVATION@")

TIMEOUTS = Timeouts(shutdown_grace_s=10.0, probe_interval_s=0.5)


def build_manifest(unit: UnitContext, processes: list[Process], unit_dir: Path) -> Manifest:
    return Manifest(
        unit=unit.id,
        kind=unit.kind,
        namespace=unit.namespace,
        domain_id=unit.domain_id,
        use_sim_time=unit.use_sim_time,
        run_dir=CONTAINER_RUN_DIR,
        router=Router(config=ROUTER_CONFIG_FILE, tcp_port=unit.router_tcp_port),
        session_config=SESSION_CONFIG_FILE,
        description=DESCRIPTION_FILE,
        timeouts=TIMEOUTS,
        processes=tuple(processes),
        unit_dir=unit_dir,
    )


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def write_yaml(path: Path, document: Any) -> None:
    write_text(
        path, yaml.safe_dump(document, sort_keys=False, default_flow_style=False, width=10**9)
    )


def write_json(path: Path, document: Any) -> None:
    write_text(path, json.dumps(document, indent=2) + "\n")


def substitutions(unit: UnitContext) -> dict[str, str]:
    values = {
        UNIT_PLACEHOLDER: unit.id,
        FOXGLOVE_PORT_PLACEHOLDER: str(unit.foxglove_port),
        ROUTER_PORT_PLACEHOLDER: str(unit.router_tcp_port),
    }
    robot = unit.robot
    if robot is not None and robot.robot_class.madum is not None:
        values[MADUM_CLIENT_ID_PLACEHOLDER] = str(robot.vehicle_id * 100)
        values[VEHICLE_ID_PLACEHOLDER] = str(robot.vehicle_id)
        values[BROKER_HOST_PLACEHOLDER] = broker_host(unit)
        values[BROKER_PORT_PLACEHOLDER] = str(BROKER_PORT)
        values[BROKER_TLS_PLACEHOLDER] = "false"
    if unit.mode == SIMULATION:
        values[FOXGLOVE_ADDRESS_PLACEHOLDER] = LOOPBACK_ADDRESS
    for key, value in unit.site.items():
        placeholder = f"<{key}>"
        if placeholder in values:
            raise RenderError(
                f"read site values of unit {unit.id} from {SITE_FILE}",
                f"{key} names a placeholder the renderer sets",
                unit=unit.id,
            )
        values[placeholder] = value
    return values


def broker_host(unit: UnitContext) -> str:
    if unit.on_fleet_host:
        return LOOPBACK_ADDRESS
    assert unit.topology is not None
    return unit.topology.uplink_address


def copy_with_substitutions(
    source: SourceFile | GeneratedFile, target: Path, unit: UnitContext
) -> None:
    if isinstance(source, GeneratedFile):
        origin, text = f"generated {source.name}", source.text
    else:
        origin, text = f"{source.path} of package {source.package}", source.resolve().read_text()
    for placeholder, value in substitutions(unit).items():
        text = text.replace(placeholder, value)
    text = PACKAGE_SHARE_PATTERN.sub(
        lambda match: f"{CONTAINER_WORKSPACE_DIR}/install/{match[1]}/share/{match[1]}", text
    )
    unresolved = PLACEHOLDER_PATTERN.search(text) or re.search(r"\$\([^)]*\)", text)
    if unresolved is not None:
        raise RenderError(
            f"copy {origin}",
            f"unresolved placeholder {unresolved.group()}",
            unit=unit.id,
        )
    write_text(target, text)


def world_sdf(simulation: Simulation) -> str:
    world = simulation.world
    template = world.template.resolve().read_text()
    missing = [placeholder for placeholder in WORLD_PLACEHOLDERS if placeholder not in template]
    if missing:
        raise RenderError(
            f"render world {world.name}",
            f"SDF template is missing placeholders: {', '.join(missing)}",
        )
    datum = simulation.datum
    rendered = (
        template.replace("@WORLD_NAME@", world.name)
        .replace("@DATUM_LAT@", str(datum.lat))
        .replace("@DATUM_LON@", str(datum.lon))
        .replace("@ELEVATION@", str(datum.elevation))
    )
    if "@" in rendered:
        raise RenderError(f"render world {world.name}", "SDF contains an unresolved placeholder")
    return rendered


def boundary_entries(entries: tuple[Any, ...]) -> list[dict[str, Any]]:
    return [
        {
            "topic": entry.topic,
            "type": entry.type,
            "max_hz": entry.max_hz if entry.max_hz is not None else 0.0,
        }
        for entry in entries
    ]


def description(unit: UnitContext) -> dict[str, Any]:
    robot = unit.robot
    return {
        "id": unit.id,
        "robot_class": unit.kind if robot is None else robot.robot_class.id,
        "mode": unit.mode,
        "ros_namespace": unit.namespace,
        "domain_id": unit.domain_id,
        "platform_kind": unit.kind if robot is None else robot.robot_class.platform_kind,
        "capabilities": [] if robot is None else list(robot.robot_class.capabilities),
        "exports": boundary_entries(unit.boundary.exports),
        "imports": boundary_entries(unit.boundary.imports),
    }


def fleet_summary(fleet: Fleet, units: list[UnitContext], instance: Instance) -> dict[str, Any]:
    return {
        "target": fleet.name,
        "mode": fleet.mode,
        "image": image(fleet.mode),
        "gui": fleet.simulation is not None,
        "instance": instance.index,
        "domain_id": instance.domain_id,
        "units": [{"id": unit.id, "kind": unit.kind, "service": unit.service} for unit in units],
    }


def unit_processes(unit: UnitContext) -> list[Process]:
    if unit.kind == WORLD:
        return world_class.processes(unit)
    if unit.kind == STATION:
        return station_class.processes(unit)
    return unit.resolved_robot().robot_class.processes(unit)


def unit_files(unit: UnitContext) -> dict[str, SourceFile | GeneratedFile]:
    if unit.kind == WORLD:
        return {}
    if unit.kind == STATION:
        return station_class.unit_files(unit)
    return dict(unit.resolved_robot().robot_class.unit_files(unit))


def unit_layout(unit: UnitContext) -> SourceFile | GeneratedFile | None:
    if unit.kind == WORLD:
        return None
    if unit.kind == STATION:
        return station_class.layout(unit)
    return unit.resolved_robot().robot_class.layout


def render_unit(unit: UnitContext, out_dir: Path) -> None:
    unit_dir = (out_dir / unit.path).resolve()
    unit_dir.mkdir(parents=True, exist_ok=True)
    manifest = build_manifest(unit, unit_processes(unit), unit_dir)
    if manifest.timeouts.worst_case_stop_s >= UNIT_STOP_GRACE_SECONDS:
        raise RenderError(
            "check unit stop time",
            f"worst-case supervisor stop time {manifest.timeouts.worst_case_stop_s}s is not below "
            f"the Compose stop grace period {UNIT_STOP_GRACE_SECONDS}s",
            unit=unit.id,
        )
    write_yaml(unit_dir / MANIFEST_NAME, manifest_document(manifest))
    write_json(unit_dir / ROUTER_CONFIG_FILE, unit_router(unit))
    write_json(unit_dir / SESSION_CONFIG_FILE, unit_session(unit))
    write_json(unit_dir / DESCRIPTION_FILE, description(unit))
    files = unit_files(unit)
    for name, source in files.items():
        copy_with_substitutions(source, unit_dir / PARAMS_DIR / name, unit)
    robot = unit.robot
    if robot is not None and unit.mode == SIMULATION and robot.robot_class.bridge is not None:
        copy_with_substitutions(robot.robot_class.bridge, unit_dir / BRIDGE_CONFIG_FILE, unit)
    layout = unit_layout(unit)
    if layout is not None:
        copy_with_substitutions(layout, out_dir / layout_path(unit.id), unit)
        if FOXGLOVE_BRIDGE_PARAMS_FILE in files:
            check_layout_topics(
                out_dir / layout_path(unit.id), unit_dir / PARAMS_DIR / FOXGLOVE_BRIDGE_PARAMS_FILE
            )
    loaded = load_manifest(unit_dir)
    if loaded != manifest:
        raise RenderError(
            "verify manifest",
            f"{unit_dir / MANIFEST_NAME} read back differs from the manifest that was built",
            unit=unit.id,
        )
    check_references(loaded, unit.dir)


def render_target(name: str, instance: Instance, out_dir: Path) -> None:
    target = TARGETS.get(name)
    if target is None:
        raise RenderError(
            "resolve target", f"unknown target {name}; known targets: {', '.join(TARGETS)}"
        )
    render_fleet(target.fleet(name), instance, out_dir)


def render_fleet(fleet: Fleet, instance: Instance, out_dir: Path) -> None:
    robots = compile_fleet(fleet)
    checked: set[str] = set()
    for robot in robots:
        robot_class = robot.robot_class
        if robot_class.check is not None and robot_class.id not in checked:
            robot_class.check()
            checked.add(robot_class.id)
    units = contexts(fleet, robots, instance)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_yaml(out_dir / FLEET_SUMMARY_FILE, fleet_summary(fleet, units, instance))
    write_yaml(out_dir / COMPOSE_FILE, compose(fleet, units, instance))
    if any(unit.on_fleet_host for unit in units):
        write_json(
            out_dir / FLEET_ROUTER_CONFIG_FILE, fleet_router(instance, fleet_router_address(fleet))
        )
    if fleet.simulation is not None:
        write_text(out_dir / fleet.simulation.world.sdf_file, world_sdf(fleet.simulation))
    for unit in units:
        render_unit(unit, out_dir)
