from __future__ import annotations

import re
from collections.abc import Iterator, Mapping
from functools import cache
from pathlib import Path

from fleet_config.errors import RenderError
from fleet_config.model import PHYSICAL, Fleet, ResolvedRobot
from fleet_config.paths import SOURCE_ROOT
from fleet_unit.manifest import Manifest, Process

INSTALLED_EXECUTABLES = re.compile(
    r"install\s*\(\s*(?:PROGRAMS|TARGETS)\s+([^)]*?)\s+DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\)"
)
CONSOLE_SCRIPT = re.compile(r"['\"]\s*([A-Za-z0-9_.-]+)\s*=\s*[A-Za-z0-9_.]+:[A-Za-z0-9_]+\s*['\"]")


def fleet_errors(fleet: Fleet) -> list[str]:
    errors: list[str] = []
    simulated = fleet.simulation is not None
    if not fleet.groups and (fleet.station is None or simulated):
        errors.append(f"target {fleet.name} must contain at least one robot group")
    for name, group in fleet.groups.items():
        if group.id_override is not None and group.count != 1:
            errors.append(
                f"group {name}: id_override set but count != 1 (got {group.count}); the override would be dropped"
            )
        if simulated and group.spawn_poses is None:
            errors.append(f"group {name}: spawn_poses must be set for simulation targets")
        if simulated and group.spawn_poses is not None and len(group.spawn_poses) != group.count:
            errors.append(
                f"group {name}: spawn_poses length={len(group.spawn_poses)} must match count={group.count}"
            )
        if fleet.features.madum_enabled and group.robot_class.madum is None:
            errors.append(
                f"group {name}: MADUM is enabled but robot class {group.robot_class.id} has no MADUM profile"
            )
    if fleet.mode == PHYSICAL:
        for group in fleet.groups.values():
            if group.id_override != fleet.name:
                errors.append(
                    f"physical target key {fleet.name} must match robot id {group.id_override}"
                )
    seen_ids: dict[str, list[str]] = {}
    seen_vehicles: dict[int, list[str]] = {}
    for name, group in fleet.groups.items():
        madum = group.robot_class.madum
        for index in range(1, group.count + 1):
            seen_ids.setdefault(group.robot_id(index), []).append(name)
            if madum is not None:
                seen_vehicles.setdefault(madum.vehicle_id_base + index, []).append(name)
    for robot_id, groups in seen_ids.items():
        if len(groups) > 1:
            errors.append(f"robot id {robot_id} collides across groups: {', '.join(groups)}")
        if "__" in robot_id:
            errors.append(f"robot id {robot_id} must not contain repeated underscores")
    for vehicle_id, groups in seen_vehicles.items():
        if len(groups) > 1:
            errors.append(
                f"MADUM vehicle_id {vehicle_id} collides across groups: {', '.join(groups)}"
            )
    classes = [group.robot_class.id for group in fleet.groups.values()]
    for class_id in sorted(set(classes)):
        if classes.count(class_id) > 1:
            errors.append(f"robot class {class_id} is used by more than one group")
    return errors


def compile_fleet(fleet: Fleet) -> tuple[ResolvedRobot, ...]:
    errors = fleet_errors(fleet)
    if errors:
        raise RenderError("validate fleet", "; ".join(errors), target=fleet.name)
    return tuple(
        ResolvedRobot(
            id=group.robot_id(index),
            robot_class=group.robot_class,
            mode=fleet.mode,
            use_sim_time=fleet.use_sim_time,
            topology=fleet.topology,
            simulation=fleet.simulation,
            spawn_pose=group.spawn_poses[index - 1] if group.spawn_poses is not None else None,
            features=fleet.features,
            index=index,
        )
        for group in fleet.groups.values()
        for index in range(1, group.count + 1)
    )


@cache
def first_party_executables() -> Mapping[str, frozenset[str]]:
    executables: dict[str, frozenset[str]] = {}
    for manifest in sorted(SOURCE_ROOT.glob("*/package.xml")):
        package = manifest.parent
        names: set[str] = set()
        cmake = package / "CMakeLists.txt"
        if cmake.is_file():
            for listed in INSTALLED_EXECUTABLES.findall(cmake.read_text()):
                names.update(Path(item).name for item in listed.split())
        setup = package / "setup.py"
        if setup.is_file():
            names.update(CONSOLE_SCRIPT.findall(setup.read_text()))
        executables[package.name] = frozenset(names)
    return executables


def _unit_files(process: Process) -> Iterator[str]:
    yield from process.params
    for component in process.load:
        yield from component.params
    for staged in process.files:
        yield staged.src


def _container_paths(process: Process, container_dir: str) -> Iterator[str]:
    prefix = f"{container_dir}/"
    values = [*process.args, *process.cmd, *process.param_values.values()]
    for value in values:
        if isinstance(value, str) and value.startswith(prefix):
            yield value[len(prefix) :]


def _package_files(process: Process) -> Iterator[tuple[str, str]]:
    descriptions = [process.robot_description]
    if process.spawn is not None:
        descriptions.append(process.spawn.robot_description)
        if process.spawn.model is not None and process.spawn.model.package is not None:
            yield process.spawn.model.package, process.spawn.model.path
    for description in descriptions:
        if description is not None:
            yield description.package, description.path


def check_references(manifest: Manifest, container_dir: str) -> None:
    executables = first_party_executables()
    for process in manifest.processes:
        operation = f"check references of process {process.name}"
        installed = executables.get(process.package or "")
        if (
            installed is not None
            and process.executable is not None
            and process.executable not in installed
        ):
            raise RenderError(
                operation,
                f"package {process.package} installs no executable {process.executable}; "
                f"it installs {', '.join(sorted(installed))}",
                unit=manifest.unit,
            )
        for relative in [*_unit_files(process), *_container_paths(process, container_dir)]:
            if not (manifest.unit_dir / relative).is_file():
                raise RenderError(
                    operation,
                    f"{manifest.unit_dir / relative} was not rendered",
                    unit=manifest.unit,
                )
        for package, path in _package_files(process):
            if package in executables and not (SOURCE_ROOT / package / path).is_file():
                raise RenderError(
                    operation, f"package {package} has no file {path}", unit=manifest.unit
                )
