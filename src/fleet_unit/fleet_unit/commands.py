from __future__ import annotations

import json
import os
import shutil
from pathlib import Path
from typing import Any

import xacro
import yaml

from fleet_unit.errors import UnitError
from fleet_unit.manifest import ComponentSpec, Manifest, Process, RobotDescription, SpawnSpec
from fleet_unit.packages import package_executable, package_file, package_share

ROUTER_EXECUTABLE = "zenohd"
SPAWN_PACKAGE = "ros_gz_sim"
SPAWN_EXECUTABLE = "create"
RMW_IMPLEMENTATION = "rmw_zenoh_cpp"


def format_parameter_value(value: Any) -> str:
    if isinstance(value, tuple):
        return json.dumps(list(value))
    return json.dumps(value)


def environment_value(manifest: Manifest, process: str, variable: str) -> str:
    value = os.environ.get(variable)
    if value is None:
        raise UnitError(
            f"read environment variable {variable}",
            "the variable is unset",
            manifest.unit,
            process,
        )
    return value


def ros_environment(manifest: Manifest) -> dict[str, str]:
    return {
        "ROS_DOMAIN_ID": str(manifest.domain_id),
        "RMW_IMPLEMENTATION": RMW_IMPLEMENTATION,
        "ZENOH_SESSION_CONFIG_URI": str(manifest.session_config_path),
    }


def process_environment(manifest: Manifest, process: Process) -> dict[str, str]:
    environment = dict(os.environ)
    environment.update(ros_environment(manifest))
    environment.update(process.env)
    return environment


def router_argv(manifest: Manifest) -> list[str]:
    executable = shutil.which(ROUTER_EXECUTABLE)
    if executable is None:
        raise UnitError(
            f"resolve {ROUTER_EXECUTABLE}",
            f"{ROUTER_EXECUTABLE} is not on PATH",
            manifest.unit,
        )
    return [executable, "-c", str(manifest.router_config)]


def prepare_cwd(manifest: Manifest, process: Process) -> str | None:
    if process.cwd is None:
        return None
    directory = Path(process.cwd)
    try:
        directory.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise UnitError(
            f"create working directory {directory}", exc, manifest.unit, process.name
        ) from exc
    return process.cwd


def robot_description_xml(
    manifest: Manifest, process: Process, description: RobotDescription
) -> str:
    path = package_share(description.package) / description.path
    try:
        document = xacro.process_file(str(path), mappings=dict(description.arguments))
    except xacro.XacroException as exc:
        raise UnitError(
            f"process robot description {path}", exc, manifest.unit, process.name
        ) from exc
    return document.toxml()


def node_argv(manifest: Manifest, process: Process) -> list[str]:
    if process.package is None or process.executable is None:
        raise UnitError(
            "build node command line",
            f"kind {process.kind} requires package and executable",
            manifest.unit,
            process.name,
        )
    argv = [str(package_executable(process.package, process.executable))]
    argv.extend(process.args)
    argv.append("--ros-args")
    namespace = process.namespace or ""
    if namespace:
        argv.extend(["-r", f"__ns:=/{namespace}"])
    if process.node_name:
        argv.extend(["-r", f"__node:={process.node_name}"])
    argv.extend(["-p", f"use_sim_time:={format_parameter_value(manifest.use_sim_time)}"])
    for relative in process.params:
        argv.extend(["--params-file", str(manifest.unit_dir / relative)])
    for name, value in process.param_values.items():
        argv.extend(["-p", f"{name}:={format_parameter_value(value)}"])
    for name, variable in process.param_env.items():
        argv.extend(
            [
                "-p",
                f"{name}:={format_parameter_value(environment_value(manifest, process.name, variable))}",
            ]
        )
    if process.robot_description is not None:
        content = robot_description_xml(manifest, process, process.robot_description)
        argv.extend(["-p", f"robot_description:={format_parameter_value(content)}"])
    for source, target in process.remaps.items():
        argv.extend(["-r", f"{source}:={target}"])
    return argv


def stage_files(manifest: Manifest, process: Process) -> None:
    for staged in process.files:
        source = manifest.unit_dir / staged.src
        destination = Path(staged.dst)
        try:
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
        except OSError as exc:
            raise UnitError(
                f"stage file {source} to {destination}", exc, manifest.unit, process.name
            ) from exc


def spawn_model(manifest: Manifest, process: Process, spawn: SpawnSpec) -> str:
    if spawn.robot_description is not None:
        return robot_description_xml(manifest, process, spawn.robot_description)
    if spawn.model is None:
        raise UnitError(
            "read spawn model",
            "spawn requires model or robot_description",
            manifest.unit,
            process.name,
        )
    model_path = package_file(spawn.model.package, spawn.model.path)
    try:
        return model_path.read_text()
    except OSError as exc:
        raise UnitError(f"read spawn model {model_path}", exc, manifest.unit, process.name) from exc


def spawn_argv(manifest: Manifest, process: Process) -> list[str]:
    spawn = process.spawn
    if spawn is None:
        raise UnitError(
            "build spawn command line",
            "kind spawn requires spawn",
            manifest.unit,
            process.name,
        )
    model = spawn_model(manifest, process, spawn)
    x, y, z, roll, pitch, yaw = spawn.pose
    argv = [
        str(package_executable(SPAWN_PACKAGE, SPAWN_EXECUTABLE)),
        "-name",
        spawn.entity_name,
    ]
    if spawn.world_name is not None:
        argv.extend(["-world", spawn.world_name])
    argv.extend(
        [
            "-string",
            model,
            "-x",
            repr(x),
            "-y",
            repr(y),
            "-z",
            repr(z),
            "-R",
            repr(roll),
            "-P",
            repr(pitch),
            "-Y",
            repr(yaw),
        ]
    )
    return argv


def process_argv(manifest: Manifest, process: Process) -> list[str]:
    if process.kind in ("node", "container"):
        return node_argv(manifest, process)
    if process.kind == "exec":
        return list(process.cmd)
    if process.kind == "spawn":
        return spawn_argv(manifest, process)
    raise UnitError(
        "build process command line",
        f"kind {process.kind} does not start a child process",
        manifest.unit,
        process.name,
    )


def container_fqn(manifest: Manifest, container: Process) -> str:
    namespace = (container.namespace or "").strip("/")
    name = container.node_name
    if not name:
        raise UnitError(
            "resolve container node name",
            "container does not declare node_name",
            manifest.unit,
            container.name,
        )
    if namespace:
        return f"/{namespace}/{name}"
    return f"/{name}"


def _flatten(prefix: str, body: dict[str, Any], into: dict[str, Any]) -> None:
    for name, value in body.items():
        key = f"{prefix}.{name}" if prefix else name
        if isinstance(value, dict):
            _flatten(key, value, into)
        else:
            into[key] = value


def component_parameters(
    manifest: Manifest, process: Process, component: ComponentSpec
) -> dict[str, Any]:
    resolved: dict[str, Any] = {"use_sim_time": manifest.use_sim_time}
    sections = [*component.namespace.strip("/").split("/"), component.node_name]
    for relative in component.params:
        path = manifest.unit_dir / relative
        try:
            section = yaml.safe_load(path.read_text())
        except (OSError, yaml.YAMLError) as exc:
            raise UnitError(
                f"load parameter file {path}", exc, manifest.unit, process.name
            ) from exc
        for name in [*filter(None, sections), "ros__parameters"]:
            if not isinstance(section, dict) or name not in section:
                raise UnitError(
                    f"load parameter file {path}",
                    f"the file has no section {name} for node {component.namespace}/{component.node_name}",
                    manifest.unit,
                    process.name,
                )
            section = section[name]
        if not isinstance(section, dict):
            raise UnitError(
                f"load parameter file {path}",
                f"ros__parameters of node {component.namespace}/{component.node_name} is not a mapping",
                manifest.unit,
                process.name,
            )
        _flatten("", section, resolved)
    for name, value in component.param_values.items():
        resolved[name] = list(value) if isinstance(value, tuple) else value
    for name, variable in component.param_env.items():
        resolved[name] = environment_value(manifest, process.name, variable)
    return resolved


def log_path(manifest: Manifest, process: Process) -> Path:
    log_dir = os.environ.get("ROS_LOG_DIR")
    if not log_dir:
        raise UnitError(
            "resolve ROS_LOG_DIR",
            "output log requires ROS_LOG_DIR; it is unset",
            manifest.unit,
            process.name,
        )
    return Path(log_dir) / manifest.unit / f"{process.name}.log"
