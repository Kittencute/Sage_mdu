from __future__ import annotations

import types
import typing
from collections.abc import Mapping
from dataclasses import MISSING, Field, dataclass, fields, is_dataclass
from graphlib import CycleError, TopologicalSorter
from pathlib import Path
from types import MappingProxyType
from typing import Any, get_args, get_origin, get_type_hints

import yaml

from fleet_unit.errors import UnitError

MANIFEST_NAME = "manifest.yaml"
ROUTER_NAME = "router"

EMPTY: Mapping[str, Any] = MappingProxyType({})


@dataclass(frozen=True)
class NodeProbe:
    node: str


@dataclass(frozen=True)
class TopicProbe:
    topic: str


@dataclass(frozen=True)
class MessageProbe:
    message: str


@dataclass(frozen=True)
class TcpProbe:
    host: str
    port: int


@dataclass(frozen=True)
class ExitProbe:
    exit: int


Probe = NodeProbe | TopicProbe | MessageProbe | TcpProbe | ExitProbe
PROBE_TAGS: Mapping[str, type[Any]] = MappingProxyType(
    {
        "node": NodeProbe,
        "topic": TopicProbe,
        "message": MessageProbe,
        "port": TcpProbe,
        "exit": ExitProbe,
    }
)


@dataclass(frozen=True)
class Timeouts:
    shutdown_grace_s: float
    probe_interval_s: float

    @property
    def worst_case_stop_s(self) -> float:
        return 2 * self.shutdown_grace_s


@dataclass(frozen=True)
class Router:
    config: str
    tcp_port: int


@dataclass(frozen=True)
class RobotDescription:
    package: str
    path: str
    arguments: Mapping[str, str] = EMPTY


@dataclass(frozen=True)
class PackageFile:
    package: str | None
    path: str


@dataclass(frozen=True)
class SpawnSpec:
    entity_name: str
    pose: tuple[float, float, float, float, float, float]
    model: PackageFile | None = None
    world_name: str | None = None
    robot_description: RobotDescription | None = None


@dataclass(frozen=True)
class WaitFor:
    path: str


@dataclass(frozen=True)
class StagedFile:
    src: str
    dst: str


@dataclass(frozen=True)
class ComponentSpec:
    package: str
    plugin: str
    node_name: str
    namespace: str
    params: tuple[str, ...] = ()
    param_values: Mapping[str, Any] = EMPTY
    param_env: Mapping[str, str] = EMPTY
    remaps: Mapping[str, str] = EMPTY


@dataclass(frozen=True)
class Process:
    name: str
    kind: str
    package: str | None = None
    executable: str | None = None
    node_name: str | None = None
    namespace: str | None = None
    params: tuple[str, ...] = ()
    param_values: Mapping[str, Any] = EMPTY
    param_env: Mapping[str, str] = EMPTY
    remaps: Mapping[str, str] = EMPTY
    args: tuple[str, ...] = ()
    env: Mapping[str, str] = EMPTY
    robot_description: RobotDescription | None = None
    container: str | None = None
    load: tuple[ComponentSpec, ...] = ()
    cmd: tuple[str, ...] = ()
    cwd: str | None = None
    files: tuple[StagedFile, ...] = ()
    spawn: SpawnSpec | None = None
    wait_for: WaitFor | None = None
    after: tuple[str, ...] = ()
    ready: Probe | None = None
    output: str = "screen"


@dataclass(frozen=True)
class Manifest:
    unit: str
    kind: str
    namespace: str
    domain_id: int
    use_sim_time: bool
    run_dir: str
    router: Router
    session_config: str
    description: str
    timeouts: Timeouts
    processes: tuple[Process, ...]
    unit_dir: Path

    @property
    def state_file(self) -> Path:
        return Path(self.run_dir) / self.unit / "state.json"

    @property
    def description_path(self) -> Path:
        return self.unit_dir / self.description

    @property
    def router_config(self) -> Path:
        return self.unit_dir / self.router.config

    @property
    def session_config_path(self) -> Path:
        return self.unit_dir / self.session_config

    def process(self, name: str) -> Process:
        for entry in self.processes:
            if entry.name == name:
                return entry
        raise UnitError("resolve process", f"{name} is not declared", self.unit)


def _required(item: Field[Any]) -> bool:
    return item.default is MISSING and item.default_factory is MISSING


def _is_default(item: Field[Any], value: Any) -> bool:
    if item.default is not MISSING:
        return value == item.default
    if item.default_factory is not MISSING:
        return value == item.default_factory()
    return False


def _convert(hint: Any, value: Any, key: str) -> Any:
    origin = get_origin(hint)
    if hint is Any:
        return _convert_any(value, key)
    if origin in (typing.Union, types.UnionType):
        return _convert_union(hint, value, key)
    if is_dataclass(hint):
        return _convert_dataclass(hint, value, key)
    if origin is tuple:
        return _convert_tuple(hint, value, key)
    if origin is Mapping:
        return _convert_mapping(hint, value, key)
    return _convert_scalar(hint, value, key)


def _convert_any(value: Any, key: str) -> Any:
    if isinstance(value, list):
        return tuple(_convert(Any, item, f"{key}[{index}]") for index, item in enumerate(value))
    return value


def _convert_union(hint: Any, value: Any, key: str) -> Any:
    options = tuple(option for option in get_args(hint) if option is not type(None))
    if value is None and len(options) < len(get_args(hint)):
        return None
    if len(options) == 1:
        return _convert(options[0], value, key)
    assert set(options) == set(PROBE_TAGS.values()), f"{key}: only Probe may be a union"
    tags = [name for name in PROBE_TAGS if isinstance(value, dict) and name in value]
    if len(tags) != 1:
        raise ValueError(
            f"{key}: {value!r} is not a probe with exactly one of {', '.join(PROBE_TAGS)}"
        )
    return _convert(PROBE_TAGS[tags[0]], value, key)


def _convert_dataclass(hint: Any, value: Any, key: str) -> Any:
    label = key or "top level"
    if not isinstance(value, dict):
        raise ValueError(f"{label}: expected a mapping, got {type(value).__name__}")
    known = {item.name: item for item in fields(hint)}
    unknown = sorted(str(name) for name in value.keys() - known.keys())
    if unknown:
        raise ValueError(f"{label}: unknown keys {', '.join(unknown)}")
    missing = sorted(name for name, item in known.items() if _required(item) and name not in value)
    if missing:
        raise ValueError(f"{label}: missing keys {', '.join(missing)}")
    hints = get_type_hints(hint)
    return hint(
        **{
            name: _convert(hints[name], item, f"{key}.{name}" if key else name)
            for name, item in value.items()
        }
    )


def _convert_tuple(hint: Any, value: Any, key: str) -> Any:
    if not isinstance(value, list):
        raise ValueError(f"{key}: expected a list, got {type(value).__name__}")
    arguments = get_args(hint)
    if arguments[1:] == (Ellipsis,):
        arguments = (arguments[0],) * len(value)
    if len(arguments) != len(value):
        raise ValueError(f"{key}: expected {len(arguments)} items, got {len(value)}")
    return tuple(
        _convert(argument, item, f"{key}[{index}]")
        for index, (argument, item) in enumerate(zip(arguments, value, strict=True))
    )


def _convert_mapping(hint: Any, value: Any, key: str) -> Any:
    if not isinstance(value, dict) or not all(isinstance(name, str) for name in value):
        raise ValueError(f"{key}: expected a mapping with string keys")
    _, item_hint = get_args(hint)
    return MappingProxyType(
        {name: _convert(item_hint, item, f"{key}.{name}") for name, item in value.items()}
    )


def _convert_scalar(hint: Any, value: Any, key: str) -> Any:
    if hint is float and isinstance(value, int) and not isinstance(value, bool):
        return float(value)
    if not isinstance(value, hint) or (hint is not bool and isinstance(value, bool)):
        raise ValueError(f"{key}: expected {hint.__name__}, got {type(value).__name__}")
    return value


def from_document(cls: type[Any], document: Any, operation: str, unit: str | None = None) -> Any:
    try:
        return _convert(cls, document, "")
    except ValueError as exc:
        raise UnitError(operation, exc, unit) from exc


def to_document(value: Any) -> Any:
    if is_dataclass(value):
        return {
            item.name: to_document(getattr(value, item.name))
            for item in fields(value)
            if not _is_default(item, getattr(value, item.name))
        }
    if isinstance(value, Mapping):
        return {name: to_document(item) for name, item in value.items()}
    if isinstance(value, tuple):
        return [to_document(item) for item in value]
    return value


def manifest_document(manifest: Manifest) -> dict[str, Any]:
    document = to_document(manifest)
    del document["unit_dir"]
    return document


def _check(manifest: Manifest, operation: str) -> None:
    names = [entry.name for entry in manifest.processes] + [ROUTER_NAME]
    by_name = {entry.name: entry for entry in manifest.processes}
    for entry in manifest.processes:
        if names.count(entry.name) > 1:
            raise UnitError(operation, "the process name is not unique", manifest.unit, entry.name)
        unknown = sorted(set(entry.after) - set(names))
        if unknown:
            raise UnitError(
                operation,
                f"after references undeclared {', '.join(unknown)}",
                manifest.unit,
                entry.name,
            )
        if entry.wait_for is not None and not entry.wait_for.path.startswith("/"):
            raise UnitError(
                operation,
                f"wait_for path {entry.wait_for.path} is not absolute",
                manifest.unit,
                entry.name,
            )
        if entry.kind == "components":
            container = by_name.get(entry.container or "")
            if (
                container is None
                or container.kind != "container"
                or container.name not in entry.after
            ):
                raise UnitError(
                    operation,
                    f"container {entry.container} must be a container process listed in after",
                    manifest.unit,
                    entry.name,
                )
    try:
        TopologicalSorter({entry.name: entry.after for entry in manifest.processes}).prepare()
    except CycleError as exc:
        raise UnitError(
            operation, f"after forms a cycle {' -> '.join(exc.args[1])}", manifest.unit
        ) from exc


def load_manifest(unit_dir: Path) -> Manifest:
    try:
        resolved = unit_dir.resolve(strict=True)
    except OSError as exc:
        raise UnitError(f"resolve unit directory {unit_dir}", exc) from exc
    path = resolved / MANIFEST_NAME
    operation = f"load manifest {path}"
    try:
        document = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise UnitError(operation, exc) from exc
    if not isinstance(document, dict):
        raise UnitError(operation, "the document is not a mapping")
    manifest = from_document(Manifest, {**document, "unit_dir": resolved}, operation)
    _check(manifest, operation)
    return manifest
