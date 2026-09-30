from __future__ import annotations

import re
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import TYPE_CHECKING

import yaml

from fleet_config.errors import RenderError
from fleet_config.paths import SOURCE_ROOT

if TYPE_CHECKING:
    from fleet_config.unit import UnitContext
    from fleet_unit.manifest import Process

PORT_STRIDE = 100
FLEET_ROUTER_TCP_PORT_BASE = 7447
UNIT_ROUTER_TCP_PORT_BASE = 7460
FOXGLOVE_BRIDGE_PORT_BASE = 8765

UNIT_PLACEHOLDER = "<robot_namespace>"

SIMULATION = "simulation"
PHYSICAL = "physical"

ROBOT = "robot"
WORLD = "world"
STATION = "station"

WORLD_UNIT_ID = "world"
STATION_UNIT_ID = "station"

BROKER_USER_VARIABLE = "MADUM_BROKER_USER"
BROKER_PASS_VARIABLE = "MADUM_BROKER_PASS"
BROKER_PORT = 1883

SITE_FILE = SOURCE_ROOT.parent / "site.yaml"
SITE_KEY_PATTERN = re.compile(r"[a-z][a-z_]*")
UPLINK_ADDRESS_KEY = "uplink_address"

Pose = tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class Instance:
    index: int

    def __post_init__(self) -> None:
        if self.index < 1:
            raise RenderError(
                "read instance index", f"expected a positive integer, got {self.index}"
            )

    @property
    def offset(self) -> int:
        return (self.index - 1) * PORT_STRIDE

    @property
    def domain_id(self) -> int:
        return self.index - 1

    @property
    def fleet_router_tcp_port(self) -> int:
        return FLEET_ROUTER_TCP_PORT_BASE + self.offset

    @property
    def gz_partition(self) -> str:
        return f"sage_{self.index}"

    def unit_router_tcp_port(self, unit_index: int) -> int:
        return UNIT_ROUTER_TCP_PORT_BASE + self.offset + unit_index

    def foxglove_port(self, unit_index: int) -> int:
        return FOXGLOVE_BRIDGE_PORT_BASE + self.offset + unit_index


def parse_instance(raw: str) -> Instance:
    if not raw.isdigit():
        raise RenderError("read SAGE_INSTANCE", f"expected a positive integer, got {raw!r}")
    return Instance(int(raw))


@dataclass(frozen=True)
class SourceFile:
    package: str
    path: str

    def resolve(self) -> Path:
        candidate = SOURCE_ROOT / self.package / self.path
        if not candidate.is_file():
            raise RenderError(
                f"read {self.path} of package {self.package}",
                f"{candidate} is not a file",
            )
        return candidate


@dataclass(frozen=True)
class Datum:
    lat: float
    lon: float
    elevation: float


@dataclass(frozen=True)
class World:
    name: str
    template: SourceFile

    @property
    def sdf_file(self) -> str:
        return f"worlds/{self.name}.sdf"


@dataclass(frozen=True)
class Simulation:
    world: World
    datum: Datum
    seed: int
    gpu: bool


@dataclass(frozen=True)
class Features:
    recording_enabled: bool = False
    madum_enabled: bool = False


@dataclass(frozen=True)
class Topology:
    uplink_address: str
    uplink_interface: str


@dataclass(frozen=True)
class MadumProfile:
    vehicle_id_base: int
    vehicle_type_code: int
    max_speed: float
    equipments_json: str
    alarm_codes: tuple[int, ...]
    odometry_topic: str
    battery_topic: str
    max_flight_time_minutes: int | None = None


@dataclass(frozen=True)
class RobotClass:
    id: str
    platform_kind: str
    capabilities: tuple[str, ...]
    position_topic: str
    position_type: str
    processes: Callable[[UnitContext], list[Process]]
    unit_files: Callable[[UnitContext], Mapping[str, SourceFile]]
    layout: SourceFile
    madum: MadumProfile | None = None
    bridge: SourceFile | None = None
    gz_models_dir: str | None = None
    description_package: str | None = None
    check: Callable[[], None] | None = None


@dataclass(frozen=True)
class Station:
    id: str = STATION_UNIT_ID
    robots: tuple[tuple[str, RobotClass], ...] = ()


@dataclass(frozen=True)
class RobotGroup:
    robot_class: RobotClass
    count: int
    spawn_poses: tuple[Pose, ...] | None = None
    id_override: str | None = None

    def __post_init__(self) -> None:
        if self.count < 1:
            raise RenderError("declare robot group", f"count must be positive, got {self.count}")
        for index, pose in enumerate(self.spawn_poses or ()):
            if len(pose) != 6:
                raise RenderError(
                    "declare robot group",
                    f"spawn_poses[{index}] must have six numbers, got {len(pose)}",
                )

    def robot_id(self, index: int) -> str:
        if self.id_override is not None and self.count == 1:
            return self.id_override
        return f"{self.robot_class.id}{index}"


@dataclass(frozen=True)
class Fleet:
    name: str
    mode: str
    simulation: Simulation | None
    features: Features
    topology: Topology | None
    groups: Mapping[str, RobotGroup]
    station: Station | None
    site: Mapping[str, str]

    @property
    def use_sim_time(self) -> bool:
        return self.mode == SIMULATION


@dataclass(frozen=True)
class SimTarget:
    world: World
    datum: Datum
    groups: Mapping[str, RobotGroup]
    seed: int
    gpu: bool = False
    features: Features = Features()

    def fleet(self, name: str) -> Fleet:
        return Fleet(
            name=name,
            mode=SIMULATION,
            simulation=Simulation(self.world, self.datum, self.seed, self.gpu),
            features=self.features,
            topology=None,
            groups=MappingProxyType(dict(self.groups)),
            station=Station(),
            site=MappingProxyType({}),
        )


@dataclass(frozen=True)
class PhysicalTarget:
    robot_class: RobotClass
    id: str
    features: Features = Features()

    def fleet(self, name: str) -> Fleet:
        site = read_site(self.id)
        station_site = read_site(STATION_UNIT_ID)
        return Fleet(
            name=name,
            mode=PHYSICAL,
            simulation=None,
            features=self.features,
            topology=Topology(
                site_value(station_site, STATION_UNIT_ID, UPLINK_ADDRESS_KEY),
                site_value(site, self.id, "uplink_interface"),
            ),
            groups=MappingProxyType({"self": RobotGroup(self.robot_class, 1, id_override=self.id)}),
            station=None,
            site=site,
        )


@dataclass(frozen=True)
class StationTarget:
    robots: tuple[tuple[str, RobotClass], ...]
    features: Features = Features()

    def fleet(self, name: str) -> Fleet:
        station = Station(robots=self.robots)
        return Fleet(
            name=name,
            mode=PHYSICAL,
            simulation=None,
            features=self.features,
            topology=None,
            groups=MappingProxyType({}),
            station=station,
            site=read_site(station.id),
        )


Target = SimTarget | PhysicalTarget | StationTarget


def read_site(unit_id: str) -> Mapping[str, str]:
    operation = f"read the site values of unit {unit_id} from {SITE_FILE}"
    try:
        document = yaml.safe_load(SITE_FILE.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise RenderError(operation, exc) from exc
    if not isinstance(document, dict) or unit_id not in document:
        raise RenderError(operation, f"the file has no top-level entry {unit_id}")
    values = document[unit_id]
    if not isinstance(values, dict) or not all(
        isinstance(key, str) and SITE_KEY_PATTERN.fullmatch(key) and isinstance(value, str)
        for key, value in values.items()
    ):
        raise RenderError(
            operation, f"expected a mapping of lowercase names to strings, got {values!r}"
        )
    return MappingProxyType(values)


def site_value(site: Mapping[str, str], unit_id: str, key: str) -> str:
    if key not in site:
        raise RenderError(
            f"read {key} of unit {unit_id} from {SITE_FILE}",
            f"the entry {unit_id} has no key {key}",
        )
    return site[key]


@dataclass(frozen=True)
class ResolvedRobot:
    id: str
    robot_class: RobotClass
    mode: str
    use_sim_time: bool
    topology: Topology | None
    simulation: Simulation | None
    spawn_pose: Pose | None
    features: Features
    index: int

    @property
    def vehicle_id(self) -> int:
        madum = self.robot_class.madum
        assert madum is not None
        return madum.vehicle_id_base + self.index
