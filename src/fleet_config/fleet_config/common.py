from __future__ import annotations

from collections.abc import Iterable, Mapping
from types import MappingProxyType
from typing import Any

from fleet_config.model import BROKER_PASS_VARIABLE, BROKER_USER_VARIABLE, Pose, SourceFile
from fleet_config.paths import (
    BRIDGE_CONFIG_FILE,
    CONTAINER_RECORDINGS_DIR,
    FOXGLOVE_BRIDGE_PARAMS_FILE,
    params_path,
)
from fleet_config.unit import CLOCK_TOPIC, CLOCK_TYPE, UnitContext
from fleet_unit.manifest import (
    EMPTY,
    ROUTER_NAME,
    ComponentSpec,
    ExitProbe,
    PackageFile,
    Probe,
    Process,
    RobotDescription,
    SpawnSpec,
    StagedFile,
    TopicProbe,
    WaitFor,
)

ROUTER = ROUTER_NAME
WORLD_CHECK_MODULE = "fleet_unit.world_check"
WORLD_READY = "world_ready"
ENTITY_WATCH = "entity_watch"

RUNNER_PACKAGE = "fleet_unit"
AGENT_HEALTH_PERIOD_S = 1.0
AGENT_DESCRIPTION_PERIOD_S = 5.0
AGENT_HEALTH_MAX_AGE_S = 5 * AGENT_HEALTH_PERIOD_S

GZ_PARTITION_VARIABLE = "GZ_PARTITION"
ROS_GZ_BRIDGE_PACKAGE = "ros_gz_bridge"
PARAMETER_BRIDGE_EXECUTABLE = "parameter_bridge"
GZ_CLOCK_TYPE = "gz.msgs.Clock"

CLOCK_BRIDGE = "clock_bridge"
UNIT_AGENT = "unit_agent"
FOXGLOVE_BRIDGE = "foxglove_bridge"
FOXGLOVE_PARAMS = params_path(FOXGLOVE_BRIDGE_PARAMS_FILE)

TF_REMAPS: Mapping[str, str] = MappingProxyType({"/tf": "tf", "/tf_static": "tf_static"})


def frozen(mapping: Mapping[str, Any] | None) -> Mapping[str, Any]:
    if not mapping:
        return EMPTY
    return MappingProxyType(dict(mapping))


def _param_values(mapping: Mapping[str, Any] | None) -> Mapping[str, Any]:
    if not mapping:
        return EMPTY
    return MappingProxyType(
        {
            name: tuple(value) if isinstance(value, list) else value
            for name, value in mapping.items()
        }
    )


def merged(*mappings: Mapping[str, str] | None) -> dict[str, str]:
    result: dict[str, str] = {}
    for mapping in mappings:
        if mapping:
            result.update(mapping)
    return result


def gz_environment(unit: UnitContext) -> dict[str, str]:
    return {GZ_PARTITION_VARIABLE: unit.instance.gz_partition}


def _common(
    name: str,
    kind: str,
    after: Iterable[str],
    ready: Probe | None,
    output: str,
    env: Mapping[str, str] | None,
    wait_for: WaitFor | None,
    files: Iterable[StagedFile],
) -> dict[str, Any]:
    return {
        "name": name,
        "kind": kind,
        "after": tuple(after),
        "ready": ready,
        "output": output,
        "env": frozen(env),
        "wait_for": wait_for,
        "files": tuple(files),
    }


def node(
    name: str,
    *,
    package: str,
    executable: str,
    namespace: str,
    after: Iterable[str],
    node_name: str | None = None,
    params: Iterable[str] = (),
    param_values: Mapping[str, Any] | None = None,
    param_env: Mapping[str, str] | None = None,
    remaps: Mapping[str, str] | None = None,
    args: Iterable[str] = (),
    robot_description: RobotDescription | None = None,
    kind: str = "node",
    ready: Probe | None = None,
    output: str = "screen",
    env: Mapping[str, str] | None = None,
    wait_for: WaitFor | None = None,
) -> Process:
    return Process(
        package=package,
        executable=executable,
        node_name=node_name,
        namespace=namespace,
        params=tuple(params),
        param_values=_param_values(param_values),
        param_env=frozen(param_env),
        remaps=frozen(remaps),
        args=tuple(args),
        robot_description=robot_description,
        **_common(name, kind, after, ready, output, env, wait_for, ()),
    )


def component(
    *,
    package: str,
    plugin: str,
    node_name: str,
    namespace: str,
    params: Iterable[str] = (),
    param_values: Mapping[str, Any] | None = None,
    param_env: Mapping[str, str] | None = None,
    remaps: Mapping[str, str] | None = None,
) -> ComponentSpec:
    return ComponentSpec(
        package=package,
        plugin=plugin,
        node_name=node_name,
        namespace=namespace,
        params=tuple(params),
        param_values=_param_values(param_values),
        param_env=frozen(param_env),
        remaps=frozen(remaps),
    )


def components(
    name: str,
    *,
    container: str,
    load: Iterable[ComponentSpec],
    after: Iterable[str],
    output: str = "screen",
) -> Process:
    return Process(
        container=container,
        load=tuple(load),
        **_common(name, "components", after, None, output, None, None, ()),
    )


def exec_process(
    name: str,
    *,
    cmd: Iterable[str],
    after: Iterable[str],
    cwd: str | None = None,
    env: Mapping[str, str] | None = None,
    files: Iterable[StagedFile] = (),
    ready: Probe | None = None,
    output: str = "screen",
) -> Process:
    return Process(
        cmd=tuple(cmd),
        cwd=cwd,
        **_common(name, "exec", after, ready, output, env, None, files),
    )


def world_check_command(mode: str, world: str, *options: str) -> list[str]:
    return ["python3", "-m", WORLD_CHECK_MODULE, mode, "--world", world, *options]


def world_ready(unit: UnitContext, env: Mapping[str, str], after: Iterable[str]) -> Process:
    assert unit.simulation is not None
    return exec_process(
        WORLD_READY,
        cmd=world_check_command("ready", unit.simulation.world.name),
        env=env,
        after=after,
        ready=ExitProbe(0),
    )


def entity_watch(unit: UnitContext, after: Iterable[str]) -> Process:
    assert unit.simulation is not None
    return exec_process(
        ENTITY_WATCH,
        cmd=world_check_command(
            "watch", unit.simulation.world.name, "--entity", unit.resolved_robot().id
        ),
        env=gz_environment(unit),
        after=after,
    )


def spawn_process(
    name: str,
    *,
    model: PackageFile | None = None,
    robot_description: RobotDescription | None = None,
    entity_name: str,
    pose: Pose,
    world_name: str | None,
    after: Iterable[str],
    env: Mapping[str, str] | None = None,
    ready: Probe | None = None,
) -> Process:
    return Process(
        spawn=SpawnSpec(
            model=model,
            entity_name=entity_name,
            pose=tuple(float(value) for value in pose),
            world_name=world_name,
            robot_description=robot_description,
        ),
        **_common(name, "spawn", after, ready, "screen", env, None, ()),
    )


def unit_agent(unit: UnitContext, watched_units: tuple[str, ...] = ()) -> Process:
    watch = (
        {"watched_units": watched_units, "health_max_age_s": AGENT_HEALTH_MAX_AGE_S}
        if watched_units
        else {}
    )
    return node(
        UNIT_AGENT,
        package=RUNNER_PACKAGE,
        executable=UNIT_AGENT,
        node_name=UNIT_AGENT,
        namespace=unit.namespace,
        param_values={
            "use_sim_time": False,
            "health_period_s": AGENT_HEALTH_PERIOD_S,
            "description_period_s": AGENT_DESCRIPTION_PERIOD_S,
            **watch,
        },
        after=(ROUTER,),
        ready=TopicProbe(f"/{unit.namespace}/unit/description"),
    )


def clock_bridge(unit: UnitContext) -> Process:
    return node(
        CLOCK_BRIDGE,
        package=ROS_GZ_BRIDGE_PACKAGE,
        executable=PARAMETER_BRIDGE_EXECUTABLE,
        node_name=f"{unit.id}_{CLOCK_BRIDGE}",
        namespace="",
        args=(f"{CLOCK_TOPIC}@{CLOCK_TYPE}[{GZ_CLOCK_TYPE}",),
        env=gz_environment(unit),
        after=(ROUTER,),
        ready=TopicProbe(CLOCK_TOPIC),
    )


def gz_bridge(unit: UnitContext, after: Iterable[str], ready: Probe) -> Process:
    return node(
        "gz_bridge",
        package=ROS_GZ_BRIDGE_PACKAGE,
        executable=PARAMETER_BRIDGE_EXECUTABLE,
        node_name=f"{unit.id}_gz_bridge",
        namespace="",
        param_values={"config_file": unit.file(BRIDGE_CONFIG_FILE)},
        env=gz_environment(unit),
        after=after,
        ready=ready,
    )


def foxglove_bridge(
    unit: UnitContext,
    after: Iterable[str],
    *,
    remaps: Mapping[str, str] | None = None,
) -> Process:
    return node(
        FOXGLOVE_BRIDGE,
        package="foxglove_bridge",
        executable="foxglove_bridge",
        node_name=FOXGLOVE_BRIDGE,
        namespace=unit.namespace,
        params=(FOXGLOVE_PARAMS,),
        remaps=remaps,
        after=after,
    )


def twist_stamper(
    unit: UnitContext, after: Iterable[str], *, frame_id: str, in_topic: str, out_topic: str
) -> Process:
    return node(
        "twist_stamper",
        package="terrascout_navigation",
        executable="twist_stamper.py",
        node_name="twist_stamper",
        namespace=unit.namespace,
        param_values={"frame_id": frame_id},
        remaps={"cmd_vel_in": in_topic, "cmd_vel_out": out_topic},
        after=after,
        output="log",
    )


MQTT_CLIENT = "mqtt_client"
MADUM_BRIDGE = "madum_bridge"
RECORDER = "recorder"
MADUM_STATE_VECTOR_INTERVAL_S = 1.0
MADUM_BATTERY_LOW_THRESHOLD = 20.0
MADUM_SENSOR_TIMEOUT_S = 5.0
MADUM_BATTERY_TIMEOUT_S = 30.0
RECORDING_STORAGE = "mcap"
RECORDING_MAX_BAG_DURATION_S = 60
MQTT_CLIENT_FILE = "mqtt_client.yaml"
MCAP_WRITER_FILE = "mcap_writer.yaml"
QOS_OVERRIDES_FILE = "qos_overrides.yaml"


def madum_state_file(unit: UnitContext) -> str:
    robot = unit.resolved_robot()
    return f"{unit.run_dir}/madum_last_completed_mission_{robot.vehicle_id}.json"


def madum_processes(unit: UnitContext) -> list[Process]:
    robot = unit.resolved_robot()
    madum = robot.robot_class.madum
    if not robot.features.madum_enabled or madum is None:
        return []
    remaps = {
        "position": robot.robot_class.position_topic,
        "odometry": madum.odometry_topic,
        "battery_state": madum.battery_topic,
    }
    return [
        node(
            MQTT_CLIENT,
            package="mqtt_client",
            executable="mqtt_client",
            node_name=MQTT_CLIENT,
            namespace=unit.namespace,
            params=(params_path(MQTT_CLIENT_FILE),),
            param_env={"broker.user": BROKER_USER_VARIABLE, "broker.pass": BROKER_PASS_VARIABLE},
            after=(ROUTER,),
        ),
        node(
            MADUM_BRIDGE,
            package="madum_bridge",
            executable="bridge.py",
            node_name=MADUM_BRIDGE,
            namespace=unit.namespace,
            param_values={
                "vehicle_id": robot.vehicle_id,
                "vehicle_name": robot.id,
                "completed_mission_state_file": madum_state_file(unit),
                "vehicle_type": madum.vehicle_type_code,
                "vehicle_max_speed": madum.max_speed,
                "state_vector_interval": MADUM_STATE_VECTOR_INTERVAL_S,
                "battery_low_threshold": MADUM_BATTERY_LOW_THRESHOLD,
                "sensor_timeout": MADUM_SENSOR_TIMEOUT_S,
                "battery_timeout": MADUM_BATTERY_TIMEOUT_S,
                "equipments_json": madum.equipments_json,
                "allowed_alarm_codes": list(madum.alarm_codes),
                "max_flight_time_minutes": madum.max_flight_time_minutes or 0,
            },
            remaps=remaps,
            after=(ROUTER,),
        ),
    ]


def recorder_process(unit: UnitContext, topics: Iterable[str]) -> list[Process]:
    robot = unit.resolved_robot()
    if not robot.features.recording_enabled:
        return []
    resolved = sorted(unit.scoped(topic) for topic in topics)
    cmd = [
        "ros2",
        "bag",
        "record",
        "-s",
        RECORDING_STORAGE,
        "--storage-config-file",
        unit.file(params_path(MCAP_WRITER_FILE)),
        "--qos-profile-overrides-path",
        unit.file(params_path(QOS_OVERRIDES_FILE)),
        "--max-bag-duration",
        str(RECORDING_MAX_BAG_DURATION_S),
    ]
    if unit.use_sim_time:
        cmd.append("--use-sim-time")
    cmd += ["--topics", *resolved]
    return [
        exec_process(
            RECORDER,
            cmd=cmd,
            cwd=f"{CONTAINER_RECORDINGS_DIR}/{unit.id}",
            after=(ROUTER,),
        )
    ]


def feature_files(unit: UnitContext, qos_overrides: SourceFile) -> dict[str, SourceFile]:
    robot = unit.resolved_robot()
    files: dict[str, SourceFile] = {}
    if robot.features.madum_enabled:
        files[MQTT_CLIENT_FILE] = SourceFile("madum_bridge", f"config/{MQTT_CLIENT_FILE}")
    if robot.features.recording_enabled:
        files[MCAP_WRITER_FILE] = SourceFile("fleet_unit", f"config/{MCAP_WRITER_FILE}")
        files[QOS_OVERRIDES_FILE] = qos_overrides
    return files
