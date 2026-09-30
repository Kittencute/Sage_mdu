from __future__ import annotations

from fleet_config.common import (
    CLOCK_BRIDGE,
    ROUTER,
    TF_REMAPS,
    WORLD_READY,
    clock_bridge,
    entity_watch,
    exec_process,
    feature_files,
    foxglove_bridge,
    gz_environment,
    madum_processes,
    node,
    recorder_process,
    spawn_process,
    unit_agent,
    world_ready,
)
from fleet_config.errors import RenderError
from fleet_config.model import SIMULATION, MadumProfile, RobotClass, SourceFile
from fleet_config.paths import FOXGLOVE_BRIDGE_PARAMS_FILE, params_path
from fleet_config.unit import UnitContext
from fleet_unit.manifest import ExitProbe, MessageProbe, PackageFile, Process, StagedFile

ID = "aeroscout"
MISSION_PACKAGE = "aeroscout_mission"

POSITION_TOPIC = "navsat"
POSITION_TYPE = "sensor_msgs/msg/NavSatFix"
CAPABILITIES = ("px4",)
MAX_HORIZONTAL_SPEED_M_S = 5.0

MADUM = MadumProfile(
    vehicle_id_base=3000,
    vehicle_type_code=1,
    max_speed=MAX_HORIZONTAL_SPEED_M_S,
    equipments_json=(
        '[{"id": 1, "name": "Gimbal Video", "type": 2, "status": 1}, '
        '{"id": 2, "name": "Gimbal Photo", "type": 1, "status": 1}, '
        '{"id": 3, "name": "Follow Target", "type": 5, "status": 1}]'
    ),
    alarm_codes=(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 99),
    odometry_topic="odom",
    battery_topic="battery_state",
    max_flight_time_minutes=20,
)

RECORDED_TOPICS = (
    "/rosout",
    "fmu/in/offboard_control_mode",
    "fmu/in/trajectory_setpoint",
    "fmu/in/vehicle_command",
    "battery_state",
    "fmu/out/battery_status",
    "fmu/out/vehicle_local_position",
    "fmu/out/vehicle_odometry",
    "fmu/out/vehicle_status",
    "mission_path",
    "mission_progress",
    "navsat",
    "odom",
    "tf",
    "tf_static",
    "waypoints_geo",
)
RECORDED_TOPICS_SIMULATION = ("/clock",)

PX4_DIR = "px4"
PX4_ZENOH_DIR = "zenoh"
PX4_PUB_FILE = "pub.csv"
PX4_SUB_FILE = "sub.csv"
PX4_NET_FILE = "net.txt"
PX4_AIRFRAME_ID = 4001
PX4_SIM_MODEL = "gz_x500"
PX4_BINARY = "/opt/px4/build/px4_sitl_zenoh/bin/px4"
PX4_DATA_DIR = "/opt/px4/build/px4_sitl_zenoh/etc"
PX4_INSTANCES_PER_TARGET = 10
PX4_MAX_INSTANCE = 249
PX4_GZ_MODELS_DIR = "/opt/px4/Tools/simulation/gz/models"
PX4_GLOBAL_POSITION_TOPIC = "fmu/out/vehicle_global_position"
PX4_ZENOH_FILES = (PX4_PUB_FILE, PX4_SUB_FILE, PX4_NET_FILE)

MODEL_SPAWN = "model_spawn"
PX4_SITL = "px4_sitl"
PX4_STATE_RESET = "px4_state_reset"
PX4_HEALTH_WATCH = "px4_health_watch"


def gz_model_base(sim_model: str) -> str:
    return sim_model.replace("gz_", "", 1)


def px4_instance(unit: UnitContext) -> int:
    drone = unit.resolved_robot().index - 1
    instance = (unit.instance.index - 1) * PX4_INSTANCES_PER_TARGET + drone
    if drone >= PX4_INSTANCES_PER_TARGET or instance > PX4_MAX_INSTANCE:
        raise RenderError(
            "assign PX4 SITL instance",
            f"drone index {drone} in SAGE instance {unit.instance.index} gives PX4 instance "
            f"{instance}; at most {PX4_INSTANCES_PER_TARGET} drones per target and PX4 instance "
            f"{PX4_MAX_INSTANCE} are supported",
            unit=unit.id,
        )
    return instance


def flight_controller_dependencies(unit: UnitContext) -> list[str]:
    return [PX4_SITL, CLOCK_BRIDGE] if unit.mode == SIMULATION else [ROUTER]


def agent_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    return [
        node(
            "odom_converter",
            package=MISSION_PACKAGE,
            executable="odom_converter.py",
            node_name="odom_converter",
            namespace=unit.namespace,
            remaps=TF_REMAPS,
            after=after,
        ),
        node(
            "battery_converter",
            package=MISSION_PACKAGE,
            executable="battery_converter.py",
            node_name="battery_converter",
            namespace=unit.namespace,
            after=after,
        ),
        node(
            "mission_server",
            package=MISSION_PACKAGE,
            executable="mission_server",
            node_name="mission_server",
            namespace=unit.namespace,
            params=(params_path("agent.yaml"),),
            param_values={"default_horizontal_velocity": MAX_HORIZONTAL_SPEED_M_S},
            after=after,
        ),
        node(
            "mission_progress",
            package=MISSION_PACKAGE,
            executable="mission_progress.py",
            node_name="mission_progress",
            namespace=unit.namespace,
            after=after,
        ),
    ]


def px4_processes(unit: UnitContext) -> list[Process]:
    if unit.mode != SIMULATION:
        return []
    robot = unit.resolved_robot()
    assert robot.spawn_pose is not None and unit.simulation is not None
    rootfs = f"{unit.run_dir}/{PX4_DIR}"
    world = unit.simulation.world.name
    return [
        world_ready(unit, gz_environment(unit), after=(ROUTER,)),
        spawn_process(
            MODEL_SPAWN,
            model=PackageFile(
                None, f"{PX4_GZ_MODELS_DIR}/{gz_model_base(PX4_SIM_MODEL)}/model.sdf"
            ),
            entity_name=robot.id,
            world_name=world,
            pose=robot.spawn_pose,
            env=gz_environment(unit),
            after=(WORLD_READY,),
            ready=ExitProbe(0),
        ),
        entity_watch(unit, after=(MODEL_SPAWN,)),
        node(
            PX4_HEALTH_WATCH,
            package=MISSION_PACKAGE,
            executable="px4_health_watch.py",
            node_name=PX4_HEALTH_WATCH,
            namespace=unit.namespace,
            after=(MODEL_SPAWN,),
        ),
        exec_process(
            PX4_STATE_RESET,
            cmd=("rm", "-rf", rootfs),
            after=(ROUTER,),
            ready=ExitProbe(0),
        ),
        exec_process(
            PX4_SITL,
            cmd=(PX4_BINARY, "-d", "-i", str(px4_instance(unit)), PX4_DATA_DIR),
            cwd=rootfs,
            env={
                "PX4_GZ_STANDALONE": "1",
                "PX4_SYS_AUTOSTART": str(PX4_AIRFRAME_ID),
                "PX4_SIM_MODEL": PX4_SIM_MODEL,
                "PX4_GZ_WORLD": world,
                "PX4_GZ_MODEL_POSE": ",".join(str(float(value)) for value in robot.spawn_pose),
                "PX4_GZ_MODEL_NAME": robot.id,
                **gz_environment(unit),
            },
            files=[
                StagedFile(params_path(f"{PX4_DIR}/{name}"), f"{rootfs}/{PX4_ZENOH_DIR}/{name}")
                for name in PX4_ZENOH_FILES
            ],
            after=(MODEL_SPAWN, PX4_STATE_RESET),
            ready=MessageProbe(unit.scoped(PX4_GLOBAL_POSITION_TOPIC)),
        ),
    ]


def processes(unit: UnitContext) -> list[Process]:
    entries = agent_processes(unit, flight_controller_dependencies(unit))
    entries.append(foxglove_bridge(unit, [ROUTER], remaps=TF_REMAPS))
    if unit.mode == SIMULATION:
        entries.append(clock_bridge(unit))
    entries += px4_processes(unit)
    entries.append(unit_agent(unit))
    entries += madum_processes(unit)
    entries += recorder_process(unit, recorded_topics(unit))
    return entries


def recorded_topics(unit: UnitContext) -> tuple[str, ...]:
    if unit.mode == SIMULATION:
        return RECORDED_TOPICS + RECORDED_TOPICS_SIMULATION
    return RECORDED_TOPICS


def unit_files(unit: UnitContext) -> dict[str, SourceFile]:
    files = {
        "agent.yaml": SourceFile(MISSION_PACKAGE, "config/agent.yaml"),
        FOXGLOVE_BRIDGE_PARAMS_FILE: SourceFile(MISSION_PACKAGE, "config/foxglove_bridge.yaml"),
    }
    for name in PX4_ZENOH_FILES:
        files[f"{PX4_DIR}/{name}"] = SourceFile(MISSION_PACKAGE, f"config/{PX4_DIR}/{name}")
    files.update(feature_files(unit, SourceFile(MISSION_PACKAGE, "config/qos_overrides.yaml")))
    return files


AEROSCOUT = RobotClass(
    id=ID,
    platform_kind="aerial",
    capabilities=CAPABILITIES,
    position_topic=POSITION_TOPIC,
    position_type=POSITION_TYPE,
    madum=MADUM,
    processes=processes,
    unit_files=unit_files,
    layout=SourceFile(MISSION_PACKAGE, "layouts/aeroscout.json"),
    gz_models_dir=PX4_GZ_MODELS_DIR,
)
