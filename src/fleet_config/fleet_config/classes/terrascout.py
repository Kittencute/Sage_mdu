from __future__ import annotations

from collections.abc import Mapping

from fleet_config.checks import (
    check_equal,
    check_footprint_covers,
    check_inflation_covers_footprint,
    check_lattice_resolution,
    read_yaml,
    value_at,
)
from fleet_config.common import (
    CLOCK_BRIDGE,
    ROUTER,
    TF_REMAPS,
    WORLD_READY,
    clock_bridge,
    component,
    components,
    entity_watch,
    exec_process,
    feature_files,
    foxglove_bridge,
    gz_bridge,
    gz_environment,
    madum_processes,
    merged,
    node,
    recorder_process,
    spawn_process,
    twist_stamper,
    unit_agent,
    world_ready,
)
from fleet_config.errors import RenderError
from fleet_config.model import (
    PHYSICAL,
    SIMULATION,
    UNIT_PLACEHOLDER,
    MadumProfile,
    RobotClass,
    SourceFile,
    site_value,
)
from fleet_config.paths import FOXGLOVE_BRIDGE_PARAMS_FILE, params_path
from fleet_config.unit import UnitContext
from fleet_unit.manifest import (
    ExitProbe,
    MessageProbe,
    NodeProbe,
    Process,
    RobotDescription,
    WaitFor,
)

ID = "terrascout"
DESCRIPTION_PACKAGE = "terrascout_description"
NAVIGATION_PACKAGE = "terrascout_navigation"
HARDWARE_PACKAGE = "terrascout_hardware"
MISSION_PACKAGE = "terrascout_mission"
SIMULATION_PACKAGE = "fleet_simulation"

URDF_MODEL = "urdf/scout.urdf.xacro"
GEOMETRY = SourceFile(DESCRIPTION_PACKAGE, "config/geometry.yaml")
NAV2_CONFIG = SourceFile(NAVIGATION_PACKAGE, "config/nav2.yaml")
GROUND_SEG_CONFIG = SourceFile(NAVIGATION_PACKAGE, "config/ground_seg.yaml")
CONTROLLERS_CONFIG = SourceFile(HARDWARE_PACKAGE, "config/controllers.yaml")
COSTMAPS = ("global_costmap", "local_costmap")
GLOBAL_COSTMAP_KEYS = (UNIT_PLACEHOLDER, "global_costmap", "global_costmap", "ros__parameters")
GOAL_MARGIN_M = 5.0
COLLISION_MONITOR = "collision_monitor"
COLLISION_POLYGONS = ("PolygonStop", "PolygonSlow")

FRAME_BASE = "base_link"
FRAME_GLOBAL = "map"
FRAME_LIDAR = "lidar_link"
FRAME_CAMERA = "fp_cam_link"

POSITION_TOPIC = "fixposition/odometry_llh"
DATUM_TOPIC = "fixposition/datum"
POSITION_TYPE = "sensor_msgs/msg/NavSatFix"

CAPABILITIES = (
    "battery_monitor",
    "camera",
    "diff_drive",
    "fixposition",
    "ground_segmentation",
    "livox",
    "microstrain",
    "nav2",
)

NAV2_GLOBAL_REMAPS: Mapping[str, str] = {
    "/tf": "tf",
    "/tf_static": "tf_static",
    "/map": "map",
    "/map_metadata": "map_metadata",
    "/clicked_point": "clicked_point",
    "/initialpose": "initialpose",
}

NAV2_SERVERS: tuple[tuple[str, str, str, bool, bool], ...] = (
    ("controller_server", "nav2_controller", "nav2_controller::ControllerServer", True, True),
    ("planner_server", "nav2_planner", "nav2_planner::PlannerServer", False, False),
    ("behavior_server", "nav2_behaviors", "behavior_server::BehaviorServer", True, False),
    ("bt_navigator", "nav2_bt_navigator", "nav2_bt_navigator::BtNavigator", False, False),
    (
        "velocity_smoother",
        "nav2_velocity_smoother",
        "nav2_velocity_smoother::VelocitySmoother",
        True,
        False,
    ),
    (
        "collision_monitor",
        "nav2_collision_monitor",
        "nav2_collision_monitor::CollisionMonitor",
        False,
        False,
    ),
)
NAV2_LIFECYCLE_MANAGER = "lifecycle_manager_navigation"
NAV2_CONTAINER = "nav2_container"
NAV2 = "nav2"
NAV2_PARAMS = params_path("nav2.yaml")
NAV2_LOG_BUFFER_VARIABLE = "RCUTILS_LOGGING_BUFFERED_STREAM"

ROBOT_SPAWN = "robot_spawn"
ROBOT_STATE_PUBLISHER = "robot_state_publisher"
ROS2_CONTROL_NODE = "ros2_control_node"
CONTROLLER_MANAGER = "controller_manager"
DIFF_DRIVE_CONTROLLER = "diff_drive_controller"
JOINT_STATE_BROADCASTER = "joint_state_broadcaster"
CONTROLLERS_PARAMS = params_path("controllers.yaml")
MISSION_SERVER = "mission_server"
KEEPOUT_SERVER = "keepout_server"

CAMERA_NAME = "front_camera"
CAMERA_TOPIC = f"{CAMERA_NAME}/image"
CAMERA_GSCAM_CONFIG = (
    'udpsrc port=5004 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"'
    " ! rtph264depay ! avdec_h264 ! queue max-size-buffers=1 leaky=downstream ! videoscale ! videorate"
    " ! video/x-raw,width=640,height=480,framerate=8/1 ! videoconvert ! jpegenc quality=50"
)

LIDAR_NAME = "lidar_primary"
LIDAR_PARAMS = params_path("lidar_primary.json")
LIDAR_XFER_FORMAT = 0
LIDAR_MULTI_TOPIC = 0
LIDAR_DATA_SRC = 0
LIDAR_PUBLISH_FREQ = 10.0
LIDAR_OUTPUT_DATA_TYPE = 0
LIDAR_BROADCAST_CODE = "livox0000000001"

ORIENTATION_IMU_TOPIC = {
    SIMULATION: "microstrain/imu/data",
    PHYSICAL: "microstrain/ekf/imu/data",
}

MADUM = MadumProfile(
    vehicle_id_base=2000,
    vehicle_type_code=2,
    max_speed=2.0,
    equipments_json='[{"id": 1, "name": "Front Camera", "type": 2, "status": 1}]',
    alarm_codes=(1, 2, 3, 4, 10, 12, 99),
    odometry_topic="fixposition/odometry_enu",
    battery_topic="battery_state",
)

RECORDED_TOPICS = (
    "/rosout",
    "battery_state",
    "clicked_point",
    "cmd_vel",
    "cmd_vel_auto",
    "cmd_vel_joy",
    "cmd_vel_nav",
    "cmd_vel_smoothed",
    "cmd_vel_teleop",
    "autonomy_lock",
    "estop",
    "estop_lock",
    "fixposition/datum",
    "fixposition/odometry_enu",
    "fixposition/odometry_llh",
    "fixposition/odometry_smooth",
    "fixposition/speed",
    "global_costmap/costmap",
    "goal_pose",
    "ground_points",
    "imu",
    "initialpose",
    "joint_states",
    "joy",
    "lidar_imu",
    "lidar_points_filtered",
    "lidar_points_raw",
    "local_costmap/costmap",
    "map",
    "map_metadata",
    "microstrain/ekf/imu/data",
    "microstrain/imu/data",
    "microstrain/imu/mag",
    "navsat",
    "obstacle_points",
    "odom",
    "plan",
    "platform/safety_stop",
    "raw_points",
    "robot_description",
    "teleop_takeover",
    "tf",
    "tf_static",
    "trajectories",
    "waypoints_geo",
)
RECORDED_TOPICS_SIMULATION = ("/clock", "front_camera/camera_info", "front_camera/image", "odom_gt")

JOYSTICK_DEVICE = "/dev/input/js0"
JOYSTICK_DEADZONE = 0.05
JOYSTICK_AUTOREPEAT_RATE = 20.0
JOY_PARAMS = params_path("joy.yaml")

CLOCK_SYNC = "clock_sync"
CLOCK_OFFSET_GATE_S = 0.001
CHRONY_ADDRESS = "127.0.0.1"

BATTERY = "battery"
SAFETY_STOP_TOPIC = "platform/safety_stop"
BATTERY_REMAPS: Mapping[str, str] = {"safety_stop": SAFETY_STOP_TOPIC}


def robot_description(unit: UnitContext) -> RobotDescription:
    arguments = {"namespace": unit.namespace}
    if unit.mode == PHYSICAL:
        arguments["can_interface"] = site_value(unit.site, unit.id, "drive_can_interface")
    return RobotDescription(package=DESCRIPTION_PACKAGE, path=URDF_MODEL, arguments=arguments)


def ground_spawn(unit: UnitContext) -> Process:
    robot = unit.resolved_robot()
    assert robot.spawn_pose is not None and unit.simulation is not None
    return spawn_process(
        ROBOT_SPAWN,
        robot_description=robot_description(unit),
        entity_name=robot.id,
        world_name=unit.simulation.world.name,
        pose=robot.spawn_pose,
        env=gz_environment(unit),
        after=(WORLD_READY,),
        ready=ExitProbe(0),
    )


def robot_state_publisher(unit: UnitContext, after: list[str]) -> Process:
    return node(
        ROBOT_STATE_PUBLISHER,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        node_name=ROBOT_STATE_PUBLISHER,
        namespace=unit.namespace,
        remaps=TF_REMAPS,
        robot_description=robot_description(unit),
        after=after,
        ready=NodeProbe(f"/{unit.namespace}/{ROBOT_STATE_PUBLISHER}"),
    )


def camera_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    if unit.mode == SIMULATION:
        return []
    return [
        node(
            CAMERA_NAME,
            package="gscam2",
            executable="gscam_main",
            node_name=CAMERA_NAME,
            namespace=unit.namespace,
            param_values={
                "gscam_config": CAMERA_GSCAM_CONFIG,
                "image_encoding": "jpeg",
                "preroll": False,
                "use_gst_timestamps": False,
                "frame_id": FRAME_CAMERA,
            },
            remaps={
                "image_raw/compressed": CAMERA_TOPIC,
                "camera_info": f"{CAMERA_NAME}/camera_info",
            },
            after=after,
            ready=MessageProbe(f"/{unit.namespace}/{CAMERA_TOPIC}"),
        )
    ]


def lidar_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    if unit.mode == SIMULATION:
        return []
    return [
        node(
            LIDAR_NAME,
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            node_name=f"{LIDAR_NAME}_publisher",
            namespace=unit.namespace,
            param_values={
                "xfer_format": LIDAR_XFER_FORMAT,
                "multi_topic": LIDAR_MULTI_TOPIC,
                "data_src": LIDAR_DATA_SRC,
                "publish_freq": LIDAR_PUBLISH_FREQ,
                "output_data_type": LIDAR_OUTPUT_DATA_TYPE,
                "frame_id": FRAME_LIDAR,
                "user_config_path": unit.file(LIDAR_PARAMS),
                "cmdline_input_bd_code": LIDAR_BROADCAST_CODE,
            },
            remaps={"livox/points": "lidar_points_raw", "livox/imu": "lidar_imu"},
            after=after,
            ready=MessageProbe(f"/{unit.namespace}/lidar_points_raw"),
        ),
        lidar_filter(unit, f"{LIDAR_NAME}_filter", after),
    ]


def lidar_filter(unit: UnitContext, name: str, after: list[str]) -> Process:
    return node(
        name,
        package=NAVIGATION_PACKAGE,
        executable="lidar_filter.py",
        node_name=name,
        namespace=unit.namespace,
        param_values={"filter_noise_tag": unit.mode == PHYSICAL},
        remaps={"cloud_in": "lidar_points_raw", "cloud_out": "lidar_points_filtered"},
        after=after,
        output="log",
        ready=MessageProbe(f"/{unit.namespace}/lidar_points_filtered"),
    )


def ground_segmentation(unit: UnitContext, after: list[str]) -> Process:
    values: dict[str, object] = {}
    if unit.mode == SIMULATION:
        values["use_imu_orientation"] = False
    return node(
        "ground_seg",
        package="ground_segmentation_ros2",
        executable="ground_segmentation_ros2_node",
        node_name="ground_seg",
        namespace=unit.namespace,
        params=(params_path("ground_seg.yaml"),),
        param_values=values,
        remaps=merged(
            {
                "/ground_segmentation/input_pointcloud": "lidar_points_filtered",
                "/ground_segmentation/input_imu": ORIENTATION_IMU_TOPIC[unit.mode],
                "/ground_segmentation/obstacle_points": "obstacle_points",
                "/ground_segmentation/ground_points": "ground_points",
                "/ground_segmentation/raw_points": "raw_points",
            },
            TF_REMAPS,
        ),
        after=after,
        output="log" if unit.mode == SIMULATION else "screen",
    )


def gnss_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    if unit.mode == SIMULATION:
        assert unit.simulation is not None
        datum = unit.simulation.datum
        return [
            node(
                "gnss_shim",
                package=NAVIGATION_PACKAGE,
                executable="gnss_shim.py",
                node_name="gnss_shim",
                namespace=unit.namespace,
                param_values={
                    "datum_latitude": datum.lat,
                    "datum_longitude": datum.lon,
                    "datum_altitude": datum.elevation,
                },
                remaps=TF_REMAPS,
                after=after,
                ready=MessageProbe(f"/{unit.namespace}/{POSITION_TOPIC}"),
            ),
        ]
    return [
        node(
            "gnss",
            package="fixposition_driver_ros2",
            executable="fixposition_driver_ros2_exec",
            node_name="gnss",
            namespace=unit.namespace,
            params=(params_path("gnss.yaml"),),
            remaps=TF_REMAPS,
            after=after,
            ready=MessageProbe(f"/{unit.namespace}/{POSITION_TOPIC}"),
        )
    ]


def imu_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    if unit.mode == SIMULATION:
        return []
    return [
        node(
            "imu_aux",
            package="microstrain_inertial_driver",
            executable="microstrain_inertial_driver_node",
            node_name="imu_aux",
            namespace=unit.namespace,
            params=(params_path("imu_aux.yaml"),),
            remaps=merged(
                TF_REMAPS,
                {"imu/data": "microstrain/imu/data", "ekf/imu/data": "microstrain/ekf/imu/data"},
            ),
            after=after,
            ready=MessageProbe(f"/{unit.namespace}/{ORIENTATION_IMU_TOPIC[PHYSICAL]}"),
        )
    ]


def battery_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    if unit.mode == SIMULATION:
        return [
            node(
                BATTERY,
                package=SIMULATION_PACKAGE,
                executable="simulated_battery.py",
                node_name=BATTERY,
                namespace=unit.namespace,
                params=(params_path("battery.yaml"),),
                remaps=BATTERY_REMAPS,
                after=after,
                ready=MessageProbe(f"/{unit.namespace}/{SAFETY_STOP_TOPIC}"),
            )
        ]
    return [
        node(
            BATTERY,
            package=HARDWARE_PACKAGE,
            executable="battery_monitor.py",
            node_name=BATTERY,
            namespace=unit.namespace,
            params=(params_path("battery.yaml"),),
            remaps=BATTERY_REMAPS,
            after=after,
            ready=MessageProbe(f"/{unit.namespace}/battery_state"),
        )
    ]


def hardware_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    return (
        camera_processes(unit, after)
        + lidar_processes(unit, after)
        + [ground_segmentation(unit, after)]
        + gnss_processes(unit, after)
        + imu_processes(unit, after)
        + battery_processes(unit, after)
    )


def clock_sync(after: list[str]) -> Process:
    return exec_process(
        CLOCK_SYNC,
        cmd=("chronyc", "-h", CHRONY_ADDRESS, "waitsync", "0", str(CLOCK_OFFSET_GATE_S), "0", "1"),
        after=after,
        ready=ExitProbe(0),
    )


def ros2_control_processes(unit: UnitContext) -> list[Process]:
    if unit.mode != PHYSICAL:
        return []
    return [
        node(
            ROS2_CONTROL_NODE,
            package="controller_manager",
            executable="ros2_control_node",
            namespace=unit.namespace,
            params=(CONTROLLERS_PARAMS,),
            remaps=merged(
                TF_REMAPS,
                {
                    f"{DIFF_DRIVE_CONTROLLER}/cmd_vel": "cmd_vel",
                    f"{DIFF_DRIVE_CONTROLLER}/odom": "odom",
                },
            ),
            robot_description=robot_description(unit),
            after=(ROBOT_STATE_PUBLISHER,),
            ready=NodeProbe(f"/{unit.namespace}/{CONTROLLER_MANAGER}"),
        ),
        node(
            f"spawn_{DIFF_DRIVE_CONTROLLER}",
            package="controller_manager",
            executable="spawner",
            namespace=unit.namespace,
            args=(DIFF_DRIVE_CONTROLLER, "-p", unit.file(CONTROLLERS_PARAMS)),
            after=(ROS2_CONTROL_NODE,),
            ready=ExitProbe(0),
        ),
        node(
            f"spawn_{JOINT_STATE_BROADCASTER}",
            package="controller_manager",
            executable="spawner",
            namespace=unit.namespace,
            args=(JOINT_STATE_BROADCASTER,),
            after=(ROS2_CONTROL_NODE,),
            ready=ExitProbe(0),
        ),
    ]


def localization_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    processes = [robot_state_publisher(unit, after)]
    processes += hardware_processes(unit, after)
    processes += ros2_control_processes(unit)
    if unit.mode == SIMULATION:
        processes.append(lidar_filter(unit, "lidar_filter", after))
    return processes


def twist_mux(unit: UnitContext, after: list[str]) -> Process:
    return node(
        "twist_mux",
        package="twist_mux",
        executable="twist_mux",
        node_name="twist_mux",
        namespace=unit.namespace,
        params=(params_path("twist_mux.yaml"),),
        remaps={"cmd_vel_out": "cmd_vel"},
        after=after,
    )


def teleop_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    processes = [
        twist_stamper(
            unit, after, frame_id=FRAME_BASE, in_topic="cmd_vel_teleop", out_topic="cmd_vel_joy"
        ),
        node(
            "teleop_takeover",
            package=NAVIGATION_PACKAGE,
            executable="teleop_takeover.py",
            node_name="teleop_takeover",
            namespace=unit.namespace,
            after=[*after, NAV2],
        ),
    ]
    if unit.mode == SIMULATION:
        return processes
    return [
        node(
            "joy_node",
            package="joy",
            executable="joy_node",
            node_name="joy_node",
            namespace=unit.namespace,
            param_values={
                "deadzone": JOYSTICK_DEADZONE,
                "autorepeat_rate": JOYSTICK_AUTOREPEAT_RATE,
            },
            wait_for=WaitFor(JOYSTICK_DEVICE),
            after=after,
        ),
        node(
            "teleop_node",
            package="teleop_twist_joy",
            executable="teleop_node",
            node_name="teleop_node",
            namespace=unit.namespace,
            params=(JOY_PARAMS,),
            remaps={"cmd_vel": "cmd_vel_teleop"},
            after=after,
            output="log",
        ),
        *processes,
    ]


def foxglove_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    namespace = unit.namespace
    return [
        foxglove_bridge(
            unit,
            after,
            remaps=merged(
                TF_REMAPS,
                {
                    "/move_base_simple/goal": f"/{namespace}/goal_pose",
                    "/initialpose": f"/{namespace}/initialpose",
                    "/clicked_point": f"/{namespace}/clicked_point",
                },
            ),
        ),
    ]


def nav2_server_remaps(publishes_velocity: bool, publishes_trajectories: bool) -> dict[str, str]:
    remaps = dict(NAV2_GLOBAL_REMAPS)
    if publishes_velocity:
        remaps["cmd_vel"] = "cmd_vel_nav"
    if publishes_trajectories:
        remaps["/trajectories"] = "trajectories"
    return remaps


def nav2_processes(unit: UnitContext, after: list[str]) -> list[Process]:
    namespace = unit.namespace
    servers = [
        component(
            package=package,
            plugin=plugin,
            node_name=node_name,
            namespace=namespace,
            params=(NAV2_PARAMS,),
            remaps=nav2_server_remaps(velocity, trajectories),
        )
        for node_name, package, plugin, velocity, trajectories in NAV2_SERVERS
    ]
    servers.append(
        component(
            package="nav2_lifecycle_manager",
            plugin="nav2_lifecycle_manager::LifecycleManager",
            node_name=NAV2_LIFECYCLE_MANAGER,
            namespace=namespace,
            params=(NAV2_PARAMS,),
        )
    )
    return [
        node(
            NAV2_CONTAINER,
            kind="container",
            package="rclcpp_components",
            executable="component_container_isolated",
            node_name=NAV2_CONTAINER,
            namespace=namespace,
            params=(NAV2_PARAMS,),
            remaps=NAV2_GLOBAL_REMAPS,
            env={NAV2_LOG_BUFFER_VARIABLE: "1"},
            after=after,
            ready=NodeProbe(f"/{namespace}/{NAV2_CONTAINER}"),
        ),
        components(NAV2, container=NAV2_CONTAINER, load=servers, after=(NAV2_CONTAINER,)),
    ]


def max_leg_length_m() -> float:
    nav2 = NAV2_CONFIG.resolve()
    costmap = value_at(read_yaml(nav2), GLOBAL_COSTMAP_KEYS, nav2)
    window_m = min(value_at(costmap, ("width",), nav2), value_at(costmap, ("height",), nav2))
    return window_m / 2 - GOAL_MARGIN_M


def mission_server(unit: UnitContext, after: list[str]) -> Process:
    return node(
        MISSION_SERVER,
        package=MISSION_PACKAGE,
        executable="mission_server.py",
        node_name=MISSION_SERVER,
        namespace=unit.namespace,
        param_values={
            "global_frame": FRAME_GLOBAL,
            "navigation_manager": NAV2_LIFECYCLE_MANAGER,
            "datum_topic": DATUM_TOPIC,
            "position_topic": POSITION_TOPIC,
            "max_leg_length_m": max_leg_length_m(),
        },
        after=after,
        ready=NodeProbe(f"/{unit.namespace}/{MISSION_SERVER}"),
    )


def keepout_server(unit: UnitContext, after: list[str]) -> Process:
    return node(
        KEEPOUT_SERVER,
        package=MISSION_PACKAGE,
        executable="keepout_server.py",
        node_name=KEEPOUT_SERVER,
        namespace=unit.namespace,
        param_values={
            "global_frame": FRAME_GLOBAL,
            "datum_topic": DATUM_TOPIC,
            "zone_topic": "polygon_stop",
            "obstacle_topic": "keepout_obstacle_points",
            "marker_topic": "keepout_zone_marker",
            "marker_height_m": 2.0,
            "marker_alpha": 0.8,
            "publish_period_s": 0.2,
        },
        after=after,
        ready=NodeProbe(f"/{unit.namespace}/{KEEPOUT_SERVER}"),
    )


def processes(unit: UnitContext) -> list[Process]:
    simulated = unit.mode == SIMULATION
    stage_after = [ROBOT_SPAWN, CLOCK_BRIDGE] if simulated else [CLOCK_SYNC]
    localization = localization_processes(unit, stage_after)
    navigation_after = [process.name for process in localization] if simulated else [CLOCK_SYNC]
    entries: list[Process] = []
    if simulated:
        entries += [
            world_ready(unit, gz_environment(unit), after=(ROUTER,)),
            ground_spawn(unit),
            entity_watch(unit, after=[ROBOT_SPAWN]),
            clock_bridge(unit),
        ]
    else:
        entries.append(clock_sync([ROUTER]))
    entries += localization
    if simulated:
        entries.append(
            gz_bridge(unit, stage_after, MessageProbe(f"/{unit.namespace}/{CAMERA_TOPIC}"))
        )
    entries.append(twist_mux(unit, [*stage_after, BATTERY] if simulated else stage_after))
    entries += teleop_processes(unit, stage_after)
    entries += foxglove_processes(unit, stage_after)
    entries += nav2_processes(unit, navigation_after)
    entries += [mission_server(unit, [NAV2]), keepout_server(unit, [NAV2]), unit_agent(unit)]
    entries += madum_processes(unit)
    entries += recorder_process(unit, recorded_topics(unit))
    return entries


def recorded_topics(unit: UnitContext) -> tuple[str, ...]:
    if unit.mode == SIMULATION:
        return RECORDED_TOPICS + RECORDED_TOPICS_SIMULATION
    return RECORDED_TOPICS


def unit_files(unit: UnitContext) -> dict[str, SourceFile]:
    files = {
        "nav2.yaml": NAV2_CONFIG,
        "ground_seg.yaml": GROUND_SEG_CONFIG,
        "twist_mux.yaml": SourceFile(NAVIGATION_PACKAGE, "config/twist_mux.yaml"),
        "joy.yaml": SourceFile(HARDWARE_PACKAGE, "config/joy.yaml"),
    }
    if unit.mode == SIMULATION:
        files["battery.yaml"] = SourceFile(SIMULATION_PACKAGE, "config/battery.yaml")
    else:
        files["gnss.yaml"] = SourceFile(HARDWARE_PACKAGE, "config/gnss.yaml")
        files["imu_aux.yaml"] = SourceFile(HARDWARE_PACKAGE, "config/imu_aux.yaml")
        files["battery.yaml"] = SourceFile(HARDWARE_PACKAGE, "config/battery.yaml")
        files["lidar_primary.json"] = SourceFile(HARDWARE_PACKAGE, "config/lidar_primary.json")
        files["controllers.yaml"] = CONTROLLERS_CONFIG
    files[FOXGLOVE_BRIDGE_PARAMS_FILE] = SourceFile(
        MISSION_PACKAGE, f"config/{unit.mode}/foxglove_bridge.yaml"
    )
    files.update(feature_files(unit, SourceFile(MISSION_PACKAGE, "config/qos_overrides.yaml")))
    return files


def check() -> None:
    geometry_file = GEOMETRY.resolve()
    geometry = read_yaml(geometry_file)
    outline = value_at(geometry, ("outline",), geometry_file)
    wheel = value_at(geometry, ("wheel",), geometry_file)
    length = value_at(outline, ("length",), geometry_file)
    width = value_at(outline, ("width",), geometry_file)
    nav2 = NAV2_CONFIG.resolve()
    check_lattice_resolution(nav2, UNIT_PLACEHOLDER)
    for costmap in COSTMAPS:
        costmap_keys = (UNIT_PLACEHOLDER, costmap, costmap, "ros__parameters")
        check_footprint_covers(nav2, (*costmap_keys, "footprint"), length, width)
        check_inflation_covers_footprint(nav2, costmap_keys)
    monitor_keys = (UNIT_PLACEHOLDER, COLLISION_MONITOR, "ros__parameters")
    monitor = value_at(read_yaml(nav2), monitor_keys, nav2)
    listed = value_at(monitor, ("polygons",), nav2)
    for polygon in COLLISION_POLYGONS:
        if polygon not in listed or value_at(monitor, (polygon, "enabled"), nav2) is not True:
            raise RenderError(
                f"check {COLLISION_MONITOR} polygon {polygon} in {nav2}",
                "the polygon is not listed in polygons or is not enabled",
            )
        check_footprint_covers(nav2, (*monitor_keys, polygon, "points"), length, width)
    controllers = CONTROLLERS_CONFIG.resolve()
    drive = value_at(
        read_yaml(controllers),
        (UNIT_PLACEHOLDER, DIFF_DRIVE_CONTROLLER, "ros__parameters"),
        controllers,
    )
    for key, geometry_key in (("wheel_radius", "radius"), ("wheel_separation", "separation")):
        check_equal(
            f"{DIFF_DRIVE_CONTROLLER} {key} against {geometry_file}",
            value_at(drive, (key,), controllers),
            value_at(wheel, (geometry_key,), geometry_file),
            controllers,
        )
    ground_seg = GROUND_SEG_CONFIG.resolve()
    lidar_height = value_at(geometry, ("base_height",), geometry_file) + value_at(
        geometry, ("lidar", "z"), geometry_file
    )
    check_equal(
        f"lidar_to_ground against the lidar height in {geometry_file}",
        value_at(read_yaml(ground_seg), ("/**", "ros__parameters", "lidar_to_ground"), ground_seg),
        -lidar_height,
        ground_seg,
    )


TERRASCOUT = RobotClass(
    id=ID,
    platform_kind="ground",
    capabilities=CAPABILITIES,
    position_topic=POSITION_TOPIC,
    position_type=POSITION_TYPE,
    description_package=DESCRIPTION_PACKAGE,
    madum=MADUM,
    processes=processes,
    unit_files=unit_files,
    layout=SourceFile(MISSION_PACKAGE, "layouts/terrascout.json"),
    bridge=SourceFile(DESCRIPTION_PACKAGE, "config/gz_bridge.yaml"),
    check=check,
)
