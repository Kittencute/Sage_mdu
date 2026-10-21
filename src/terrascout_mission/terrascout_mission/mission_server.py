#!/usr/bin/env python3
from __future__ import annotations
import itertools
import json
import math
import os
import sys
from pathlib import Path
from dataclasses import dataclass, field
from enum import Enum, auto
from functools import partial
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from fleet_interfaces.action import ExecuteMission
from fleet_interfaces.msg import MissionCommand, MissionPlan, MissionState
from geographic_msgs.msg import GeoPoint
from fleet_interfaces.srv import ControlMission
from foxglove_msgs.msg import GeoJSON
from lifecycle_msgs.msg import State, TransitionEvent
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from fleet_common import signals
from fleet_common.geodesy import llh_to_enu, enu_to_map
from fleet_common.params import declare
from fleet_common.qos import latched_qos
from fleet_common.waypoint_geojson import build_waypoints_geojson
from terrascout_mission.foxglove_mission_input import FoxgloveMissionInput
from terrascout_mission.route_staging import next_navigation_segment, next_path_segment
POSITION_KINDS = frozenset({MissionCommand.WAYPOINT, MissionCommand.HOME})
INSTANT_KINDS = frozenset({MissionCommand.TAKEOFF, MissionCommand.LAND})
NAVIGATION_NODES_PARAMETER = "node_names"
TICK_PERIOD_S = 0.1
GOAL_RESPONSE_TIMEOUT_S = 10.0
STOP_TIMEOUT_S = 10.0
POSITION_MAX_AGE_S = 2.0
HOME_POINT = 0
FACING_EAST = 0.0
TERMINAL_GOAL_STATUSES = frozenset(
    {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
)
CANCELLED_BY_MANAGER = "cancelled by the manager"
OPERATOR_TOOK_OVER = "operator took over"
STOP_OVERDUE = (
    f"stop navigation failed: cause: navigate_to_pose did not end the goal "
    f"within {STOP_TIMEOUT_S} s"
)
class Phase(Enum):
    NAVIGATING = auto()
    HOLDING = auto()
    SUSPENDING = auto()
    PAUSED = auto()
    ABORTING = auto()
    RETURNING = auto()
class MissionError(Exception):
    pass
@dataclass
class _Navigation:
    send: Future | None
    answer_by: Time
    staged: bool = False
    plan: Future | None = None
    plan_result: Future | None = None
    destination: tuple[float, float] | None = None
    needs_staging: bool = False
    final_yaw: float = 0.0
    result: Future | None = None
    cancel: Future | None = None
@dataclass
class _Mission:
    plan: MissionPlan
    goal: object
    points: list[tuple[float, float]]
    yaws: list[float]
    point_of: dict[int, int]
    status: list[int]
    waypoints: list[dict[str, float]]
    start_position: dict[str, float]
    result: Future = field(default_factory=Future)
    phase: Phase = Phase.NAVIGATING
    index: int = 0
    navigation: _Navigation | None = None
    waypoint_retries: int = 0
    retry_after: Time | None = None
    last_reached_point: int = 0
    backtrack_pending: bool = False
    resume_command_index: int | None = None
    hold_until: Time | None = None
    deadline: Time | None = None
    overdue: str = ""
    abort_reason: str = ""
def goal_yaws(points: list[tuple[float, float]]) -> list[float]:
    yaws = []
    for index, point in enumerate(points):
        ahead = [other for other in points[index + 1 :] if other != point]
        behind = [other for other in points[:index] if other != point]
        if ahead:
            yaws.append(_bearing(point, ahead[0]))
        elif behind:
            yaws.append(_bearing(behind[-1], point))
        else:
            yaws.append(FACING_EAST)
    return yaws
def _bearing(start: tuple[float, float], end: tuple[float, float]) -> float:
    return math.atan2(end[1] - start[1], end[0] - start[0])
class GroundMissionServer(Node):
    def __init__(self) -> None:
        super().__init__("mission_server")
        self._global_frame = declare(
            self,
            "global_frame",
            Parameter.Type.STRING,
            "Frame in which navigation goals are sent",
            constraints="a tf frame id",
        )
        self._robot_base_frame = declare(
            self,
            "robot_base_frame",
            Parameter.Type.STRING,
            "Robot base frame used to read the current pose for staged goals",
            constraints="a tf frame id",
        )
        self._navigation_manager = declare(
            self,
            "navigation_manager",
            Parameter.Type.STRING,
            "Name of the lifecycle manager node that owns the navigation nodes",
            constraints="a node name relative to the robot namespace",
        )
        self._datum_topic = declare(
            self,
            "datum_topic",
            Parameter.Type.STRING,
            "Topic that publishes the geodetic origin of the map frame",
            constraints="a ROS topic name",
        )
        self._position_topic = declare(
            self,
            "position_topic",
            Parameter.Type.STRING,
            "Topic that publishes the robot geodetic position",
            constraints="a ROS topic name",
        )
        self._localization_max_horizontal_std_m = declare(
            self,
            "localization_max_horizontal_std_m",
            Parameter.Type.DOUBLE,
            "Maximum reported horizontal position standard deviation allowed for resume, in metres",
            0.1,
            100.0,
            "Configured by fleet_config; recommended default is 5.0 m",
        )
        self._max_leg_length_m = declare(
            self,
            "max_leg_length_m",
            Parameter.Type.DOUBLE,
            "Longest straight leg and farthest distance from home a mission may contain, in metres",
            1.0,
            1000.0,
            "at most half the navigation global costmap window minus the goal margin",
        )
        self._max_navigation_segment_m = declare(
            self,
            "max_navigation_segment_m",
            Parameter.Type.DOUBLE,
            "Maximum straight-line distance for each rolling-costmap navigation goal, in metres",
            1.0,
            100.0,
        )
        self._waypoint_retry_limit = declare(
            self,
            "waypoint_retry_limit",
            Parameter.Type.INTEGER,
            "Additional same-goal attempts after Nav2 aborts a mission waypoint",
            0,
            3,
            "0 disables mission-layer retry; unreachable waypoints are never skipped",
        )
        self._waypoint_retry_delay_s = declare(
            self,
            "waypoint_retry_delay_s",
            Parameter.Type.DOUBLE,
            "Delay before retrying a waypoint after Nav2 exhausts its behavior-tree recoveries",
            0.0,
            30.0,
        )
        self._map_yaw_offset_deg = declare(
            self,
            "map_yaw_offset_deg",
            Parameter.Type.DOUBLE,
            "Counter-clockwise yaw from geographic east to map +X, in degrees",
            -180.0,
            180.0,
            "0 for an ENU-aligned map; set only from measured axis calibration",
        )
        self._continue_after_waypoint_failure = declare(
            self,
            "continue_after_waypoint_failure",
            Parameter.Type.BOOL,
            "After retries, mark a failed waypoint and try the next waypoint",
            constraints="explicitly reports the failed command; does not apply to the last waypoint",
        )
        self._saved_home = declare(
            self,
            "saved_home",
            Parameter.Type.STRING,
            "Saved GPS Home position as latitude,longitude; empty when not configured",
        )
        self._last_mission_start: tuple[float, float] | None = None
        namespace = self.get_namespace().strip("/") or "default"
        self._snapshot_path = Path("/run/sage") / namespace / "mission_state.json"
        self._last_failure_reason = ""
        self._recovered_failure_reason = ""
        self._clear_pending = False
        self._mission: _Mission | None = None
        self._restore_pending = False
        self._recovery_complete = False
        self._results: dict[bytes, Future] = {}
        self._datum: NavSatFix | None = None
        self._position: NavSatFix | None = None
        self._spawn_position: tuple[float, float] | None = None
        self._taken_over = False
        self._navigation_states: dict[str, int | None] = {}
        self._node_list: Future | None = None
        self._state_requests: dict[str, tuple[object, Future | None]] = {}
        self._ready = False
        self._not_ready_reason = ""
        self._last_state = MissionState(mission_id=-1, current_command_id=-1)
        self._state_pub = self.create_publisher(MissionState, "mission_state", latched_qos())
        self._restore_lock_pub = self.create_publisher(Bool, "mission_restore_lock", latched_qos())
        self._geojson_pub = self.create_publisher(GeoJSON, "waypoints_geo", latched_qos())
        self._navigation_parameters_client = self.create_client(
            GetParameters, f"{self._navigation_manager}/get_parameters"
        )
        self._path_callback_group = ReentrantCallbackGroup()
        self._path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose",
            callback_group=self._path_callback_group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._startup_cancel_client = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
        )
        self._startup_cancel_sent = False
        self._startup_cancel_done = False
        self._server = ActionServer(
            self,
            ExecuteMission,
            "execute_mission",
            self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=self._on_cancel,
        )
        self.create_service(
            ControlMission, "control_mission", self._on_control,
        )
        self.create_service(Trigger, "clear_mission", self._on_clear_mission)
        self.create_service(Trigger, "get_current_point", self._on_get_current_point)
        self.create_subscription(String, "add_mission_waypoint", self._on_add_waypoint, 10)
        self.create_service(Trigger, "get_home_point", self._on_get_home_point)
        self._foxglove_mission_input = FoxgloveMissionInput(
            self, math.radians(self._map_yaw_offset_deg)
        )
        self.create_subscription(Bool, "teleop_takeover", self._on_takeover, latched_qos())
        self.create_subscription(NavSatFix, self._datum_topic, self._on_datum, latched_qos())
        self.create_subscription(
            NavSatFix, self._position_topic, self._on_position, qos_profile_sensor_data
        )
        self.create_timer(TICK_PERIOD_S, self._tick)
        self._update_readiness()

    def _write_snapshot(self, mission: _Mission | None, status: int | None = None) -> None:
        if mission is None:
            return
        payload = {
            "mission_id": mission.plan.id,
            "home": [mission.plan.home.latitude, mission.plan.home.longitude,
                     mission.plan.home.altitude],
            "commands": [
                {"id": c.id, "kind": c.kind, "latitude": c.latitude,
                 "longitude": c.longitude, "altitude_agl": c.altitude_agl,
                 "hold_s": c.hold_s, "acceptance_radius_m": c.acceptance_radius_m,
                 "yaw_deg": c.yaw_deg, "parameters": list(c.parameters)}
                for c in mission.plan.commands
            ],
            "command_status": list(mission.status),
            "current_index": mission.index,
            "mission_status": status if status is not None else MissionState.MISSION_PAUSED,
            "phase": "paused" if status == MissionState.MISSION_PAUSED else "terminal" if status in (
                MissionState.MISSION_FINISHED, MissionState.MISSION_FAILED,
                MissionState.MISSION_ABORTED,
            ) else "paused",
            "failure_reason": self._last_failure_reason or mission.abort_reason,
            "start_position": mission.start_position,
            "waypoints": mission.waypoints,
        }
        self._snapshot_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._snapshot_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, allow_nan=True) + "\n", encoding="utf-8")
        os.replace(temporary, self._snapshot_path)

    def _restore_mission_snapshot(self) -> None:
        try:
            payload = json.loads(self._snapshot_path.read_text(encoding="utf-8"))
            if payload.get("phase") != "paused":
                if (
                    payload.get("phase") == "terminal"
                    and int(payload.get("mission_status", MissionState.MISSION_IDLE))
                    == MissionState.MISSION_FAILED
                ):
                    payload["phase"] = "paused"
                    self.get_logger().warning(
                        "restoring failed mission as suspended at its failed command; "
                        "explicit resume will retry that command"
                    )
                else:
                    if payload.get("phase") == "terminal":
                        command_ids = [int(command["id"]) for command in payload["commands"]]
                        if command_ids:
                            index = max(0, min(int(payload["current_index"]), len(command_ids) - 1))
                            msg = MissionState()
                            msg.mission_id = int(payload["mission_id"])
                            msg.current_command_id = command_ids[index]
                            msg.command_ids = command_ids
                            msg.command_status = list(payload["command_status"])
                            msg.mission_status = int(payload["mission_status"])
                            msg.failure_reason = payload.get("failure_reason", "")
                            self._last_failure_reason = msg.failure_reason
                            self._publish(msg)
                    return
            if self._taken_over:
                self.get_logger().warning(
                    "saved mission remains suspended; explicit resume is required"
                )
            plan = MissionPlan()
            plan.id = int(payload["mission_id"])
            plan.home = GeoPoint(latitude=payload["home"][0], longitude=payload["home"][1],
                                 altitude=payload["home"][2])
            commands = []
            for item in payload["commands"]:
                command = MissionCommand()
                for key in ("id", "kind", "latitude", "longitude", "altitude_agl",
                            "hold_s", "acceptance_radius_m", "yaw_deg"):
                    setattr(command, key, item[key])
                command.parameters = item["parameters"]
                commands.append(command)
            plan.commands = commands
            if not commands:
                raise ValueError("restored mission has no commands")
            positions, points = self._plan_points(plan)
            statuses = list(payload["command_status"])
            index = max(0, min(int(payload["current_index"]), len(commands) - 1))
            if statuses[index] == MissionState.COMMAND_RUNNING:
                statuses[index] = MissionState.COMMAND_PAUSED
            self._mission = _Mission(
                plan=plan, goal=None, points=points, yaws=goal_yaws(points),
                point_of={command_index: point for point, command_index in
                          enumerate(positions, start=1)}, status=statuses,
                start_position=payload.get("start_position", {}),
                waypoints=payload.get("waypoints", []), phase=Phase.PAUSED, index=index,
            )
            self._last_failure_reason = payload.get("failure_reason", "")
            self._recovered_failure_reason = self._last_failure_reason
            self._restore_lock_pub.publish(Bool(data=True))
            self.get_logger().warning(
                f"restored mission {plan.id} suspended at command {commands[index].id}; "
                "explicit resume is required"
            )
        except FileNotFoundError:
            return
        except (KeyError, TypeError, ValueError, IndexError, json.JSONDecodeError) as error:
            self.get_logger().error(f"could not restore mission snapshot: {error}")
    def _track_navigation(self) -> None:
        if not self._navigation_states:
            self._request_node_list()
            return
        for name, (client, request) in list(self._state_requests.items()):
            if request is None:
                if client.service_is_ready():
                    self._state_requests[name] = (client, client.call_async(GetState.Request()))
                continue
            if not request.done():
                continue
            del self._state_requests[name]
            if self._navigation_states[name] is None:
                self._navigation_states[name] = request.result().current_state.id

    def _cancel_navigation_after_restart(self) -> None:
        if self._startup_cancel_done:
            return
        if self._startup_cancel_sent:
            return
        if not self._startup_cancel_client.service_is_ready():
            return
        self._startup_cancel_sent = True
        future = self._startup_cancel_client.call_async(CancelGoal.Request())
        future.add_done_callback(self._startup_cancel_complete)

    def _startup_cancel_complete(self, future) -> None:
        try:
            response = future.result()
            if response.return_code in (
                CancelGoal.Response.ERROR_NONE,
                CancelGoal.Response.ERROR_REJECTED,
                CancelGoal.Response.ERROR_GOAL_TERMINATED,
            ):
                self._startup_cancel_done = True
            else:
                self._startup_cancel_sent = False
                self.get_logger().error(
                    f"startup Nav2 cancel-all failed with return code {response.return_code}"
                )
        except Exception as error:
            self._startup_cancel_sent = False
            self.get_logger().error(f"startup Nav2 cancellation failed: {error}")
    def _request_node_list(self) -> None:
        if self._node_list is None:
            if self._navigation_parameters_client.service_is_ready():
                self._node_list = self._navigation_parameters_client.call_async(
                    GetParameters.Request(names=[NAVIGATION_NODES_PARAMETER])
                )
            return
        if not self._node_list.done():
            return
        names = list(self._node_list.result().values[0].string_array_value)
        if not names:
            raise RuntimeError(
                f"track navigation failed: cause: {self._navigation_manager} has no "
                f"{NAVIGATION_NODES_PARAMETER}"
            )
        self._navigation_states = {name: None for name in names}
        for name in names:
            self.create_subscription(
                TransitionEvent,
                f"{name}/transition_event",
                partial(self._on_transition, name),
                10,
            )
            self._state_requests[name] = (
                self.create_client(GetState, f"{name}/get_state"),
                None,
            )
    def _on_transition(self, name: str, msg: TransitionEvent) -> None:
        self._navigation_states[name] = msg.goal_state.id
    def _on_datum(self, msg: NavSatFix) -> None:
        if self._datum is None:
            self._datum = msg
            if not self._restore_pending:
                self._restore_pending = True
                self._restore_mission_snapshot()
                self._recovery_complete = True
            if self._mission is not None:
                self._publish_state(self._mission, MissionState.MISSION_PAUSED)
    def _on_position(self, msg: NavSatFix) -> None:
        self._position = msg
        if (
            self._spawn_position is None
            and msg.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        ):
            self._spawn_position = (msg.latitude, msg.longitude)
    def _update_readiness(self) -> None:
        reasons = []
        if not self._navigation_states:
            reasons.append(f"waiting for the node list of {self._navigation_manager}")
        inactive = [
            name
            for name, state in self._navigation_states.items()
            if state != State.PRIMARY_STATE_ACTIVE
        ]
        if inactive:
            reasons.append(f"navigation nodes not active: {', '.join(inactive)}")
        if not self._nav.server_is_ready():
            reasons.append("navigate_to_pose server unavailable")
        if self._datum is None:
            reasons.append(f"no datum on {self._datum_topic}")
        localization_reason = self._localization_reason(check_tf=False)
        if localization_reason:
            reasons.append(localization_reason)
        if self._taken_over:
            reasons.append("manual control; call resume_autonomy")
        ready, reason = not reasons, "; ".join(reasons)
        if (ready, reason) == (self._ready, self._not_ready_reason):
            return
        self._ready, self._not_ready_reason = ready, reason
        self.get_logger().info(
            "ready for missions" if ready else f"not ready for missions: {reason}"
        )
        self._publish(self._last_state)
    def _on_goal(self, request) -> GoalResponse:
        plan = request.plan
        cause = self._rejection(plan)
        if cause:
            self.get_logger().error(f"accept mission {plan.id} failed: cause: {cause}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    def _rejection(self, plan: MissionPlan) -> str:
        if not self._ready:
            return f"not ready: {self._not_ready_reason}"
        if not self._startup_cancel_done:
            return "startup navigation cancellation is not confirmed"
        if self._mission is not None:
            return f"mission {self._mission.plan.id} is active"
        if not plan.commands:
            return "no commands"
        for command in plan.commands:
            if command.kind not in POSITION_KINDS and command.kind not in INSTANT_KINDS:
                return f"command {command.id} has unsupported kind {command.kind}"
        if self._position.status.status < NavSatStatus.STATUS_FIX:
            return f"robot position status {self._position.status.status} is not a fix"
        return self._leg_rejection(plan)
    def _leg_rejection(self, plan: MissionPlan) -> str:
        positions, points = self._plan_points(plan)
        route = [self._map_point(self._position.latitude, self._position.longitude), *points[1:]]
        names = ["the robot position"] + [f"command {plan.commands[i].id}" for i in positions]
        for leg, (start, end) in enumerate(itertools.pairwise(route), start=1):
            length = math.dist(start, end)
            if length > self._max_leg_length_m:
                return (
                    f"leg {leg} from {names[leg - 1]} to {names[leg]} is {length:.1f} m, "
                    f"longer than {self._max_leg_length_m} m"
                )
        for name, point in zip(names, route, strict=True):
            distance = math.dist(point, points[HOME_POINT])
            if distance > self._max_leg_length_m:
                return (
                    f"{name} is {distance:.1f} m from home, farther than {self._max_leg_length_m} m"
                )
        return ""
    def _plan_points(self, plan: MissionPlan) -> tuple[list[int], list[tuple[float, float]]]:
        positions = [
            index for index, command in enumerate(plan.commands) if command.kind in POSITION_KINDS
        ]
        targets = [plan.home] + [
            plan.home if plan.commands[index].kind == MissionCommand.HOME else plan.commands[index]
            for index in positions
        ]
        return positions, [self._map_point(target.latitude, target.longitude) for target in targets]
    def _on_accepted(self, goal_handle) -> None:
        plan = goal_handle.request.plan
        positions, points = self._plan_points(plan)
        mission = _Mission(
            plan=plan,
            goal=goal_handle,
            points=points,
            yaws=goal_yaws(points),
            point_of={index: point for point, index in enumerate(positions, start=1)},
            status=[MissionState.COMMAND_WAITING] * len(plan.commands),
            start_position={"lat": self._position.latitude, "lon": self._position.longitude},
            waypoints=[
                {"lat": command.latitude, "lon": command.longitude}
                for command in plan.commands
                if command.kind == MissionCommand.WAYPOINT
            ],
        )
        self._last_mission_start = (self._position.latitude, self._position.longitude)
        self._last_failure_reason = ""
        self._recovered_failure_reason = ""
        self._mission = mission
        self._last_state.failure_reason = ""
        self._write_snapshot(mission)
        self._results[bytes(goal_handle.goal_id.uuid)] = mission.result
        goal_handle.execute()
        self.get_logger().info(f"mission {plan.id} started with {len(plan.commands)} commands")
        self._publish_geojson(mission, 0)
        self._start_command(mission)
    async def _execute(self, goal_handle) -> ExecuteMission.Result:
        return await self._results.pop(bytes(goal_handle.goal_id.uuid))
    def _on_cancel(self, goal_handle) -> CancelResponse:
        mission = self._mission
        if mission is None or mission.goal != goal_handle:
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT
    def _on_takeover(self, msg: Bool) -> None:
        self._taken_over = msg.data
        if msg.data and self._mission is not None:
            mission = self._mission
            if mission.phase in (Phase.NAVIGATING, Phase.HOLDING):
                mission.phase = Phase.SUSPENDING
                mission.hold_until = None
                self._set_deadline(mission, STOP_TIMEOUT_S, STOP_OVERDUE)
                self._write_snapshot(mission, MissionState.MISSION_PAUSED)
                self.get_logger().warning(
                    f"mission {mission.plan.id} suspended for operator takeover"
                )
    def _on_add_waypoint(self, msg: String) -> None:
        mission = self._mission
        if mission is None or mission.phase not in (Phase.NAVIGATING, Phase.HOLDING, Phase.PAUSED):
            self.get_logger().warning("add waypoint rejected: no editable active mission")
            return
        try:
            data = json.loads(msg.data)
            lat, lon, alt = float(data["lat"]), float(data["lon"]), float(data["alt"])
            mode = data["mode"]
            if mode not in ("ordered", "unordered") or not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError("invalid coordinates or mode")
            commands = list(mission.plan.commands)
            # Preserve the active target, reorder only future waypoint commands.
            future_indices = [i for i in range(mission.index + 1, len(commands))
                              if commands[i].kind == MissionCommand.WAYPOINT]
            new = MissionCommand()
            new.id = max(c.id for c in commands) + 1
            new.kind = MissionCommand.WAYPOINT
            new.latitude, new.longitude, new.altitude_agl = lat, lon, alt
            new.yaw_deg = float("nan")
            if mode == "unordered" and future_indices:
                candidates = [commands[i] for i in future_indices] + [new]
                active = commands[mission.index]
                if active.kind == MissionCommand.WAYPOINT:
                    current = (active.latitude, active.longitude)
                else:
                    current = (self._position.latitude, self._position.longitude)
                ordered = []
                while candidates:
                    best = min(candidates, key=lambda c: math.dist(self._map_point(*current),
                                                                   self._map_point(c.latitude, c.longitude)))
                    ordered.append(best)
                    candidates.remove(best)
                    current = (best.latitude, best.longitude)
                for i, cmd in zip(future_indices, ordered[:-1]):
                    commands[i] = cmd
                insertion = next((i for i in range(mission.index + 1, len(commands))
                                  if commands[i].kind == MissionCommand.HOME), len(commands))
                commands.insert(insertion, ordered[-1])
            else:
                insertion = next((i for i in range(mission.index + 1, len(commands))
                                  if commands[i].kind == MissionCommand.HOME), len(commands))
                commands.insert(insertion, new)
            # Validate the complete revised route before modifying the active mission.
            candidate = MissionPlan()
            candidate.id = mission.plan.id
            candidate.home = mission.plan.home
            candidate.commands = commands
            rejection = self._leg_rejection(candidate)
            if rejection:
                raise ValueError(rejection)
            mission.plan.commands = commands
            mission.status = mission.status[:insertion] + [MissionState.COMMAND_WAITING] + mission.status[insertion:]
            positions, mission.points = self._plan_points(mission.plan)
            mission.yaws = goal_yaws(mission.points)
            mission.point_of = {index: point for point, index in enumerate(positions, start=1)}
            mission.waypoints = [{"lat": c.latitude, "lon": c.longitude}
                                 for c in commands if c.kind == MissionCommand.WAYPOINT]
            self._write_snapshot(mission)
            self._publish_geojson(mission, self._waypoints_before(mission, mission.index))
            self._publish_state(mission, MissionState.MISSION_RUNNING)
            self.get_logger().info(f"added waypoint {lat},{lon} ({mode})")
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f"add waypoint rejected: {exc}")

    def _on_get_current_point(self, request, response):
        point = (self._position.latitude, self._position.longitude) if self._position else None
        response.success = point is not None
        response.message = f"{point[0]:.9f},{point[1]:.9f}" if point else "no robot position available"
        return response

    def _on_get_home_point(self, request, response):
        saved = self.get_parameter("saved_home").value
        if saved:
            response.success = True
            response.message = saved
        elif self._spawn_position is not None:
            response.success = True
            response.message = (
                f"{self._spawn_position[0]:.9f},{self._spawn_position[1]:.9f}"
            )
        else:
            response.success = False
            response.message = "spawn GPS position is not available yet"
        return response

    def _clear_visualization(self):
        msg = GeoJSON()
        msg.geojson = '{"type":"FeatureCollection","features":[]}'
        self._geojson_pub.publish(msg)

    def _on_clear_mission(self, request, response):
        if self._mission is not None:
            self._clear_pending = True
            self._abort(self._mission, "mission cleared by operator")
            response.success = True
            response.message = "mission cancellation requested; visualization will clear after stop"
        else:
            self._clear_visualization()
            response.success = True
            response.message = "mission visualization cleared"
        return response

    async def _on_control(self, request, response):
        response.accepted, response.reason = await self._control(request.request)
        return response
    async def _control(self, request: int) -> tuple[bool, str]:
        mission = self._mission
        if mission is None:
            if not self._recovery_complete:
                return False, "mission recovery is waiting for map datum; remain stopped"
            return False, "no active mission"
        if request == ControlMission.Request.SUSPEND:
            if mission.phase not in (Phase.NAVIGATING, Phase.HOLDING):
                return False, "mission is not running"
            mission.phase = Phase.SUSPENDING
            self._set_deadline(mission, STOP_TIMEOUT_S, STOP_OVERDUE)
            return True, ""
        if request == ControlMission.Request.RESUME:
            if mission.phase != Phase.PAUSED:
                return False, "mission is not suspended"
            localization_reason = self._localization_reason(check_tf=True)
            if localization_reason:
                return False, f"{localization_reason}; mission remains suspended"
            if not self._startup_cancel_done:
                return False, "startup navigation cancellation is not confirmed; remain stopped"
            if not self._nav.server_is_ready() or not self._path_client.server_is_ready():
                return False, "navigation planner/controller is unavailable; mission remains suspended"
            path_error = await self._resume_path_error(mission)
            if path_error:
                return False, f"cannot resume mission: {path_error}; mission remains suspended"
            if mission.status[mission.index] in (
                MissionState.COMMAND_FAILED,
                MissionState.COMMAND_ABORTED,
                MissionState.COMMAND_PAUSED,
            ):
                mission.status[mission.index] = MissionState.COMMAND_WAITING
            mission.abort_reason = ""
            self._last_failure_reason = ""
            self._recovered_failure_reason = ""
            self._start_command(mission)
            return True, ""
        return False, f"unknown request {request}"

    async def _resume_path_error(self, mission: _Mission) -> str:
        """Require a fresh feasible path from current TF pose before accepting resume."""
        if mission.index >= len(mission.plan.commands):
            return "there is no remaining command to resume"
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return "current robot transform is unavailable"
        current = (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        destination = mission.points[mission.point_of.get(mission.index, HOME_POINT)]
        target, _ = next_navigation_segment(
            current, destination, self._max_navigation_segment_m
        )
        request = ComputePathToPose.Goal()
        request.start.header.frame_id = self._global_frame
        request.start.header.stamp = self._now().to_msg()
        request.start.pose.position.x = current[0]
        request.start.pose.position.y = current[1]
        request.start.pose.orientation.w = 1.0
        request.goal.header.frame_id = self._global_frame
        request.goal.header.stamp = self._now().to_msg()
        request.goal.pose.position.x = target[0]
        request.goal.pose.position.y = target[1]
        yaw = mission.yaws[mission.point_of.get(mission.index, HOME_POINT)]
        request.goal.pose.orientation.z = math.sin(yaw / 2.0)
        request.goal.pose.orientation.w = math.cos(yaw / 2.0)
        request.planner_id = "GridBased"
        request.use_start = True
        try:
            send_future = self._path_client.send_goal_async(request)
            handle = await self._await_ros_future(send_future, GOAL_RESPONSE_TIMEOUT_S)
            if not handle.accepted:
                return "planner rejected the fresh path request"
            result = await self._await_ros_future(
                handle.get_result_async(), GOAL_RESPONSE_TIMEOUT_S
            )
        except TimeoutError:
            return "planner did not produce a fresh path before timeout"
        except Exception as error:
            return f"fresh path request failed: {error}"
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
            return f"no valid path from current position (planner status {result.status})"
        return ""

    async def _await_ros_future(self, source: Future, timeout_s: float):
        """Await an rclpy Future with an executor-driven steady-clock timeout."""
        completed: Future = Future()

        def on_source_done(future):
            if completed.done():
                return
            try:
                completed.set_result(future.result())
            except Exception as error:
                completed.set_exception(error)

        def on_timeout():
            if not completed.done():
                completed.set_exception(TimeoutError())

        timer = self.create_timer(
            timeout_s,
            on_timeout,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
            callback_group=ReentrantCallbackGroup(),
        )
        source.add_done_callback(on_source_done)
        try:
            return await completed
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    def _localization_reason(self, check_tf: bool) -> str:
        fix = self._position
        if fix is None:
            return f"no robot position on {self._position_topic}"
        if fix.status.status < NavSatStatus.STATUS_FIX:
            return f"position status {fix.status.status} is not a valid fix"
        if not all(math.isfinite(value) for value in (fix.latitude, fix.longitude)):
            return "position latitude/longitude is not finite"
        if not (-90.0 <= fix.latitude <= 90.0 and -180.0 <= fix.longitude <= 180.0):
            return "position latitude/longitude is outside valid bounds"
        stamp = Time.from_msg(fix.header.stamp)
        if stamp.nanoseconds <= 0:
            return "position timestamp is missing"
        age_ns = (self._now() - stamp).nanoseconds
        if age_ns < 0:
            return "position timestamp is in the future"
        if age_ns > int(POSITION_MAX_AGE_S * 1e9):
            return f"position is stale ({age_ns / 1e9:.2f} s old; limit {POSITION_MAX_AGE_S:.1f} s)"
        if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return "position uncertainty is unknown"
        variance_east, variance_north = fix.position_covariance[0], fix.position_covariance[4]
        if not all(math.isfinite(value) and value >= 0.0 for value in (variance_east, variance_north)):
            return "position uncertainty is invalid"
        horizontal_std = math.sqrt(max(variance_east, variance_north))
        if horizontal_std > self._localization_max_horizontal_std_m:
            return (
                f"position uncertainty is too high ({horizontal_std:.2f} m; "
                f"limit {self._localization_max_horizontal_std_m:.2f} m)"
            )
        if check_tf:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._global_frame, self._robot_base_frame, Time(),
                    timeout=Duration(seconds=0.0),
                )
            except TransformException:
                return "localization transform is unavailable"
            tf_stamp = Time.from_msg(transform.header.stamp)
            tf_age_ns = (self._now() - tf_stamp).nanoseconds
            if tf_stamp.nanoseconds <= 0 or tf_age_ns < 0:
                return "localization transform timestamp is invalid"
            if tf_age_ns > int(POSITION_MAX_AGE_S * 1e9):
                return f"localization transform is stale ({tf_age_ns / 1e9:.2f} s old)"
        return ""
    def _abort(self, mission: _Mission, reason: str) -> None:
        if mission.abort_reason in (reason, OPERATOR_TOOK_OVER):
            return
        self.get_logger().warning(f"mission {mission.plan.id} aborting: {reason}")
        mission.abort_reason = reason
        mission.phase = Phase.ABORTING
        self._set_deadline(mission, STOP_TIMEOUT_S, STOP_OVERDUE)
    def _tick(self) -> None:
        self._cancel_navigation_after_restart()
        self._track_navigation()
        self._update_readiness()
        mission = self._mission
        if mission is None:
            return
        if mission.goal is not None and mission.goal.is_cancel_requested:
            self._abort(mission, CANCELLED_BY_MANAGER)
        try:
            self._advance(mission)
        except MissionError as error:
            self._finish(
                mission,
                MissionState.MISSION_FAILED,
                MissionState.COMMAND_FAILED,
                str(error),
                mission.goal.abort if mission.goal is not None else (lambda: None),
            )
    def _advance(self, mission: _Mission) -> None:
        if mission.deadline is not None and self._now() > mission.deadline:
            raise MissionError(mission.overdue)
        if mission.phase == Phase.NAVIGATING:
            self._navigate(mission)
        elif mission.phase == Phase.HOLDING:
            if self._now() >= mission.hold_until:
                self._complete_command(mission)
        elif mission.phase == Phase.SUSPENDING:
            if self._navigation_stopped(mission):
                self._pause(mission)
        elif mission.phase == Phase.ABORTING:
            if self._navigation_stopped(mission):
                self._end_abort(mission)
        elif mission.phase == Phase.RETURNING:
            self._return_home(mission)
    def _start_command(self, mission: _Mission) -> None:
        commands = mission.plan.commands
        while mission.index < len(commands) and commands[mission.index].kind in INSTANT_KINDS:
            command = commands[mission.index]
            self.get_logger().info(
                f"command {command.id} kind {command.kind} finishes at once on a ground robot"
            )
            self._set_status(mission, mission.index, MissionState.COMMAND_RUNNING)
            self._set_status(mission, mission.index, MissionState.COMMAND_FINISHED)
            mission.index += 1
        if mission.index == len(commands):
            self._finish(
                mission, MissionState.MISSION_FINISHED, None, "",
                mission.goal.succeed if mission.goal is not None else (lambda: None),
            )
            return
        mission.phase = Phase.NAVIGATING
        self._set_status(mission, mission.index, MissionState.COMMAND_RUNNING)
        self._publish_state(mission, MissionState.MISSION_RUNNING)
    def _navigate(self, mission: _Mission) -> None:
        if mission.navigation is None:
            if mission.backtrack_pending:
                self.get_logger().warning(
                    f"returning to last reached mission point {mission.last_reached_point} "
                    "before continuing"
                )
                mission.navigation = self._send_goal(mission, mission.last_reached_point)
                return
            if mission.retry_after is not None:
                if self._now() < mission.retry_after:
                    return
                mission.retry_after = None
                self.get_logger().warning(
                    f"retrying waypoint command "
                    f"{mission.plan.commands[mission.index].id} at the same target "
                    f"(attempt {mission.waypoint_retries + 1} of "
                    f"{self._waypoint_retry_limit + 1})"
                )
            mission.navigation = self._send_goal(mission, mission.point_of[mission.index])
            return
        status = self._navigation_status(mission.navigation)
        if status not in TERMINAL_GOAL_STATUSES:
            return
        staged = mission.navigation.staged
        mission.navigation = None
        if status == GoalStatus.STATUS_SUCCEEDED and staged:
            self.get_logger().info(
                "intermediate rolling-costmap goal reached; advancing toward mission destination"
            )
            return
        if mission.backtrack_pending:
            if status != GoalStatus.STATUS_SUCCEEDED:
                raise MissionError(
                    "failed to return to the last reached mission point before continuing"
                )
            mission.backtrack_pending = False
            mission.index = mission.resume_command_index
            mission.resume_command_index = None
            self.get_logger().info("backtrack complete; resuming mission at next planned target")
            self._start_command(mission)
            return
        command = mission.plan.commands[mission.index]
        if status != GoalStatus.STATUS_SUCCEEDED:
            if (
                command.kind == MissionCommand.WAYPOINT
                and status == GoalStatus.STATUS_ABORTED
                and mission.waypoint_retries < self._waypoint_retry_limit
            ):
                mission.waypoint_retries += 1
                mission.retry_after = self._now() + Duration(
                    seconds=self._waypoint_retry_delay_s
                )
                self.get_logger().warning(
                    f"Nav2 exhausted recovery for waypoint command {command.id}; "
                    f"retrying the same target after {self._waypoint_retry_delay_s:.1f} s "
                    f"({mission.waypoint_retries}/{self._waypoint_retry_limit} retries)"
                )
                return
            if (
                command.kind == MissionCommand.WAYPOINT
                and status == GoalStatus.STATUS_ABORTED
                and self._continue_after_waypoint_failure
            ):
                next_target = next(
                    (
                        index
                        for index in range(mission.index + 1, len(mission.plan.commands))
                        if mission.plan.commands[index].kind in POSITION_KINDS
                    ),
                    None,
                )
                if next_target is not None:
                    self._set_status(mission, mission.index, MissionState.COMMAND_FAILED)
                    for skipped in range(mission.index + 1, next_target):
                        if mission.plan.commands[skipped].kind in INSTANT_KINDS:
                            self._set_status(mission, skipped, MissionState.COMMAND_FINISHED)
                    self.get_logger().error(
                        f"waypoint command {command.id} failed after "
                        f"{mission.waypoint_retries + 1} attempts; marking it failed, "
                        f"backtracking to the last reached point, then continuing at "
                        f"command {mission.plan.commands[next_target].id}"
                    )
                    mission.waypoint_retries = 0
                    mission.retry_after = None
                    mission.resume_command_index = next_target
                    mission.backtrack_pending = True
                    self._publish_geojson(
                        mission,
                        self._waypoints_before(mission, next_target),
                        failed=True,
                    )
                    return
            attempts = mission.waypoint_retries + 1
            retry_note = (
                f" after {attempts} attempts"
                if command.kind == MissionCommand.WAYPOINT
                else ""
            )
            raise MissionError(
                f"navigate to command {command.id}{retry_note} failed: cause: "
                f"navigate_to_pose ended with status {status}; target was not skipped"
            )
        if command.hold_s > 0.0:
            mission.last_reached_point = mission.point_of[mission.index]
            mission.phase = Phase.HOLDING
            mission.hold_until = self._now() + Duration(seconds=command.hold_s)
            return
        self._complete_command(mission)
    def _complete_command(self, mission: _Mission) -> None:
        if mission.index in mission.point_of:
            mission.last_reached_point = mission.point_of[mission.index]
        mission.waypoint_retries = 0
        mission.retry_after = None
        self._set_status(mission, mission.index, MissionState.COMMAND_FINISHED)
        mission.index += 1
        self._publish_geojson(mission, self._waypoints_before(mission, mission.index))
        self._start_command(mission)
    def _pause(self, mission: _Mission) -> None:
        mission.deadline = None
        mission.phase = Phase.PAUSED
        self._set_status(mission, mission.index, MissionState.COMMAND_PAUSED)
        self._publish_state(mission, MissionState.MISSION_PAUSED)
        self._write_snapshot(mission)
    def _end_abort(self, mission: _Mission) -> None:
        if mission.abort_reason in (OPERATOR_TOOK_OVER, "mission cleared by operator"):
            self._finish_aborted(mission)
            return
        mission.deadline = None
        mission.phase = Phase.RETURNING
    def _return_home(self, mission: _Mission) -> None:
        if mission.navigation is None:
            self.get_logger().info("navigating home after abort")
            self._publish_geojson(
                mission,
                self._waypoints_before(mission, len(mission.plan.commands)),
                return_active=True,
            )
            mission.navigation = self._send_goal(mission, HOME_POINT)
            return
        if self._accepted_handle(mission.navigation) is not None:
            mission.navigation = None
            self._finish_aborted(mission)
    def _finish_aborted(self, mission: _Mission) -> None:
        cancelled = (
            mission.goal is not None
            and mission.goal.is_cancel_requested
            and mission.abort_reason != OPERATOR_TOOK_OVER
        )
        self._finish(
            mission,
            MissionState.MISSION_ABORTED,
            MissionState.COMMAND_ABORTED,
            mission.abort_reason,
            (mission.goal.canceled if cancelled else mission.goal.abort)
            if mission.goal is not None else (lambda: None),
        )
    def _finish(
        self,
        mission: _Mission,
        mission_status: int,
        command_status: int | None,
        reason: str,
        conclude,
    ) -> None:
        index = self._current(mission)
        if command_status is not None:
            self._set_status(mission, index, command_status)
            current = self._waypoints_before(mission, index)
            self._publish_geojson(
                mission,
                current,
                failed=current < len(mission.waypoints),
                mission_complete=mission_status == MissionState.MISSION_FINISHED,
            )
        elif mission_status == MissionState.MISSION_FINISHED:
            self._publish_geojson(
                mission,
                len(mission.waypoints),
                mission_complete=True,
            )
        self._last_failure_reason = reason if mission_status in (
            MissionState.MISSION_FAILED, MissionState.MISSION_ABORTED
        ) else ""
        self._publish_state(mission, mission_status)
        self._write_snapshot(mission, mission_status)
        if mission.goal is not None:
            conclude()
        result = ExecuteMission.Result()
        result.stamp = self._now().to_msg()
        result.status = mission_status
        result.command_id = mission.plan.commands[index].id
        result.reason = reason
        if not mission.result.done():
            mission.result.set_result(result)
        self._mission = None
        if self._clear_pending:
            self._clear_pending = False
            self._clear_visualization()
        message = f"mission {mission.plan.id} ended with status {mission_status}: {reason}"
        if mission_status == MissionState.MISSION_FINISHED:
            self.get_logger().info(f"mission {mission.plan.id} finished")
        elif mission_status == MissionState.MISSION_ABORTED:
            self.get_logger().warning(message)
        else:
            self.get_logger().error(message)
    def _send_goal(self, mission: _Mission, point: int) -> _Navigation:
        destination = mission.points[point]
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as error:
            raise MissionError(
                f"stage navigation goal failed: cause: no current transform from "
                f"{self._robot_base_frame} to {self._global_frame}: {error}"
            ) from error
        current = (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        if not self._path_client.server_is_ready():
            raise MissionError(
            "plan navigation goal failed: cause: compute_path_to_pose action is unavailable"
            )
        request = ComputePathToPose.Goal()
        request.start.header.frame_id = self._global_frame
        request.start.header.stamp = self._now().to_msg()
        request.start.pose.position.x = current[0]
        request.start.pose.position.y = current[1]
        request.start.pose.orientation.w = 1.0
        request.goal.header.frame_id = self._global_frame
        request.goal.header.stamp = self._now().to_msg()
        planner_destination, needs_staging = next_navigation_segment(
            current, destination, self._max_navigation_segment_m
        )
        request.goal.pose.position.x = planner_destination[0]
        request.goal.pose.position.y = planner_destination[1]
        yaw = mission.yaws[point]
        request.goal.pose.orientation.z = math.sin(yaw / 2.0)
        request.goal.pose.orientation.w = math.cos(yaw / 2.0)
        request.planner_id = "GridBased"
        request.use_start = True
        now = self._now()
        return _Navigation(
            send=None,
            answer_by=now + Duration(seconds=GOAL_RESPONSE_TIMEOUT_S),
            plan=self._path_client.send_goal_async(request),
            destination=destination,
            needs_staging=needs_staging,
            final_yaw=yaw,
        )
    def _accepted_handle(self, navigation: _Navigation):
        if navigation.send is None:
            if navigation.plan is None or not navigation.plan.done():
                if self._now() > navigation.answer_by:
                    raise MissionError(
                        "plan navigation goal failed: cause: compute_path_to_pose "
                        "did not answer within 10 s"
                    )
                return None
            planner_handle = navigation.plan.result()
            if not planner_handle.accepted:
                raise MissionError(
                    "plan navigation goal failed: cause: compute_path_to_pose rejected the request"
                )
            if navigation.plan_result is None:
                navigation.plan_result = planner_handle.get_result_async()
                navigation.answer_by = self._now() + Duration(
                    seconds=GOAL_RESPONSE_TIMEOUT_S
                )
                return None
            if not navigation.plan_result.done():
                if self._now() > navigation.answer_by:
                    raise MissionError(
                        "plan navigation goal failed: cause: compute_path_to_pose "
                        "did not return a path within 10 s"
                    )
                return None
            result = navigation.plan_result.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise MissionError(
                    "plan navigation goal failed: cause: compute_path_to_pose "
                    f"ended with status {result.status}"
                )
            path = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in result.result.path.poses
            ]
            if not path:
                raise MissionError(
                    "plan navigation goal failed: cause: planner returned an empty path"
                )
            if navigation.destination is None:
                raise MissionError("plan navigation goal failed: cause: missing destination")
            (x, y), path_requires_another_segment = next_path_segment(
                path, self._max_navigation_segment_m
            )
            navigation.staged = navigation.needs_staging or path_requires_another_segment
            yaw = (
                math.atan2(navigation.destination[1] - y, navigation.destination[0] - x)
                if navigation.staged
                else navigation.final_yaw
            )
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = self._global_frame
            goal.pose.header.stamp = self._now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            if navigation.staged:
                goal.behavior_tree = str(
                    Path(get_package_share_directory("terrascout_mission"))
                    / "config"
                    / "navigate_to_pose_intermediate.xml"
                )
            navigation.send = self._nav.send_goal_async(goal)
            navigation.answer_by = self._now() + Duration(seconds=GOAL_RESPONSE_TIMEOUT_S)
            navigation.plan = None
            navigation.plan_result = None
        if not navigation.send.done():
            if self._now() > navigation.answer_by:
                raise MissionError(
                    f"send navigation goal failed: cause: navigate_to_pose did not answer "
                    f"within {GOAL_RESPONSE_TIMEOUT_S} s"
                )
            return None
        handle = navigation.send.result()
        if not handle.accepted:
            raise MissionError("send navigation goal failed: cause: navigate_to_pose rejected it")
        return handle
    def _navigation_status(self, navigation: _Navigation) -> int:
        handle = self._accepted_handle(navigation)
        if handle is None:
            return GoalStatus.STATUS_UNKNOWN
        if navigation.result is None:
            navigation.result = handle.get_result_async()
        if navigation.result.done():
            return navigation.result.result().status
        return handle.status
    def _navigation_stopped(self, mission: _Mission) -> bool:
        navigation = mission.navigation
        if navigation is None:
            return True
        handle = self._accepted_handle(navigation)
        if handle is None:
            return False
        if self._navigation_status(navigation) in TERMINAL_GOAL_STATUSES:
            mission.navigation = None
            return True
        if navigation.cancel is None:
            navigation.cancel = handle.cancel_goal_async()
            return False
        if not navigation.cancel.done():
            return False
        code = navigation.cancel.result().return_code
        if code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
            mission.navigation = None
            return True
        if code != CancelGoal.Response.ERROR_NONE:
            raise MissionError(f"cancel navigation goal failed: cause: return code {code}")
        return False
    def _map_point(self, latitude: float, longitude: float) -> tuple[float, float]:
        datum = self._datum
        east, north, _ = llh_to_enu(
            latitude, longitude, 0.0, datum.latitude, datum.longitude, datum.altitude
        )
        return enu_to_map(east, north, math.radians(self._map_yaw_offset_deg))
    def _now(self) -> Time:
        return self.get_clock().now()
    def _set_deadline(self, mission: _Mission, seconds: float, overdue: str) -> None:
        mission.deadline = self._now() + Duration(seconds=seconds)
        mission.overdue = overdue
    def _current(self, mission: _Mission) -> int:
        return min(mission.index, len(mission.plan.commands) - 1)
    def _waypoints_before(self, mission: _Mission, index: int) -> int:
        return sum(
            1
            for command in mission.plan.commands[:index]
            if command.kind == MissionCommand.WAYPOINT
        )
    def _set_status(self, mission: _Mission, index: int, status: int) -> None:
        if mission.status[index] == status:
            return
        mission.status[index] = status
        feedback = ExecuteMission.Feedback()
        feedback.stamp = self._now().to_msg()
        feedback.command_id = mission.plan.commands[index].id
        feedback.status = status
        if mission.goal is not None:
            mission.goal.publish_feedback(feedback)
        self._write_snapshot(mission)
        mission_status = (
            MissionState.MISSION_PAUSED
            if mission.phase == Phase.PAUSED
            else MissionState.MISSION_RUNNING
        )
        self._publish_state(mission, mission_status)
    def _publish_state(self, mission: _Mission, mission_status: int) -> None:
        msg = MissionState()
        msg.mission_id = mission.plan.id
        msg.current_command_id = mission.plan.commands[self._current(mission)].id
        msg.command_ids = [command.id for command in mission.plan.commands]
        msg.command_status = list(mission.status)
        msg.mission_status = mission_status
        msg.failure_reason = self._last_failure_reason or mission.abort_reason
        self._publish(msg)
        self._write_snapshot(mission, mission_status)
    def _publish(self, msg: MissionState) -> None:
        msg.stamp = self._now().to_msg()
        msg.ready = self._ready
        msg.not_ready_reason = self._not_ready_reason
        self._last_state = msg
        self._state_pub.publish(msg)
    def _publish_geojson(
        self,
        mission: _Mission,
        current: int,
        failed: bool = False,
        return_active: bool = False,
        mission_complete: bool = False,
    ) -> None:
        if not return_active and mission.index < len(mission.plan.commands):
            return_active = (
                mission.plan.commands[mission.index].kind == MissionCommand.HOME
            )
        msg = GeoJSON()
        waypoint_statuses = []
        for command_index, command in enumerate(mission.plan.commands):
            if command.kind != MissionCommand.WAYPOINT:
                continue
            status = mission.status[command_index]
            waypoint_statuses.append({
                MissionState.COMMAND_FINISHED: "completed",
                MissionState.COMMAND_FAILED: "failed",
                MissionState.COMMAND_ABORTED: "failed",
                MissionState.COMMAND_RUNNING: "active",
                MissionState.COMMAND_PAUSED: "active",
            }.get(status, "pending"))
        msg.geojson = build_waypoints_geojson(
            mission.waypoints, current, failed=failed, start_position=mission.start_position,
            return_position={"lat": mission.plan.home.latitude, "lon": mission.plan.home.longitude},
            active=not failed, return_active=return_active, mission_complete=mission_complete,
            waypoint_statuses=waypoint_statuses,
        )
        self._geojson_pub.publish(msg)
def main() -> None:
    signals.init()
    node = GroundMissionServer()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
if __name__ == "__main__":
    sys.exit(main())
