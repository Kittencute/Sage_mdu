#!/usr/bin/env python3


from __future__ import annotations

import contextlib
import json
import math
import threading

import rclpy
from fleet_interfaces.action import ExecuteMission
from fleet_interfaces.msg import MissionPlan, MissionState
from fleet_interfaces.srv import ControlMission
from nav_msgs.msg import Odometry
from pydantic import ValidationError
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import String

from fleet_common import signals
from fleet_common.params import declare
from fleet_common.qos import latched_qos
from madum_bridge.mission_executor import MissionExecutor
from madum_bridge.mission_executor import MissionState as AdapterState
from madum_bridge.mission_plan import (
    plan_from_mission,
    status_code_for,
    terminal_status_code_for,
)
from madum_bridge.protocol import (
    AlarmCode,
    MissionAction,
    MissionActionCode,
    MissionFailure,
    MissionFullAction,
    MissionSend,
    MissionUpdate,
    StatusCode,
)
from madum_bridge.state_reporter import AlarmTracker, build_state_vector, build_vehicle_summary

_ALARM_BATTERY_LOW = int(AlarmCode.BATTERY_LOW)
_ALARM_GPS_LOST = int(AlarmCode.GPS_SIGNAL_LOST)
_NO_ID = -1
_RETAINED_EXPIRY_PERIOD_S = 1.0
_STATE_VECTOR_LIFETIME_S = 20.0
_MISSION_UPDATE_LIFETIME_S = 3600.0
_ALARM_LIFETIME_S = 1800.0


class MadumBridge(Node):
    def __init__(self):
        super().__init__("madum_bridge")
        self._lock = threading.RLock()
        self.setup_parameters()

        self._latest_llh: NavSatFix | None = None
        self._latest_odometry: Odometry | None = None
        self._batteries: dict[str, BatteryState] = {}
        self._mission_executor = MissionExecutor(persist_path=self.completed_mission_state_file)
        self._alarm_tracker = AlarmTracker(self.vehicle_id, self.allowed_alarm_codes)
        self._goal_handle = None
        self._goal_pending = False
        self._cancel_requested = False
        self._replacement: tuple[MissionSend, MissionPlan] | None = None
        self._group = ReentrantCallbackGroup()
        self._mission_advance_error = None
        self._mission_advance_error_guard = self.create_guard_condition(
            self._raise_mission_advance_error
        )

        self.setup_publishers()
        self.setup_subscribers()
        self._mission_client = ActionClient(
            self, ExecuteMission, "execute_mission", callback_group=self._group
        )
        self._control_client = self.create_client(
            ControlMission, "control_mission", callback_group=self._group
        )
        self.setup_timers()

    def setup_parameters(self):
        self.vehicle_id = declare(
            self,
            "vehicle_id",
            Parameter.Type.INTEGER,
            "MADUM identifier of this vehicle",
            constraints="any MADUM vehicle id",
        )
        self.vehicle_name = declare(
            self, "vehicle_name", Parameter.Type.STRING, "Name of this vehicle as reported to MADUM"
        )
        self.completed_mission_state_file = declare(
            self,
            "completed_mission_state_file",
            Parameter.Type.STRING,
            "File that remembers the last completed mission across restarts",
            constraints="a writable file path, not empty",
        )
        if not self.completed_mission_state_file:
            raise RuntimeError(
                "load MADUM settings failed: cause: completed_mission_state_file is required"
            )
        self.state_vector_interval = declare(
            self,
            "state_vector_interval",
            Parameter.Type.DOUBLE,
            "Time between published state vectors, in seconds",
            0.01,
            60.0,
        )
        self.battery_low_threshold = declare(
            self,
            "battery_low_threshold",
            Parameter.Type.DOUBLE,
            "Battery charge below which the low battery alarm is raised, in percent",
            0.0,
            100.0,
        )
        self.sensor_timeout = declare(
            self,
            "sensor_timeout",
            Parameter.Type.DOUBLE,
            "Age after which a position or odometry reading counts as lost, in seconds",
            0.1,
            600.0,
        )
        self.battery_timeout = declare(
            self,
            "battery_timeout",
            Parameter.Type.DOUBLE,
            "Age after which a battery reading counts as lost, in seconds",
            0.1,
            600.0,
        )
        self.vehicle_max_speed = declare(
            self,
            "vehicle_max_speed",
            Parameter.Type.DOUBLE,
            "Highest speed of this vehicle reported to MADUM, in metres per second",
            0.0,
            50.0,
        )
        self.vehicle_type = declare(
            self,
            "vehicle_type",
            Parameter.Type.INTEGER,
            "MADUM vehicle type, 1 for an aerial vehicle and 2 for a ground vehicle",
            1,
            2,
        )
        self.equipments = json.loads(
            declare(
                self,
                "equipments_json",
                Parameter.Type.STRING,
                "Equipment list of this vehicle reported to MADUM",
                constraints="a JSON array of MADUM equipment objects",
            )
        )
        codes = list(
            declare(
                self,
                "allowed_alarm_codes",
                Parameter.Type.INTEGER_ARRAY,
                "MADUM alarm codes this vehicle may raise; empty allows every code",
                constraints="MADUM alarm codes; an empty list allows all",
            )
        )
        self.allowed_alarm_codes = set(codes) if codes else None
        mflight = declare(
            self,
            "max_flight_time_minutes",
            Parameter.Type.INTEGER,
            "Longest flight time reported in the vehicle summary, in minutes; 0 leaves it out",
            0,
            1440,
        )
        self.max_flight_time_minutes = mflight if mflight > 0 else None

    def setup_publishers(self):
        self._pub_state_vector = self.create_publisher(String, "vh/stateVector", 10)
        self._pub_summary = self.create_publisher(String, "vh/summary", latched_qos())
        self._pub_mission_update = self.create_publisher(String, "vh/missionUpdate", 10)
        self._pub_alarm = self.create_publisher(String, "vh/alarm", 10)
        self._pub_alarm_resolved = self.create_publisher(String, "vh/alarmResolved", 10)
        self._retained_lifetimes = {
            self._pub_state_vector: Duration(seconds=_STATE_VECTOR_LIFETIME_S),
            self._pub_summary: None,
            self._pub_mission_update: Duration(seconds=_MISSION_UPDATE_LIFETIME_S),
            self._pub_alarm: Duration(seconds=_ALARM_LIFETIME_S),
            self._pub_alarm_resolved: Duration(seconds=_ALARM_LIFETIME_S),
        }
        start = self.get_clock().now()
        self._retained_clear_at = {publisher: start for publisher in self._retained_lifetimes}

    def setup_subscribers(self):
        self.create_subscription(Odometry, "odometry", self._odometry_cb, 10)
        self.create_subscription(NavSatFix, "position", self._llh_cb, 10)
        self.create_subscription(BatteryState, "battery_state", self._battery_cb, 10)
        self.create_subscription(String, "mw/missionSend", self._mission_send_cb, 10)
        self.create_subscription(String, "mw/mission", self._mission_action_cb, 10)
        self.create_subscription(String, "mw/missionFull", self._mission_full_cb, 10)

    def setup_timers(self):
        self.create_timer(self.state_vector_interval, self._state_vector_timer_cb)
        self.create_timer(self.sensor_timeout, self._check_gps_timeout)
        self.create_timer(_RETAINED_EXPIRY_PERIOD_S, self._expire_retained)

    def _publish_retained(self, publisher, model):
        with self._lock:
            publisher.publish(String(data=model.model_dump_json(exclude_none=True)))
            lifetime = self._retained_lifetimes[publisher]
            if lifetime is None:
                self._retained_clear_at.pop(publisher, None)
            else:
                self._retained_clear_at[publisher] = self.get_clock().now() + lifetime

    def _expire_retained(self):
        now = self.get_clock().now()
        with self._lock:
            for publisher, clear_at in list(self._retained_clear_at.items()):
                if clear_at <= now and publisher.get_subscription_count() > 0:
                    publisher.publish(String(data=""))
                    del self._retained_clear_at[publisher]

    def publish_vehicle_summary(self):
        with self._lock:
            sv = self._build_current_state_vector()
            if sv is None:
                return
            summary = build_vehicle_summary(
                vehicle_id=self.vehicle_id,
                vehicle_name=self.vehicle_name,
                max_speed=self.vehicle_max_speed,
                equipments=self.equipments,
                state_vector=sv,
                vehicle_type=self.vehicle_type,
            )
            if self.max_flight_time_minutes is not None:
                summary.maxFlightTime = self.max_flight_time_minutes
            self._publish_retained(self._pub_summary, summary)
        self.get_logger().info("Published vehicle summary")

    def _is_fresh(self, stamp, timeout_s: float) -> bool:
        return self.get_clock().now() - Time.from_msg(stamp) <= Duration(seconds=timeout_s)

    def _battery_percentage(self) -> float | None:
        if not self._batteries:
            return None
        for battery in self._batteries.values():
            if (
                not battery.present
                or not math.isfinite(battery.percentage)
                or not self._is_fresh(battery.header.stamp, self.battery_timeout)
            ):
                return None
        return min(battery.percentage for battery in self._batteries.values()) * 100.0

    def _build_current_state_vector(self):
        llh = self._latest_llh
        if llh is None or not self._is_fresh(llh.header.stamp, self.sensor_timeout):
            return None
        qx = qy = qz = qw = None
        vx = vy = vz = speed = None
        odometry = self._latest_odometry
        if odometry is not None and self._is_fresh(odometry.header.stamp, self.sensor_timeout):
            o = odometry.pose.pose.orientation
            qx, qy, qz, qw = o.x, o.y, o.z, o.w
            vx = odometry.twist.twist.linear.x
            vy = odometry.twist.twist.linear.y
            vz = odometry.twist.twist.linear.z
            speed = math.sqrt(vx * vx + vy * vy)
        mission_id = self._mission_executor.mission_id
        cmd = self._mission_executor.current_command
        stamp = llh.header.stamp
        return build_state_vector(
            vehicle_id=self.vehicle_id,
            latitude=llh.latitude,
            longitude=llh.longitude,
            altitude=llh.altitude,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            speed=speed,
            speed_x=vx,
            speed_y=vy,
            speed_z=vz,
            battery_pct=self._battery_percentage(),
            mission_id=_NO_ID if mission_id is None else mission_id,
            cur_command_id=_NO_ID if cmd is None else cmd.id,
            epoch_time=stamp.sec,
            high_precision_time=stamp.sec * 1000 + stamp.nanosec // 1_000_000,
        )

    def _state_vector_timer_cb(self):
        with self._lock:
            sv = self._build_current_state_vector()
            if sv is not None:
                self._publish_retained(self._pub_state_vector, sv)

    def _epoch_now(self) -> int:
        return self.get_clock().now().seconds_nanoseconds()[0]

    def _raise_alarm(self, code: int, message: str):
        if self._alarm_tracker.is_active(code):
            return
        alarm = self._alarm_tracker.raise_alarm(code, message, self._epoch_now())
        if alarm:
            self._publish_retained(self._pub_alarm, alarm)

    def _resolve_alarm(self, code: int):
        if not self._alarm_tracker.is_active(code):
            return
        resolved = self._alarm_tracker.resolve_alarm(code, self._epoch_now())
        if resolved:
            self._publish_retained(self._pub_alarm_resolved, resolved)

    def _odometry_cb(self, msg: Odometry):
        with self._lock:
            self._latest_odometry = msg

    def _llh_cb(self, msg: NavSatFix):
        with self._lock:
            if msg.status.status < NavSatStatus.STATUS_FIX:
                self._raise_alarm(_ALARM_GPS_LOST, "GPS signal lost")
                return
            first_fix = self._latest_llh is None
            self._latest_llh = msg
            self._resolve_alarm(_ALARM_GPS_LOST)
            if first_fix:
                self.publish_vehicle_summary()

    def _battery_cb(self, msg: BatteryState):
        with self._lock:
            self._batteries[msg.location] = msg
            pct = self._battery_percentage()
            if pct is None:
                return
            if pct < self.battery_low_threshold:
                self._raise_alarm(_ALARM_BATTERY_LOW, f"Battery low: {pct:.0f}%")
            else:
                self._resolve_alarm(_ALARM_BATTERY_LOW)

    def _check_gps_timeout(self):
        with self._lock:
            llh = self._latest_llh
            if llh is not None and not self._is_fresh(llh.header.stamp, self.sensor_timeout):
                self._raise_alarm(_ALARM_GPS_LOST, "GPS signal lost")

    def _reject_mission_send(self, data: str, exc: Exception):
        mission_id = _NO_ID
        with contextlib.suppress(ValueError):
            raw = json.loads(data)
            if isinstance(raw, dict) and type(raw.get("id")) is int:
                mission_id = raw["id"]
        reason = f"parse missionSend failed: cause: {exc}"
        self.get_logger().error(reason)
        self._publish_failure(_NO_ID, mission_id, StatusCode.FAILED, reason)

    def _mission_send_cb(self, msg: String):
        try:
            mission = MissionSend.model_validate_json(msg.data)
        except ValidationError as exc:
            self._reject_mission_send(msg.data, exc)
            return
        if mission.vehicleId != self.vehicle_id:
            self._publish_first_command_failed(
                mission,
                f"accept mission {mission.id} failed: cause: vehicle id {mission.vehicleId} "
                f"is not {self.vehicle_id}",
            )
            return
        try:
            plan = plan_from_mission(mission, self.vehicle_type)
        except ValueError as exc:
            self._publish_first_command_failed(
                mission, f"translate mission {mission.id} failed: cause: {exc}"
            )
            return

        with self._lock:
            if self._mission_executor.is_stale_mission(mission.id):
                self.get_logger().warning(f"Mission {mission.id} already ended; ignoring")
                return
            queued = self._replacement
            if self._mission_executor.mission_id == mission.id or (
                queued is not None and queued[0].id == mission.id
            ):
                self.get_logger().info(
                    f"Mission {mission.id} already received; ignoring the repeat"
                )
                return
            if self._goal_handle is not None or self._goal_pending:
                self.get_logger().warning(
                    f"Cancelling mission {self._mission_executor.mission_id} "
                    f"(state={self._mission_executor.state.value}) for new mission {mission.id}"
                )
                if queued is not None:
                    self._publish_first_command_failed(
                        queued[0],
                        f"start mission {queued[0].id} failed: cause: replaced by mission {mission.id}",
                    )
                self._replacement = (mission, plan)
                self._cancel_goal()
                return
            self._start_mission(mission, plan)

    def _start_mission(self, mission: MissionSend, plan: MissionPlan):
        if not self._mission_client.server_is_ready():
            self._publish_first_command_failed(
                mission,
                f"start mission {mission.id} failed: cause: execute_mission server unavailable",
            )
            return
        if not self._mission_executor.start_mission(mission):
            self._publish_first_command_failed(
                mission,
                f"start mission {mission.id} failed: cause: mission "
                f"{self._mission_executor.mission_id} is {self._mission_executor.state.value}",
            )
            return
        goal = ExecuteMission.Goal()
        goal.plan = plan
        self._goal_pending = True
        self._cancel_requested = False
        future = self._mission_client.send_goal_async(
            goal, feedback_callback=lambda message: self._on_feedback(message, mission.id)
        )
        future.add_done_callback(lambda f: self._on_goal_response(f, mission.id))
        self.get_logger().info(f"Mission {mission.id} sent with {len(mission.commands)} command(s)")

    def _publish_failure(
        self, command_id: int, mission_id: int, status_code: StatusCode, reason: str
    ):
        failure = MissionFailure(
            commandId=command_id,
            missionId=mission_id,
            vehicleId=self.vehicle_id,
            statusCode=status_code,
            reason=reason,
        )
        self._publish_retained(self._pub_mission_update, failure)

    def _publish_first_command_failed(self, mission: MissionSend, reason: str):
        self.get_logger().error(reason)
        command_id = mission.commands[0].id if mission.commands else _NO_ID
        self._publish_failure(command_id, mission.id, StatusCode.FAILED, reason)

    def _on_goal_response(self, future, mission_id: int):
        with self._lock:
            self._goal_pending = False
            reason = f"start mission {mission_id} failed: cause: mission server rejected the goal"
            try:
                handle = future.result()
            except Exception as exc:
                reason = f"send mission {mission_id} failed: cause: {exc}"
                handle = None
            if handle is None or not handle.accepted:
                mission = self._mission_executor.mission
                assert mission is not None and mission.id == mission_id
                self._publish_first_command_failed(mission, reason)
                self._mission_executor.abort()
                self._start_replacement()
                return
            self._goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self._on_result(f, mission_id))
            if self._cancel_requested or self._replacement is not None:
                self._cancel_goal()

    def _on_feedback(self, message, mission_id: int):
        feedback = message.feedback
        with self._lock:
            code = status_code_for(feedback.status)
            if code is None:
                return
            self._publish_mission_update(feedback.command_id, mission_id, code)
            if self._mission_executor.mission_id != mission_id:
                return
            current = self._mission_executor.current_command
            if (
                code == StatusCode.FINISHED
                and current is not None
                and current.id == feedback.command_id
            ):
                self._advance()

    def _advance(self):
        try:
            self._mission_executor.advance()
        except RuntimeError as exc:
            self._mission_advance_error = exc
            self.get_logger().fatal(f"advance mission failed: cause: {exc}")
            self._mission_advance_error_guard.trigger()

    def _on_result(self, future, mission_id: int):
        with self._lock:
            self._goal_handle = None
            current = self._mission_executor.current_command
            try:
                result = future.result().result
                command_id, status, reason = result.command_id, result.status, result.reason
            except Exception as exc:
                command_id = _NO_ID if current is None else current.id
                status = MissionState.MISSION_FAILED
                reason = f"read mission {mission_id} result failed: cause: {exc}"
            code = terminal_status_code_for(status)
            if code is None:
                code = StatusCode.FAILED
                reason = f"read mission {mission_id} result failed: cause: unknown status {status}"
            if code == StatusCode.FINISHED:
                self._publish_mission_update(command_id, mission_id, code)
            else:
                self.get_logger().error(f"mission {mission_id} ended {code.name}: {reason}")
                self._publish_failure(command_id, mission_id, code, reason)
            if self._mission_executor.mission_id == mission_id:
                if code == StatusCode.FINISHED:
                    while self._mission_executor.state == AdapterState.EXECUTING:
                        if not self._mission_executor.advance():
                            break
                    self.get_logger().info(f"Mission {mission_id} complete")
                else:
                    self._mission_executor.abort()
            self._start_replacement()

    def _start_replacement(self):
        replacement = self._replacement
        self._replacement = None
        if replacement is not None:
            self._start_mission(*replacement)

    def _raise_mission_advance_error(self):
        error = self._mission_advance_error
        if error is None:
            raise RuntimeError(
                "raise mission advance failure failed: cause: "
                "guard condition triggered without a stored error"
            )
        raise RuntimeError(f"advance mission failed: cause: {error}") from error

    def _mission_action_cb(self, msg: String):
        try:
            action = MissionAction.model_validate_json(msg.data)
        except ValidationError as exc:
            self.get_logger().error(f"parse mission action failed: cause: {exc}")
            return
        if action.vehicleId != self.vehicle_id:
            self.get_logger().error(
                f"accept mission action failed: cause: vehicle id {action.vehicleId} "
                f"is not {self.vehicle_id}"
            )
            return
        with self._lock:
            if action.missionId is None:
                action.missionId = self._mission_executor.mission_id
            if action.missionId is None:
                self.get_logger().warning("Mission action received but no active mission")
                return
            self._handle_mission_action(action.missionId, action.action)

    def _mission_full_cb(self, msg: String):
        try:
            action = MissionFullAction.model_validate_json(msg.data)
        except ValidationError as exc:
            self.get_logger().error(f"parse missionFull action failed: cause: {exc}")
            return
        with self._lock:
            self._handle_mission_action(action.missionId, action.action)

    def _handle_mission_action(self, mission_id: int, action_code: int):
        if self._mission_executor.mission_id is None:
            self.get_logger().warning("Mission action received but no active mission")
            return
        if self._mission_executor.mission_id != mission_id:
            self.get_logger().warning(
                f"Mission action for {mission_id} does not match active {self._mission_executor.mission_id}"
            )
            return

        if action_code == MissionActionCode.SUSPEND:
            self.get_logger().info(f"Suspending mission {mission_id}")
            self._control(ControlMission.Request.SUSPEND, mission_id)
        elif action_code == MissionActionCode.RESUME:
            self.get_logger().info(f"Resuming mission {mission_id}")
            self._control(ControlMission.Request.RESUME, mission_id)
        elif action_code == MissionActionCode.ABORT:
            self.get_logger().info(f"Aborting mission {mission_id}")
            self._cancel_goal()

    def _control(self, request_code: int, mission_id: int):
        if not self._control_client.service_is_ready():
            self._publish_control_refused(
                request_code, mission_id, "control_mission service unavailable"
            )
            return
        request = ControlMission.Request()
        request.request = request_code
        future = self._control_client.call_async(request)
        future.add_done_callback(lambda f: self._on_control_response(f, request_code, mission_id))

    def _on_control_response(self, future, request_code: int, mission_id: int):
        with self._lock:
            try:
                response = future.result()
            except Exception as exc:
                self._publish_control_refused(request_code, mission_id, str(exc))
                return
            if not response.accepted:
                self._publish_control_refused(request_code, mission_id, response.reason)
                return
            if self._mission_executor.mission_id != mission_id:
                return
            if request_code == ControlMission.Request.SUSPEND:
                self._mission_executor.suspend()
            else:
                self._mission_executor.resume()

    def _publish_control_refused(self, request_code: int, mission_id: int, cause: str):
        reason = f"control mission {mission_id} with request {request_code} failed: cause: {cause}"
        self.get_logger().error(reason)
        if self._mission_executor.mission_id != mission_id:
            return
        current = self._mission_executor.current_command
        status = (
            StatusCode.STOPPED
            if self._mission_executor.state == AdapterState.SUSPENDED
            else StatusCode.RUNNING
        )
        self._publish_failure(_NO_ID if current is None else current.id, mission_id, status, reason)

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        elif self._goal_pending:
            self._cancel_requested = True

    def _publish_mission_update(self, command_id: int, mission_id: int, status_code: StatusCode):
        update = MissionUpdate(
            commandId=command_id,
            missionId=mission_id,
            vehicleId=self.vehicle_id,
            statusCode=status_code,
        )
        self._publish_retained(self._pub_mission_update, update)


def main():
    signals.init()
    node = MadumBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        with contextlib.suppress(Exception):
            node.destroy_node()
        with contextlib.suppress(Exception):
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
