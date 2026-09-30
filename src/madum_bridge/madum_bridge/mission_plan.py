from __future__ import annotations

import math

from fleet_interfaces.msg import MissionCommand, MissionPlan, MissionState
from geographic_msgs.msg import GeoPoint

from madum_bridge.command_handlers import (
    parse_camera_image_params,
    parse_nav_waypoint_params,
    parse_video_start_params,
    parse_video_stop_params,
)
from madum_bridge.protocol import Command, CommandType, MissionSend, StatusCode, VehicleType

_KINDS = {
    CommandType.NAV_TAKEOFF: MissionCommand.TAKEOFF,
    CommandType.NAV_LAND: MissionCommand.LAND,
    CommandType.NAV_WAYPOINT: MissionCommand.WAYPOINT,
    CommandType.NAV_HOME: MissionCommand.HOME,
    CommandType.CAMERA_IMAGE: MissionCommand.CAMERA_IMAGE,
    CommandType.VIDEO_START_CAPTURE: MissionCommand.VIDEO_START,
    CommandType.VIDEO_STOP_CAPTURE: MissionCommand.VIDEO_STOP,
}

_STATUS_CODES = {
    MissionState.COMMAND_RUNNING: StatusCode.RUNNING,
    MissionState.COMMAND_PAUSED: StatusCode.STOPPED,
    MissionState.COMMAND_FINISHED: StatusCode.FINISHED,
    MissionState.COMMAND_FAILED: StatusCode.FAILED,
    MissionState.COMMAND_ABORTED: StatusCode.ABORTED,
}

_TERMINAL_STATUS_CODES = {
    MissionState.MISSION_FINISHED: StatusCode.FINISHED,
    MissionState.MISSION_FAILED: StatusCode.FAILED,
    MissionState.MISSION_ABORTED: StatusCode.ABORTED,
}


_INT32_MIN = -(2**31)
_INT32_MAX = 2**31 - 1


def _need(params: list[float], count: int, name: str) -> None:
    if len(params) < count:
        raise ValueError(f"{name} requires {count} params, got {len(params)}")


def _require_int32(value: int, name: str) -> None:
    if not _INT32_MIN <= value <= _INT32_MAX:
        raise ValueError(f"{name} {value} is outside the int32 range")


def _require_finite(values: list[float], name: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} {values} holds a value that is not finite")


def _require_coordinates(latitude: float, longitude: float, name: str) -> None:
    if not -90.0 <= latitude <= 90.0:
        raise ValueError(f"{name} latitude {latitude} is outside [-90, 90]")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError(f"{name} longitude {longitude} is outside [-180, 180]")


def _require_altitude_agl(altitude: float, name: str) -> None:
    if not altitude >= 0.0:
        raise ValueError(f"{name} altitude {altitude} is below ground level")


def plan_from_mission(mission: MissionSend, vehicle_type: VehicleType) -> MissionPlan:
    home = mission.homeLocation
    _require_int32(mission.id, "mission id")
    _require_finite([home.latitude, home.longitude, home.altitude], "home location")
    _require_coordinates(home.latitude, home.longitude, "home location")
    plan = MissionPlan()
    plan.id = mission.id
    plan.home = GeoPoint(
        latitude=mission.homeLocation.latitude,
        longitude=mission.homeLocation.longitude,
        altitude=mission.homeLocation.altitude,
    )
    plan.commands = [_command(command, mission, vehicle_type) for command in mission.commands]
    return plan


def _command(command: Command, mission: MissionSend, vehicle_type: VehicleType) -> MissionCommand:
    name = f"command {command.id}"
    _require_int32(command.id, "command id")
    _require_finite(command.params, f"{name} params")
    out = MissionCommand()
    out.id = command.id
    out.kind = _KINDS[command.commandType]
    out.yaw_deg = math.nan
    out.parameters = [float(value) for value in command.params]
    params = command.params
    if command.commandType == CommandType.NAV_TAKEOFF:
        _need(params, 5, "NAV_TAKEOFF")
        out.yaw_deg = params[1]
        out.latitude = params[2]
        out.longitude = params[3]
        out.altitude_agl = params[4]
        _require_coordinates(out.latitude, out.longitude, name)
        _require_altitude_agl(out.altitude_agl, name)
    elif command.commandType == CommandType.NAV_WAYPOINT:
        if vehicle_type == VehicleType.UAV:
            _need(params, 7, "NAV_WAYPOINT on a UAV")
        waypoint = parse_nav_waypoint_params(params)
        out.hold_s = float(waypoint["hold_time_s"])
        out.acceptance_radius_m = float(waypoint["acceptance_radius_m"])
        out.yaw_deg = float(waypoint["desired_yaw_deg"])
        out.latitude = float(waypoint["latitude"])
        out.longitude = float(waypoint["longitude"])
        out.altitude_agl = params[6] if len(params) > 6 else 0.0
        _require_coordinates(out.latitude, out.longitude, name)
        _require_altitude_agl(out.altitude_agl, name)
    elif command.commandType == CommandType.NAV_LAND:
        _need(params, 5, "NAV_LAND")
        out.yaw_deg = params[2]
        out.latitude = params[3]
        out.longitude = params[4]
        out.altitude_agl = params[5] if len(params) > 5 else 0.0
        _require_coordinates(out.latitude, out.longitude, name)
        _require_altitude_agl(out.altitude_agl, name)
    elif command.commandType == CommandType.NAV_HOME:
        out.latitude = mission.homeLocation.latitude
        out.longitude = mission.homeLocation.longitude
    elif command.commandType == CommandType.CAMERA_IMAGE:
        parse_camera_image_params(params)
    elif command.commandType == CommandType.VIDEO_START_CAPTURE:
        parse_video_start_params(params)
    elif command.commandType == CommandType.VIDEO_STOP_CAPTURE:
        parse_video_stop_params(params)
    return out


def status_code_for(status: int) -> StatusCode | None:
    return _STATUS_CODES.get(status)


def terminal_status_code_for(mission_status: int) -> StatusCode | None:
    return _TERMINAL_STATUS_CODES.get(mission_status)
