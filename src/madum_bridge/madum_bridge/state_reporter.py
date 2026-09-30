from __future__ import annotations

import math

from madum_bridge.protocol import (
    Alarm,
    AlarmCode,
    AlarmResolved,
    Equipment,
    Position,
    StateVector,
    VehicleSummary,
    VehicleType,
)


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def build_state_vector(
    vehicle_id: int,
    latitude: float,
    longitude: float,
    altitude: float,
    qx: float | None,
    qy: float | None,
    qz: float | None,
    qw: float | None,
    speed: float | None,
    speed_x: float | None,
    speed_y: float | None,
    speed_z: float | None,
    battery_pct: float | None,
    mission_id: int,
    cur_command_id: int,
    epoch_time: int,
    high_precision_time: int,
) -> StateVector:
    roll = pitch = yaw = None
    if None not in (qx, qy, qz, qw):
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
    return StateVector(
        vehicleId=vehicle_id,
        missionId=mission_id,
        curCommandId=cur_command_id,
        location=Position(latitude=latitude, longitude=longitude, altitude=altitude),
        yaw=yaw,
        pitch=pitch,
        roll=roll,
        battery=battery_pct,
        speed=speed,
        speedX=speed_x,
        speedY=speed_y,
        speedZ=speed_z,
        time=epoch_time,
        highPrecisionTime=high_precision_time,
    )


def build_vehicle_summary(
    vehicle_id: int,
    vehicle_name: str,
    max_speed: float,
    equipments: list[dict[str, object]],
    state_vector: StateVector,
    vehicle_type: VehicleType = VehicleType.UGV,
) -> VehicleSummary:
    return VehicleSummary(
        id=vehicle_id,
        name=vehicle_name,
        maxSpeed=max_speed,
        equipments=[Equipment(**e) for e in equipments],  # type: ignore[arg-type]
        stateVector=state_vector,
        type=vehicle_type,
    )


class AlarmTracker:
    def __init__(self, vehicle_id: int, allowed_codes: set[int] | None = None):
        self._vehicle_id = vehicle_id
        self._next_id = 1
        self._active: dict[int, Alarm] = {}
        self._allowed_codes = allowed_codes

    def is_active(self, alarm_code: int | AlarmCode) -> bool:
        return alarm_code in self._active

    def raise_alarm(
        self,
        alarm_code: int | AlarmCode,
        message: str,
        epoch_time: int,
        resource_id: int = 0,
    ) -> Alarm | None:
        if self._allowed_codes is not None and int(alarm_code) not in self._allowed_codes:
            return None
        if alarm_code in self._active:
            return None
        alarm = Alarm(
            id=self._next_id,
            vehicleId=self._vehicle_id,
            alarmCode=AlarmCode(alarm_code),
            resourceId=resource_id,
            message=message,
            time=epoch_time,
        )
        self._next_id += 1
        self._active[alarm_code] = alarm
        return alarm

    def resolve_alarm(self, alarm_code: int | AlarmCode, epoch_time: int) -> AlarmResolved | None:
        alarm = self._active.pop(alarm_code, None)
        if alarm is None:
            return None
        resolved = AlarmResolved(
            id=self._next_id,
            vehicleId=self._vehicle_id,
            resolvedAlarmId=alarm.id,
            time=epoch_time,
        )
        self._next_id += 1
        return resolved
