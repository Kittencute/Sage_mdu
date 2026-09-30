import json
import math

from madum_bridge.protocol import AlarmCode, VehicleType
from madum_bridge.state_reporter import (
    AlarmTracker,
    build_state_vector,
    build_vehicle_summary,
    quaternion_to_euler,
)


def test_quaternion_to_euler_identity():
    roll, pitch, yaw = quaternion_to_euler(0.0, 0.0, 0.0, 1.0)
    assert abs(roll) < 1e-6
    assert abs(pitch) < 1e-6
    assert abs(yaw) < 1e-6


def test_quaternion_to_euler_90_yaw():
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    roll, pitch, yaw = quaternion_to_euler(0.0, 0.0, s, c)
    assert abs(yaw - math.pi / 2) < 1e-6


def test_build_state_vector():
    sv = build_state_vector(
        vehicle_id=2001,
        latitude=59.5,
        longitude=16.4,
        altitude=10.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        speed=1.5,
        speed_x=1.2,
        speed_y=0.9,
        speed_z=0.0,
        battery_pct=80.0,
        mission_id=-1,
        cur_command_id=-1,
        epoch_time=1716816286,
        high_precision_time=1716816286000,
    )
    assert sv.vehicleId == 2001
    assert sv.location.latitude == 59.5
    assert sv.battery == 80.0
    assert sv.speed == 1.5
    assert sv.speedX == 1.2
    assert sv.speedY == 0.9
    assert sv.speedZ == 0.0
    assert sv.highPrecisionTime == 1716816286000
    assert sv.gimballPitch is None


def test_build_vehicle_summary():
    equipments = [
        {"id": 1001, "name": "Front Camera", "type": 1, "status": 1},
    ]
    sv = build_state_vector(
        vehicle_id=2001,
        latitude=0.0,
        longitude=0.0,
        altitude=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        speed=0.0,
        speed_x=0.0,
        speed_y=0.0,
        speed_z=0.0,
        battery_pct=100.0,
        mission_id=-1,
        cur_command_id=-1,
        epoch_time=0,
        high_precision_time=0,
    )
    summary = build_vehicle_summary(
        vehicle_id=2001,
        vehicle_name="TerraSCout 1",
        max_speed=2.0,
        equipments=equipments,
        state_vector=sv,
    )
    assert summary.id == 2001
    assert summary.name == "TerraSCout 1"
    assert len(summary.equipments) == 1
    dumped = json.loads(summary.model_dump_json(exclude_none=True))
    assert "maxFlightTime" not in dumped


def test_alarm_tracker_raise_and_resolve():
    tracker = AlarmTracker(vehicle_id=2001)
    alarm = tracker.raise_alarm(AlarmCode.BATTERY_LOW, "Battery below 20%", epoch_time=100)
    assert alarm is not None
    assert alarm.id == 1
    assert alarm.alarmCode == AlarmCode.BATTERY_LOW
    assert tracker.is_active(AlarmCode.BATTERY_LOW)

    resolved = tracker.resolve_alarm(AlarmCode.BATTERY_LOW, epoch_time=200)
    assert resolved is not None
    assert resolved.resolvedAlarmId == 1
    assert not tracker.is_active(AlarmCode.BATTERY_LOW)


def test_alarm_tracker_no_duplicate():
    tracker = AlarmTracker(vehicle_id=2001)
    tracker.raise_alarm(AlarmCode.GPS_SIGNAL_LOST, "No fix", epoch_time=100)
    second = tracker.raise_alarm(AlarmCode.GPS_SIGNAL_LOST, "Still no fix", epoch_time=110)
    assert second is None


def test_alarm_tracker_resolve_nonexistent():
    tracker = AlarmTracker(vehicle_id=2001)
    resolved = tracker.resolve_alarm(AlarmCode.GPS_SIGNAL_LOST, epoch_time=100)
    assert resolved is None


def test_build_vehicle_summary_includes_vehicle_type():
    equipments = [{"id": 1001, "name": "Front Camera", "type": 1, "status": 1}]
    sv = build_state_vector(
        vehicle_id=2001,
        latitude=0.0,
        longitude=0.0,
        altitude=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        speed=0.0,
        speed_x=0.0,
        speed_y=0.0,
        speed_z=0.0,
        battery_pct=100.0,
        mission_id=-1,
        cur_command_id=-1,
        epoch_time=0,
        high_precision_time=0,
    )
    summary = build_vehicle_summary(
        vehicle_id=2001,
        vehicle_name="TerraSCout 1",
        max_speed=2.0,
        equipments=equipments,
        state_vector=sv,
        vehicle_type=VehicleType.UGV,
    )
    assert summary.type == VehicleType.UGV
    assert int(summary.type) == 2


def test_alarm_tracker_rejects_disallowed_code():
    tracker = AlarmTracker(vehicle_id=2001, allowed_codes={AlarmCode.BATTERY_LOW})
    rejected = tracker.raise_alarm(AlarmCode.MAX_ALT_EXCEEDED, "should not fire", epoch_time=10)
    assert rejected is None
    assert not tracker.is_active(AlarmCode.MAX_ALT_EXCEEDED)
    allowed = tracker.raise_alarm(AlarmCode.BATTERY_LOW, "ok", epoch_time=11)
    assert allowed is not None


def test_alarm_tracker_permissive_when_no_whitelist():
    tracker = AlarmTracker(vehicle_id=2001)
    for code in (AlarmCode.BATTERY_LOW, AlarmCode.GPS_SIGNAL_LOST):
        alarm = tracker.raise_alarm(code, "ok", epoch_time=0)
        assert alarm is not None
