import json

from madum_bridge.protocol import (
    Alarm,
    AlarmCode,
    AlarmResolved,
    Command,
    CommandType,
    EquipmentType,
    MissionAction,
    MissionActionCode,
    MissionFullAction,
    MissionSend,
    MissionUpdate,
    StateVector,
    StatusCode,
    VehicleSummary,
    VehicleType,
)


def test_state_vector_roundtrip():
    raw = {
        "vehicleId": 2001,
        "missionId": 1412,
        "curCommandId": 3,
        "location": {"latitude": 32.123, "longitude": 34.567, "altitude": 100.0},
        "yaw": 45.0,
        "pitch": 0.0,
        "roll": 0.0,
        "battery": 85.0,
        "speed": 1.5,
        "time": 1700000000,
    }
    sv = StateVector.model_validate(raw)
    assert sv.vehicleId == 2001
    assert sv.missionId == 1412
    assert sv.curCommandId == 3
    assert sv.location.latitude == 32.123
    assert sv.yaw == 45.0
    assert sv.battery == 85.0
    assert sv.speed == 1.5
    assert sv.time == 1700000000
    assert sv.gimballPitch is None

    dumped = json.loads(sv.model_dump_json(exclude_none=True))
    assert "gimballPitch" not in dumped


def test_state_vector_wire_format():
    """Validate against actual MADUM wire format from tmp/MMT/state_vector.json."""
    wire_json = """{
      "vehicleId": 1111,
      "missionId": -1,
      "curCommandId": -1,
      "time": 1716816286,
      "battery": 100.0,
      "yaw": 0.023164962892904775,
      "pitch": 0.011564288995686749,
      "roll": 0.001956599594331087,
      "location": {
        "latitude": 59.55356935010635,
        "longitude": 16.420095907482445,
        "altitude": -0.12139424681663513
      },
      "speed": 0.07199886307438368
    }"""
    sv = StateVector.model_validate_json(wire_json)
    assert sv.vehicleId == 1111
    assert sv.location.latitude == 59.55356935010635
    assert isinstance(sv.battery, float)
    assert sv.battery == 100.0


def test_state_vector_with_gimball_pitch():
    raw = {
        "vehicleId": 2001,
        "missionId": -1,
        "curCommandId": -1,
        "location": {"latitude": 32.0, "longitude": 34.0, "altitude": 0.0},
        "yaw": 0.0,
        "pitch": 0.0,
        "roll": 0.0,
        "gimballPitch": -30.0,
        "battery": 90.0,
        "speed": 0.0,
        "time": 1700000000,
    }
    sv = StateVector.model_validate(raw)
    assert sv.gimballPitch == -30.0

    dumped = json.loads(sv.model_dump_json(exclude_none=True))
    assert dumped["gimballPitch"] == -30.0


def test_vehicle_summary_roundtrip():
    raw = {
        "id": 2001,
        "name": "TerraSCout-1",
        "type": 2,
        "equipments": [
            {"id": 1, "name": "Main Camera", "type": 1, "status": 1},
            {"id": 2, "name": "IR Camera", "type": 3, "status": 1},
        ],
        "stateVector": {
            "vehicleId": 2001,
            "missionId": -1,
            "curCommandId": -1,
            "location": {"latitude": 32.0, "longitude": 34.0, "altitude": 0.0},
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "battery": 100.0,
            "speed": 0.0,
            "time": 1700000000,
        },
    }
    vs = VehicleSummary.model_validate(raw)
    assert vs.id == 2001
    assert vs.name == "TerraSCout-1"
    assert vs.type == VehicleType.UGV
    assert len(vs.equipments) == 2
    assert vs.equipments[0].type == EquipmentType.CAMERA_PHOTO
    assert vs.equipments[1].type == EquipmentType.IR_CAMERA_PHOTO
    assert vs.maxSpeed is None
    assert vs.maxFlightTime is None

    dumped = json.loads(vs.model_dump_json(exclude_none=True))
    assert "maxFlightTime" not in dumped
    assert "maxSpeed" not in dumped
    assert dumped["type"] == 2
    assert dumped["equipments"][0]["name"] == "Main Camera"


def test_mission_send_parse():
    raw = {
        "id": 1412,
        "vehicleId": 2001,
        "homeLocation": {"latitude": 32.0, "longitude": 34.0, "altitude": 0.0},
        "commands": [
            {
                "id": 1,
                "commandType": 2,
                "startTime": 0,
                "endTime": 0,
                "params": [32.1, 34.5, 100.0, 0.0, 0.0, 0.0, 0.0],
            },
            {
                "id": 2,
                "commandType": 2,
                "startTime": 0,
                "endTime": 0,
                "params": [32.2, 34.6, 100.0, 0.0, 0.0, 0.0, 0.0],
            },
            {
                "id": 3,
                "commandType": 3,
                "startTime": 0,
                "endTime": 0,
                "params": [1.0, 0.0, 0.0],
            },
        ],
    }
    ms = MissionSend.model_validate(raw)
    assert ms.id == 1412
    assert ms.vehicleId == 2001
    assert ms.homeLocation.latitude == 32.0
    assert len(ms.commands) == 3
    assert ms.commands[0].commandType == CommandType.NAV_WAYPOINT
    assert ms.commands[2].commandType == CommandType.CAMERA_IMAGE
    assert len(ms.commands[0].params) == 7
    assert len(ms.commands[2].params) == 3


def test_mission_update_serialize():
    mu = MissionUpdate(commandId=1, missionId=1412, vehicleId=2001, statusCode=StatusCode.RUNNING)
    dumped = json.loads(mu.model_dump_json())
    assert dumped == {
        "commandId": 1,
        "missionId": 1412,
        "vehicleId": 2001,
        "statusCode": 2,
    }


def test_alarm_serialize():
    alarm = Alarm(
        id=1,
        vehicleId=2001,
        alarmCode=AlarmCode.BATTERY_LOW,
        message="Battery below 20%",
        time=1700000000,
    )
    dumped = json.loads(alarm.model_dump_json())
    assert dumped["resourceId"] == 0
    assert dumped["alarmCode"] == 1
    assert dumped["message"] == "Battery below 20%"


def test_alarm_resolved_serialize():
    ar = AlarmResolved(id=2, vehicleId=2001, resolvedAlarmId=1, time=1700000100)
    dumped = json.loads(ar.model_dump_json())
    assert dumped["resolvedAlarmId"] == 1
    assert dumped["id"] == 2
    assert dumped["time"] == 1700000100


def test_mission_action_parse():
    """Spec Table 26 uses vehicleID/missionID (capital ID)."""
    raw = {"vehicleID": 2001, "missionID": 1412, "action": 1}
    ma = MissionAction.model_validate(raw)
    assert ma.vehicleId == 2001
    assert ma.missionId == 1412
    assert ma.action == MissionActionCode.ABORT


def test_mission_action_parse_camelcase():
    """MMT sample mission.json uses vehicleId (camelCase)."""
    raw = {"vehicleId": 2001, "missionId": 1412, "action": 1}
    ma = MissionAction.model_validate(raw)
    assert ma.vehicleId == 2001
    assert ma.missionId == 1412
    assert ma.action == MissionActionCode.ABORT


def test_mission_action_parse_without_mission_id():
    """MMT sample mission.json omits missionID entirely."""
    raw = {"vehicleId": 1111, "action": 1}
    ma = MissionAction.model_validate(raw)
    assert ma.vehicleId == 1111
    assert ma.missionId is None
    assert ma.action == MissionActionCode.ABORT


def test_mission_action_wire_format():
    """Validate against actual MMT wire format from tmp/MMT/mission.json."""
    wire_json = '{"vehicleId": 1111, "action": 1}'
    ma = MissionAction.model_validate_json(wire_json)
    assert ma.vehicleId == 1111
    assert ma.missionId is None
    assert ma.action == MissionActionCode.ABORT


def test_mission_full_action_parse():
    raw = {"missionID": 1412, "action": 2}
    mfa = MissionFullAction.model_validate(raw)
    assert mfa.missionId == 1412
    assert mfa.action == MissionActionCode.SUSPEND


def test_waypoint_command_params():
    cmd = Command(
        id=1,
        commandType=CommandType.NAV_WAYPOINT,
        startTime=0,
        endTime=0,
        params=[20.0, 1.5, 1.0, 90.0, 32.123, 34.567, 100.0],
    )
    assert cmd.params[4] == 32.123
    assert cmd.params[5] == 34.567
