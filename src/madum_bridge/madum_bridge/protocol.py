from enum import IntEnum

from pydantic import BaseModel, ConfigDict, Field


class CommandType(IntEnum):
    NAV_TAKEOFF = 0
    NAV_LAND = 1
    NAV_WAYPOINT = 2
    CAMERA_IMAGE = 3
    VIDEO_START_CAPTURE = 4
    VIDEO_STOP_CAPTURE = 5
    NAV_HOME = 6


class StatusCode(IntEnum):
    NOT_ASSIGNED = 0
    NOT_STARTED = 1
    RUNNING = 2
    FINISHED = 3
    STOPPED = 4
    FAILED = 5
    ABORTED = 6


class MissionActionCode(IntEnum):
    ABORT = 1
    SUSPEND = 2
    RESUME = 3


class AlarmCode(IntEnum):
    BATTERY_LOW = 1
    SENSOR_FAIL = 2
    MOTOR_FAIL = 3
    GPS_SIGNAL_LOST = 4
    IMBALANCED_PROPELLER = 5
    ESC_ARMING_FAIL = 6
    MAX_ALT_EXCEEDED = 7
    MAX_ATT_EXCEEDED = 8
    WIND_LIMIT_EXCEEDED = 9
    BATTERY_UNHEALTHY = 10
    ALT_BELOW_MIN_LIMIT = 11
    VEHICLE_CRASHED = 12
    UNKNOWN_ERROR = 99


class EquipmentType(IntEnum):
    CAMERA_PHOTO = 1
    CAMERA_VIDEO = 2
    IR_CAMERA_PHOTO = 3
    IR_CAMERA_VIDEO = 4
    CAMERA_FOLLOW = 5


class Position(BaseModel):
    latitude: float
    longitude: float
    altitude: float


class StateVector(BaseModel):
    vehicleId: int
    missionId: int = -1
    curCommandId: int = -1
    location: Position
    yaw: float | None = None
    pitch: float | None = None
    roll: float | None = None
    gimballPitch: float | None = None
    battery: float | None = None
    speed: float | None = None
    speedX: float | None = None
    speedY: float | None = None
    speedZ: float | None = None
    time: int
    highPrecisionTime: int = 0


class Equipment(BaseModel):
    id: int
    name: str
    type: EquipmentType
    status: int


class VehicleType(IntEnum):
    UAV = 1
    UGV = 2


class VehicleSummary(BaseModel):
    id: int
    name: str
    maxSpeed: float | None = None
    maxFlightTime: int | None = None
    equipments: list[Equipment]
    stateVector: StateVector
    type: VehicleType


class MissionUpdate(BaseModel):
    commandId: int
    missionId: int
    vehicleId: int
    statusCode: StatusCode


class MissionFailure(MissionUpdate):
    reason: str


class Alarm(BaseModel):
    id: int
    vehicleId: int
    alarmCode: AlarmCode
    resourceId: int = 0
    message: str
    time: int


class AlarmResolved(BaseModel):
    id: int
    vehicleId: int
    resolvedAlarmId: int
    time: int


class Command(BaseModel):
    id: int
    commandType: CommandType
    startTime: int
    endTime: int
    params: list[float]


class MissionSend(BaseModel):
    id: int
    vehicleId: int
    homeLocation: Position
    commands: list[Command]


class MissionAction(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    vehicleId: int = Field(alias="vehicleID")
    missionId: int | None = Field(default=None, alias="missionID")
    action: MissionActionCode


class MissionFullAction(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    missionId: int = Field(alias="missionID")
    action: MissionActionCode
