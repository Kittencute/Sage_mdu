import json

from madum_bridge.mission_executor import MissionExecutor, MissionState
from madum_bridge.protocol import Command, CommandType, MissionSend, Position


def _make_mission(num_commands=3) -> MissionSend:
    commands = []
    for i in range(num_commands):
        commands.append(
            Command(
                id=i,
                commandType=CommandType.NAV_WAYPOINT,
                startTime=i * 10,
                endTime=(i + 1) * 10,
                params=[0.0, 0.0, 0.0, 0.0, 65.575, 19.165, 50.0],
            )
        )
    return MissionSend(
        id=1412,
        vehicleId=2001,
        homeLocation=Position(latitude=65.575, longitude=19.165, altitude=0.0),
        commands=commands,
    )


def test_initial_state():
    executor = MissionExecutor()
    assert executor.state == MissionState.IDLE
    assert executor.mission is None
    assert executor.current_command is None


def test_start_mission():
    executor = MissionExecutor()
    mission = _make_mission()
    executor.start_mission(mission)
    assert executor.state == MissionState.EXECUTING
    assert executor.mission_id == 1412
    assert executor.command_index == 0
    assert executor.current_command is not None
    assert executor.current_command.id == 0


def test_advance_command():
    executor = MissionExecutor()
    executor.start_mission(_make_mission())
    has_next = executor.advance()
    assert has_next is True
    assert executor.command_index == 1
    assert executor.current_command is not None
    assert executor.current_command.id == 1
    has_next = executor.advance()
    assert has_next is True
    assert executor.command_index == 2
    has_next = executor.advance()
    assert has_next is False
    assert executor.state == MissionState.IDLE


def test_suspend_and_resume():
    executor = MissionExecutor()
    executor.start_mission(_make_mission())
    executor.advance()
    executor.suspend()
    assert executor.state == MissionState.SUSPENDED
    assert executor.command_index == 1
    executor.resume()
    assert executor.state == MissionState.EXECUTING
    assert executor.command_index == 1
    assert executor.current_command is not None
    assert executor.current_command.id == 1


def test_abort():
    executor = MissionExecutor()
    executor.start_mission(_make_mission())
    executor.advance()
    executor.abort()
    assert executor.state == MissionState.IDLE
    assert executor.mission is None


def test_abort_from_suspended():
    executor = MissionExecutor()
    executor.start_mission(_make_mission())
    executor.suspend()
    executor.abort()
    assert executor.state == MissionState.IDLE


def test_abort_marks_mission_stale():
    executor = MissionExecutor()
    executor.start_mission(_make_mission())
    executor.abort()
    assert executor.is_stale_mission(1412) is True


def test_start_rejects_if_already_executing():
    executor = MissionExecutor()
    executor.start_mission(_make_mission())
    result = executor.start_mission(_make_mission())
    assert result is False


def test_stale_mission_detection():
    executor = MissionExecutor()
    executor.start_mission(_make_mission())
    executor.advance()
    executor.advance()
    executor.advance()
    assert executor.state == MissionState.IDLE
    assert executor.last_completed_mission_id == 1412
    assert executor.is_stale_mission(1412) is True
    assert executor.is_stale_mission(1413) is False


def test_persist_completed_mission(tmp_path):
    persist_file = tmp_path / "last_mission.json"
    executor = MissionExecutor(persist_path=str(persist_file))
    executor.start_mission(_make_mission())
    executor.advance()
    executor.advance()
    executor.advance()
    assert executor.state == MissionState.IDLE
    assert persist_file.exists()
    data = json.loads(persist_file.read_text())
    assert data["mission_id"] == 1412


def test_load_persisted_mission_on_init(tmp_path):
    persist_file = tmp_path / "last_mission.json"
    persist_file.write_text(json.dumps({"mission_id": 1412}))
    executor = MissionExecutor(persist_path=str(persist_file))
    assert executor.is_stale_mission(1412) is True
    assert executor.is_stale_mission(1413) is False


def test_no_persist_file_on_init():
    executor = MissionExecutor(persist_path="/tmp/nonexistent_test_file.json")
    assert executor.last_completed_mission_id is None


def test_aborted_mission_not_persisted(tmp_path):
    persist_file = tmp_path / "last_mission.json"
    executor = MissionExecutor(persist_path=str(persist_file))
    executor.start_mission(_make_mission())
    executor.abort()
    assert not persist_file.exists()
