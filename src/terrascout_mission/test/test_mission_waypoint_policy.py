from types import SimpleNamespace

import pytest
from rclpy.action import GoalResponse

from fleet_interfaces.msg import MissionCommand, MissionState
from terrascout_mission.mission_server import (
    GroundMissionServer,
    MissionError,
    Phase,
)


def command(command_id, kind=MissionCommand.WAYPOINT, **fields):
    values = {
        "id": command_id,
        "kind": kind,
        "latitude": 47.0,
        "longitude": 8.0,
        "altitude_agl": 0.0,
        "acceptance_radius_m": 0.5,
        "yaw_deg": float("nan"),
        "hold_s": 0.0,
    }
    values.update(fields)
    return SimpleNamespace(**values)


def ready_server():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._ready = True
    server._startup_cancel_done = True
    server._mission = None
    server._position = SimpleNamespace(status=SimpleNamespace(status=0))
    server._keepout_zones = lambda: ([], "")
    server._home_keepout_rejection = lambda plan: ""
    return server


def test_mission_without_home_is_not_rejected_for_missing_home():
    server = ready_server()
    plan = SimpleNamespace(
        id=1,
        commands=[command(1)],
        home=SimpleNamespace(latitude=47.0, longitude=8.0, altitude=0.0),
    )

    assert server._rejection(plan) == ""


def test_home_before_later_destination_is_not_rejected():
    server = ready_server()
    plan = SimpleNamespace(
        id=1,
        commands=[command(1), command(2, MissionCommand.HOME), command(3)],
        home=SimpleNamespace(latitude=47.0, longitude=8.0, altitude=0.0),
    )

    assert server._rejection(plan) == ""


def test_duplicate_command_ids_reject_mission_with_reason_in_state():
    server = ready_server()
    published = []
    server._publish = published.append
    server.get_logger = lambda: SimpleNamespace(
        error=lambda message: None,
    )
    plan = SimpleNamespace(
        id=1,
        commands=[command(4), command(4)],
        home=SimpleNamespace(latitude=47.0, longitude=8.0, altitude=0.0),
    )
    request = SimpleNamespace(plan=plan)

    response = server._on_goal(request)

    assert response == GoalResponse.REJECT
    assert published[-1].failure_reason == "mission contains duplicate command IDs"
    assert published[-1].command_failure_reasons == [
        "mission contains duplicate command IDs",
        "mission contains duplicate command IDs",
    ]


def test_waypoint_failure_marks_failed_and_advances_when_next_destination_exists():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._continue_after_waypoint_failure = True
    mission = SimpleNamespace(
        index=0,
        plan=SimpleNamespace(commands=[
            command(7), command(8), command(9, MissionCommand.HOME),
        ]),
        command_failure_reasons=["", "", ""],
        waypoint_retries=2,
        retry_after=object(),
        status=[MissionState.COMMAND_RUNNING, MissionState.COMMAND_WAITING,
                MissionState.COMMAND_WAITING],
    )
    server._set_status = lambda m, index, status: m.status.__setitem__(index, status)
    server._write_snapshot = lambda mission: None
    server._publish_geojson = lambda *args, **kwargs: None
    server._start_command = lambda m: setattr(m, "started_index", m.index)
    server.get_logger = lambda: SimpleNamespace(warning=lambda message: None)

    continued = server._fail_waypoint_and_continue(mission, "planner returned no path")

    assert continued
    assert mission.status[0] == MissionState.COMMAND_FAILED
    assert mission.command_failure_reasons[0] == "planner returned no path"
    assert mission.index == 1
    assert mission.started_index == 1
    assert mission.waypoint_retries == 0
    assert mission.retry_after is None


def test_last_waypoint_failure_does_not_skip_home_or_claim_continuation():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._continue_after_waypoint_failure = True
    mission = SimpleNamespace(
        index=0,
        plan=SimpleNamespace(commands=[command(7), command(8, MissionCommand.LAND)]),
        command_failure_reasons=["", ""],
        status=[MissionState.COMMAND_RUNNING, MissionState.COMMAND_WAITING],
    )

    assert not server._fail_waypoint_and_continue(mission, "unreachable")
    assert mission.status[0] == MissionState.COMMAND_RUNNING
    assert mission.index == 0


def test_malformed_waypoint_is_reported_as_input_error():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._keepout_zones = lambda: ([], "")
    server._destination_keepout_reason = lambda *args: ""
    plan = SimpleNamespace(commands=[command(8, latitude=95.0)])

    reasons = server._invalid_waypoint_reasons(plan)

    assert reasons == {0: "waypoint command 8 latitude is outside [-90, 90]"}


def test_waypoint_inside_known_keepout_is_marked_for_individual_failure():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._keepout_zones = lambda: ([{"label": "zone-a"}], "")
    server._destination_keepout_reason = lambda *args: "destination lies inside keepout zone 'zone-a'"
    plan = SimpleNamespace(commands=[command(8)])

    assert server._invalid_waypoint_reasons(plan) == {
        0: "waypoint command 8 destination lies inside keepout zone 'zone-a'"
    }


@pytest.mark.parametrize("latitude,longitude", [(95.0, 8.0), (47.0, 181.0)])
def test_invalid_latitude_or_longitude_has_a_specific_reason(latitude, longitude):
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._keepout_zones = lambda: ([], "")
    server._destination_keepout_reason = lambda *args: ""
    plan = SimpleNamespace(commands=[command(12, latitude=latitude, longitude=longitude)])

    reason = server._invalid_waypoint_reasons(plan)[0]

    assert "outside" in reason


def test_keepout_update_reorders_only_waiting_destinations_and_publishes_reason():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._keepout_snapshot_mtime_ns = 1
    server._keepout_mtime_ns = lambda: 2
    server._keepout_zones = lambda: ([], "")
    server._position = SimpleNamespace(latitude=47.0, longitude=8.0)
    server._datum = SimpleNamespace(latitude=47.0, longitude=8.0, altitude=0.0)
    server._map_yaw_offset_deg = 0.0
    server._write_snapshot = lambda mission: None
    server._waypoints_before = lambda mission, index: 0
    published_states = []
    server._publish_state = lambda mission, status: published_states.append(status)
    server._publish_geojson = lambda *args, **kwargs: None
    server.get_logger = lambda: SimpleNamespace(warning=lambda message: None)

    active = command(1, latitude=47.0, longitude=8.0)
    far = command(2, latitude=47.0, longitude=8.02)
    near = command(3, latitude=47.0, longitude=8.01)
    final = command(4, MissionCommand.HOME)
    mission = SimpleNamespace(
        index=0,
        plan=SimpleNamespace(
            id=1,
            order_mode="unordered",
            home=SimpleNamespace(latitude=47.0, longitude=8.03),
            commands=[active, far, near, final],
        ),
        status=[MissionState.COMMAND_RUNNING, MissionState.COMMAND_WAITING,
                MissionState.COMMAND_WAITING, MissionState.COMMAND_WAITING],
        command_failure_reasons=["", "", "", ""],
        order_change_reason="",
        points=[],
        yaws=[],
        point_of={},
        waypoints=[],
    )

    server._process_keepout_update(mission)

    assert [item.id for item in mission.plan.commands] == [1, 3, 2, 4]
    assert "Keepout zones changed" in mission.order_change_reason
    assert published_states == [MissionState.MISSION_RUNNING]


def test_unordered_keepout_update_reorders_when_optimizer_changes_next_waypoint(monkeypatch):
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._keepout_snapshot_mtime_ns = 1
    server._keepout_mtime_ns = lambda: 2
    server._keepout_zones = lambda: ([], "")
    server._position = SimpleNamespace(latitude=47.0, longitude=8.0)
    server._datum = SimpleNamespace(latitude=47.0, longitude=8.0, altitude=0.0)
    server._map_yaw_offset_deg = 0.0
    server._write_snapshot = lambda mission: None
    server._waypoints_before = lambda mission, index: 0
    server._publish_state = lambda mission, status: None
    server._publish_geojson = lambda *args, **kwargs: None
    server.get_logger = lambda: SimpleNamespace(warning=lambda message: None)

    active = command(1, latitude=47.0, longitude=8.0)
    next_best = command(2, latitude=47.0, longitude=8.01)
    then_next = command(3, latitude=47.0, longitude=8.02)
    alternate_tail = command(5, latitude=47.0, longitude=8.03)
    final = command(4, MissionCommand.HOME)
    mission = SimpleNamespace(
        index=0,
        plan=SimpleNamespace(
            id=1,
            order_mode="unordered",
            home=SimpleNamespace(latitude=47.0, longitude=8.0),
                commands=[active, next_best, alternate_tail, then_next, final],
        ),
            status=[MissionState.COMMAND_RUNNING, MissionState.COMMAND_WAITING,
                MissionState.COMMAND_WAITING, MissionState.COMMAND_WAITING,
                MissionState.COMMAND_WAITING],
            command_failure_reasons=["", "", "", "", ""],
        order_change_reason="",
        points=[],
        yaws=[],
        point_of={},
        waypoints=[],
    )
    monkeypatch.setattr(
        "terrascout_mission.mission_server.order_waypoints",
        lambda coords, *args, **kwargs: [coords[0], coords[2], coords[1]],
    )

    server._process_keepout_update(mission)

    assert [item.id for item in mission.plan.commands] == [1, 2, 3, 5, 4]
    assert mission.index == 0
    assert mission.status[0] == MissionState.COMMAND_RUNNING
    assert mission.status[1:4] == [MissionState.COMMAND_WAITING] * 3
    assert "Keepout zones changed" in mission.order_change_reason


def test_keepout_update_preserves_ordered_mission_sequence():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._keepout_snapshot_mtime_ns = 1
    server._keepout_mtime_ns = lambda: 2
    server._keepout_zones = lambda: ([], "")
    server._position = SimpleNamespace(latitude=47.0, longitude=8.0)
    server._datum = SimpleNamespace(latitude=47.0, longitude=8.0, altitude=0.0)
    server._map_yaw_offset_deg = 0.0
    server._write_snapshot = lambda mission: None
    server._waypoints_before = lambda mission, index: 0
    server._publish_state = lambda mission, status: None
    server._publish_geojson = lambda *args, **kwargs: None
    server.get_logger = lambda: SimpleNamespace(warning=lambda message: None)

    active = command(1, latitude=47.0, longitude=8.0)
    farther_first = command(2, latitude=47.0, longitude=8.02)
    closer_second = command(3, latitude=47.0, longitude=8.01)
    mission = SimpleNamespace(
        index=0,
        plan=SimpleNamespace(
            id=1,
            order_mode="ordered",
            home=SimpleNamespace(latitude=47.0, longitude=8.0),
            commands=[active, farther_first, closer_second],
        ),
        status=[MissionState.COMMAND_RUNNING, MissionState.COMMAND_WAITING,
                MissionState.COMMAND_WAITING],
        command_failure_reasons=["", "", ""],
        order_change_reason="",
        points=[],
        yaws=[],
        point_of={},
        waypoints=[],
    )

    server._process_keepout_update(mission)

    assert [item.id for item in mission.plan.commands] == [1, 2, 3]
    assert mission.order_change_reason == ""


def test_keepout_update_marks_active_waypoint_for_cancel_and_skips_blocked_future():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._keepout_snapshot_mtime_ns = 1
    server._keepout_mtime_ns = lambda: 2
    server._keepout_zones = lambda: ([{"label": "new-zone"}], "")
    server._destination_keepout_reason = lambda latitude, *_: (
        "destination lies inside keepout zone 'new-zone'" if latitude in (47.01, 47.02) else ""
    )
    server._position = SimpleNamespace(latitude=47.0, longitude=8.0)
    server._datum = SimpleNamespace(latitude=47.0, longitude=8.0, altitude=0.0)
    server._map_yaw_offset_deg = 0.0
    server._write_snapshot = lambda mission: None
    server._waypoints_before = lambda mission, index: 0
    server._publish_state = lambda mission, status: None
    server._publish_geojson = lambda *args, **kwargs: None
    server.get_logger = lambda: SimpleNamespace(warning=lambda message: None)

    active = command(1, latitude=47.01)
    blocked_future = command(2, latitude=47.02)
    reachable_future = command(3, latitude=47.03, longitude=8.01)
    other_reachable = command(4, latitude=47.04, longitude=8.02)
    mission = SimpleNamespace(
        index=0,
        plan=SimpleNamespace(
            id=1,
            home=SimpleNamespace(latitude=47.0, longitude=8.0),
            commands=[active, blocked_future, reachable_future, other_reachable],
        ),
        status=[MissionState.COMMAND_RUNNING, MissionState.COMMAND_WAITING,
                MissionState.COMMAND_WAITING, MissionState.COMMAND_WAITING],
        command_failure_reasons=["", "", "", ""],
        order_change_reason="",
        points=[],
        yaws=[],
        point_of={},
        waypoints=[],
        keepout_blocked_reason="",
    )

    server._process_keepout_update(mission)

    assert "command 1" in mission.keepout_blocked_reason
    assert mission.status[1] == MissionState.COMMAND_FAILED
    assert "command 2" in mission.command_failure_reasons[1]

    server._navigation_stopped = lambda current_mission: True
    failures = []
    server._fail_waypoint_and_continue = lambda current_mission, reason: failures.append(reason) or True
    server._navigate(mission)

    assert failures and "command 1" in failures[0]
    assert mission.keepout_blocked_reason == ""


def test_planner_failure_uses_configured_retry_window_before_policy():
    class FakeNow:
        def __add__(self, duration):
            return duration

    server = GroundMissionServer.__new__(GroundMissionServer)
    server._waypoint_retry_limit = 1
    server._waypoint_retry_delay_s = 2.0
    server._now = FakeNow
    server.get_logger = lambda: SimpleNamespace(warning=lambda message: None)
    mission = SimpleNamespace(
        index=0,
        plan=SimpleNamespace(commands=[command(5)]),
        waypoint_retries=0,
        retry_after=None,
    )

    assert server._retry_waypoint(mission, "planner found no path")
    assert mission.waypoint_retries == 1
    assert mission.retry_after is not None
    assert not server._retry_waypoint(mission, "planner still found no path")


def test_active_navigation_planner_exception_is_retried_before_failure_policy():
    class FakeNow:
        def __add__(self, duration):
            return duration

    server = GroundMissionServer.__new__(GroundMissionServer)
    server._waypoint_retry_limit = 1
    server._waypoint_retry_delay_s = 2.0
    server._now = FakeNow
    server.get_logger = lambda: SimpleNamespace(warning=lambda message: None)
    server._navigation_status = lambda navigation: (_ for _ in ()).throw(
        MissionError("planner returned no path")
    )
    server._fail_waypoint_and_continue = lambda *args, **kwargs: pytest.fail(
        "failure policy must not run before configured retries are exhausted"
    )
    mission = SimpleNamespace(
        index=0,
        navigation=object(),
        plan=SimpleNamespace(commands=[command(5)]),
        waypoint_retries=0,
        retry_after=None,
    )

    server._navigate(mission)

    assert mission.navigation is None
    assert mission.waypoint_retries == 1
    assert mission.retry_after is not None


def test_runtime_localization_fault_suspends_active_mission_and_reports_reason():
    server = GroundMissionServer.__new__(GroundMissionServer)
    mission = SimpleNamespace(
        phase=Phase.NAVIGATING,
        plan=SimpleNamespace(id=99),
        goal=None,
        abort_reason="",
        deadline=None,
    )
    server._mission = mission
    server._cancel_navigation_after_restart = lambda: None
    server._track_navigation = lambda: None
    server._update_readiness = lambda: None
    server._process_keepout_update = lambda current: None
    server._localization_reason = lambda check_tf: "position fix is stale"
    server._set_deadline = lambda current, seconds, reason: None
    server._advance = lambda current: None
    server._now = lambda: SimpleNamespace()
    server._publish_state = lambda current, status: None
    server.get_logger = lambda: SimpleNamespace(warning=lambda message: None)

    server._tick()

    assert mission.phase == Phase.SUSPENDING
    assert mission.abort_reason == "localization fault: position fix is stale"


def test_localization_fault_suspension_can_be_resumed_explicitly():
    server = GroundMissionServer.__new__(GroundMissionServer)
    server._localization_reason = lambda check_tf: "" if check_tf else "position fix is stale"
    server._startup_cancel_done = True
    server._nav = SimpleNamespace(server_is_ready=lambda: True)
    server._path_client = SimpleNamespace(server_is_ready=lambda: True)
    async def resume_path_error(mission):
        return ""

    server._resume_path_error = resume_path_error
    server._write_snapshot = lambda *args, **kwargs: None
    server._start_command = lambda mission: setattr(mission, "started", True)
    server._mission = None
    mission = SimpleNamespace(
        phase=Phase.PAUSED,
        index=0,
        plan=SimpleNamespace(commands=[command(11)]),
        status=[MissionState.COMMAND_PAUSED],
        abort_reason="localization fault: position fix is stale",
    )
    server._mission = mission
    server._set_status = lambda current, index, status: current.status.__setitem__(index, status)

    accepted, reason = __import__("asyncio").run(server._control(2))

    assert accepted
    assert reason == ""
    assert mission.abort_reason == ""
    assert mission.started
