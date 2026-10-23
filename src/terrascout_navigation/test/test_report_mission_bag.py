from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "terrascout_navigation" / "scripts"))

from report_mission_bag import (
    extract_geojson_destinations,
    minimum_footprint_distances,
    point_polygon_distance,
    count_rising_edges,
    summarize_initial_ordering,
    summarize_faults,
    summarize_motion,
    write_events,
    filter_samples_to_window,
    validate_timestamp_streams,
    destination_failure_reason,
    split_mission_instances,
    render_readable_report,
)


def test_summarize_motion_integrates_distance_and_counts_events():
    def odom(x, y):
        return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))))

    def twist(vx, wz=0.0):
        return SimpleNamespace(linear=SimpleNamespace(x=vx, y=0.0), angular=SimpleNamespace(z=wz))

    def boolean(value):
        return SimpleNamespace(data=value)

    samples = {
        "/odom": [(1, odom(0.0, 0.0)), (2, odom(3.0, 4.0)), (3, odom(6.0, 4.0))],
        "/cmd": [(1, twist(0.2)), (2, twist(0.1)), (3, twist(0.0)), (4, twist(0.0)), (5, twist(0.2)), (6, twist(0.0))],
        "/teleop": [(1, boolean(False)), (2, boolean(True)), (3, boolean(True)), (4, boolean(False)), (5, boolean(True))],
        "/safety": [(1, boolean(False)), (2, boolean(True)), (3, boolean(False))],
    }

    metrics, available, missing = summarize_motion(samples, "/odom", "/cmd", "/teleop", "/safety")

    assert metrics == {
        "distance_driven_m": 8.0,
        "stop_events": 2,
        "teleop_takeover_events": 2,
        "safety_stop_events": 1,
    }
    assert len(available) == 4
    assert missing == []


def test_summarize_motion_marks_unrecorded_topics_unavailable():
    metrics, available, missing = summarize_motion({}, "/odom", "/cmd", "/teleop", "/safety")

    assert all(value is None for value in metrics.values())
    assert available == []
    assert len(missing) == 4


def test_count_rising_edges_counts_distinct_interventions():
    messages = [SimpleNamespace(data=value) for value in (False, True, True, False, True)]
    assert count_rising_edges({"/lock": list(enumerate(messages))}, "/lock") == 2


def test_summarize_faults_extracts_error_and_fatal_only():
    messages = [
        SimpleNamespace(level=30, name="node", msg="warning"),
        SimpleNamespace(level=40, name="node", msg="error"),
        SimpleNamespace(level=50, name="node", msg="fatal"),
    ]
    faults, missing = summarize_faults({"/rosout": list(enumerate(messages))}, "/rosout")
    assert [fault["message"] for fault in faults] == ["error", "fatal"]
    assert missing is None
    assert summarize_faults({}, "/rosout") == (None, "fault_events: /rosout missing")


def test_write_events_records_changed_order_and_reason(tmp_path):
    def state(order, reason=""):
        return SimpleNamespace(
            mission_id=4,
            mission_status=1,
            current_command_id=order[0],
            command_ids=order,
            command_status=[0] * len(order),
            command_failure_reasons=[""] * len(order),
            order_change_reason=reason,
            failure_reason="",
            ready=True,
            not_ready_reason="",
        )

    path = tmp_path / "events.csv"
    events = write_events(
        [(10, state([1, 2, 3])), (20, state([1, 3, 2], "Keepout changed route feasibility."))],
        path,
        4,
    )
    changes = [event for event in events if event["event"] == "order_changed"]
    assert len(changes) == 1
    assert changes[0]["order_change_reason"] == "Keepout changed route feasibility."
    assert "order_change_reason" in path.read_text()


def test_initial_unordered_decision_reports_requested_and_selected_route():
    metadata = {"ordering_mode": "unordered"}
    requested = {"waypoints": [
        {"name": "WP1", "coordinates": [47.0, 8.0]},
        {"name": "WP2", "coordinates": [47.1, 8.1]},
    ]}
    selected = [
        {"name": "WP2", "latitude": 47.1, "longitude": 8.1, "status": "pending"},
        {"name": "WP1", "latitude": 47.0, "longitude": 8.0, "status": "pending"},
    ]
    decision = summarize_initial_ordering(metadata, requested, selected)
    assert decision["ordering_algorithm"] == (
        "exhaustive shortest closed tour using straight-line geographic distances"
    )
    assert "not Nav2 planned-path length" in decision["selection_reason"]
    assert decision["requested_waypoints"] == ["WP1", "WP2"]
    assert decision["selected_waypoints"] == ["WP2", "WP1"]
    assert decision["order_changes"] == []


def test_timestamp_validation_alternative_allows_plan_when_trajectory_empty():
    result = validate_timestamp_streams(
        {"/plan": [(12, object())], "/trajectories": []},
        {"navigation path": ("/plan", "/trajectories")},
        10,
        15,
    )
    assert result["status"] == "PASS"
    assert result["streams"]["navigation path"][0]["topic"] == "/plan"


def test_timestamp_validation_requires_samples_and_mission_window_overlap():
    samples = {"/state": [(100, object())], "/path": [(120, object())]}
    result = validate_timestamp_streams(
        samples, {"state": ("/state",), "path": ("/path", "/alt_path")}, 90, 130
    )
    assert result["status"] == "PASS"
    assert result["required_streams_have_samples"]
    assert result["required_streams_within_recording_window"]
    assert result["streams"]["path"][0]["count"] == 1
    failed = validate_timestamp_streams(samples, {"path": ("/path",)}, 0, 10)
    assert failed["status"] == "FAIL"
    assert failed["streams_without_mission_window_overlap"] == ["path"]


def test_timestamp_validation_accepts_expected_no_overlap_after_planning_failure():
    result = validate_timestamp_streams(
        {"/plan": [(10, object())], "/cmd": [(12, object())]},
        {"navigation path": ("/plan",), "commands": ("/cmd",)},
        100,
        110,
        expected_no_overlap_groups=("navigation path", "commands"),
    )
    assert result["status"] == "PASS_WITH_EXPECTED_NO_OVERLAP"
    assert result["required_streams_have_samples"]
    assert result["required_streams_within_recording_window"]
    assert result["expected_no_overlap_groups"] == ["commands", "navigation path"]
    assert result["unexpected_no_overlap_groups"] == []


def test_filter_samples_to_selected_mission_window():
    samples = {"/odom": [(1, "before"), (2, "start"), (3, "inside"), (4, "after")]}
    assert filter_samples_to_window(samples, 2, 3) == {"/odom": [(2, "start"), (3, "inside")]}


def test_destination_failure_reason_is_matched_by_command_id():
    outcomes = {
        12: {"command_id": 12, "status": "FAILED", "failure_reason": "Blocked by keepout."},
        13: {"command_id": 13, "status": "FAILED", "failure_reason": "Localization lost."},
    }
    assert destination_failure_reason({"command_id": 13}, outcomes) == "Localization lost."
    assert destination_failure_reason({"command_id": 99}, outcomes, "Mission failed.") == (
        "Destination failed; no matching command failure reason was recorded."
    )


def test_extract_geojson_preserves_destination_command_id():
    message = SimpleNamespace(data='''{"type":"FeatureCollection","features":[
      {"type":"Feature","geometry":{"type":"Point","coordinates":[8.0,47.0]},
       "properties":{"name":"WP 2","metadata":{"lat":"47.0","lon":"8.0","command_id":"12"}}}
    ]}''')
    destination = extract_geojson_destinations([(10, message)])[0]
    assert destination["command_id"] == 12


def test_repeated_mission_id_after_terminal_status_is_split_into_instances():
    def state(mission_id, status, command):
        return SimpleNamespace(
            mission_id=mission_id,
            mission_status=status,
            current_command_id=command,
            command_ids=[1, 2],
            command_status=[3, 3] if status == 3 else [1, 0],
        )

    instances = split_mission_instances([
        (10, state(7, 1, 1)),
        (15, state(8, 1, 1)),
        (20, state(7, 3, 2)),
        (25, state(8, 3, 2)),
        (30, state(7, 1, 1)),
    ])
    assert [
        (entry["mission_id"], entry["instance"], len(entry["states"]))
        for entry in instances
    ] == [(7, 1, 2), (8, 1, 2), (7, 2, 1)]


def test_readable_report_uses_plain_language_sections_and_tables():
    report = {
        "mission_id": 2,
        "mission_label": "Mission 2 — run 1",
        "mission_run_id": "mission-2-run-1-start-10ns",
        "final_mission_status": "FINISHED",
        "duration_s": 12.0,
        "caveats": [],
        "artifacts": {"trajectory_comparison": "comparison.png",
                  "position_error": None, "orientation_error": None},
        "mis21": {"timestamp_validation": {"status": "PASS"}},
        "mis22": {
            "metrics": {"distance_driven_m": 4.2, "stop_events": 1,
                        "teleop_takeover_events": 0, "safety_stop_events": 0},
            "destination_outcomes": [{"name": "WP 1", "status": "completed",
                                       "command_id": 3, "outcome_reason": "Reached"}],
            "minimum_destination_edge_distances": [{
                "command_id": 3, "minimum_vehicle_edge_distance_m": 0.0}],
            "initial_ordering_decision": {"ordering_algorithm": "input order",
                                          "selection_reason": "Preserve operator order."},
            "order_changes": [], "fault_events": [],
        },
    }
    text = render_readable_report(report)
    assert "## Mission results" in text
    assert "## Destination outcomes" in text
    assert "[Trajectory comparison](comparison.png)" in text
    assert "**Run:**" not in text
    assert "| WP 1 | completed | 0.00 m | Reached |" in text
    assert "MIS-22" not in text


def test_point_polygon_distance_is_zero_inside_and_distance_to_edge_outside():
    square = ((1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0))
    assert point_polygon_distance((0.0, 0.0), square) == 0.0
    assert point_polygon_distance((2.5, 0.0), square) == 1.5


def test_extract_geojson_waypoint_points_and_status():
    message = SimpleNamespace(data='''{"type":"FeatureCollection","features":[
      {"type":"Feature","geometry":{"type":"Point","coordinates":[8.5,47.3]},
       "properties":{"name":"WP 1","metadata":{"lat":"47.3","lon":"8.5","status":"completed"}}},
      {"type":"Feature","geometry":{"type":"LineString","coordinates":[]},"properties":{}}
    ]}''')

    assert extract_geojson_destinations([(123, message)]) == [{
        "name": "WP 1", "latitude": 47.3, "longitude": 8.5,
        "status": "completed", "command_id": None,
    }]


def test_minimum_footprint_distance_uses_ground_truth_and_datum():
    pose = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    odom = SimpleNamespace(pose=SimpleNamespace(pose=pose))
    destination = {"name": "WP 1", "latitude": 47.397742, "longitude": 8.545594, "status": "completed"}

    score = minimum_footprint_distances([destination], [(10, odom)], (47.397742, 8.545594, 488.0))

    assert score is not None
    distances, used = score
    assert distances["WP 1"] == 0.0
    assert used == 1


def test_recording_datetime_folder_name_uses_latest_existing_bundle(tmp_path):
    from compare_fast_lio_gt import recording_start_datetime

    bag = tmp_path / "rosbag2_2026_10_19-07_33_46"
    assert recording_start_datetime(bag, None).isoformat() == "2026-10-19T07:33:46+00:00"


def test_processor_reuses_bundle_tagged_for_selected_bag(tmp_path, monkeypatch):
    import json
    import process_mission_recording as processor

    results = tmp_path / "mission_results"
    bundle = results / "mission_20261019T093346"
    bundle.mkdir(parents=True)
    bag = tmp_path / "rosbag2_2026_10_19-07_33_46"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 60000000000\n")
    (bundle / "mission_report.json").write_text(json.dumps({"recording": f"/workspace/{bag.name}"}))
    monkeypatch.setattr(processor, "RESULTS_ROOT", results)

    assert processor.select_results_dir(bag) == bundle


def test_processor_ignores_unrelated_newest_bundle(tmp_path, monkeypatch):
    import json
    import process_mission_recording as processor

    results = tmp_path / "mission_results"
    results.mkdir()
    unrelated = results / "mission_20261019_112957"
    unrelated.mkdir()
    (unrelated / "metadata.json").write_text(json.dumps({"bundle_created_at": "2026-10-19T11:29:57+02:00"}))
    bag = tmp_path / "rosbag2_2026_10_19-07_33_46"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 60000000000\n")
    monkeypatch.setattr(processor, "RESULTS_ROOT", results)
    monkeypatch.setattr(processor, "WORKSPACE_ROOT", tmp_path)
    monkeypatch.setattr(processor, "STOCKHOLM", __import__("zoneinfo").ZoneInfo("Europe/Stockholm"))

    assert processor.select_results_dir(bag) == results / "mission_20261019_093346"


def test_invalid_empty_output_path_is_rejected_by_parser():
    # The CLI parsers must not interpret an empty shell variable as Path('.')
    import subprocess

    script = Path(__file__).resolve().parents[1] / "terrascout_navigation" / "scripts" / "compare_fast_lio_gt.py"
    result = subprocess.run(
        [sys.executable, str(script), "missing-bag", "--out-dir", ""],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "--out-dir cannot be empty" in result.stderr
