from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "terrascout_navigation" / "scripts"))

from report_mission_bag import (
    extract_geojson_destinations,
    minimum_footprint_distances,
    point_polygon_distance,
    summarize_motion,
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
        "name": "WP 1", "latitude": 47.3, "longitude": 8.5, "status": "completed"
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
