from collections import Counter
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest
import yaml

from fleet_config import model
from fleet_config.model import Instance
from fleet_config.render import render_target
from fleet_config.targets import TARGETS
from fleet_unit.manifest import Manifest, Process, load_manifest

COLLISION_MONITOR_PLUGIN = "nav2_collision_monitor::CollisionMonitor"
REQUIRED_POLYGON_ACTIONS = ("stop", "slowdown")
CLIENT_PUBLISH = "clientPublish"
FIXPOSITION_NAV2_EDGES = (("FP_ENU0", "map"), ("map", "odom"), ("odom", "vrtk_link"))


@pytest.fixture(params=sorted(TARGETS))
def rendered(request, tmp_path, monkeypatch) -> Path:
    monkeypatch.setattr(
        model, "SITE_FILE", model.SOURCE_ROOT / "fleet_config" / "site.example.yaml"
    )
    out = tmp_path / request.param
    render_target(request.param, Instance(1), out)
    return out


def unit_manifests(out: Path) -> Iterator[Manifest]:
    for unit in yaml.safe_load((out / "fleet.yaml").read_text())["units"]:
        yield load_manifest(out / "units" / unit["id"])


def parameters(manifest: Manifest, params: tuple[str, ...], node: str) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for path in params:
        document = yaml.safe_load((manifest.unit_dir / path).read_text())
        values |= document.get("/**", {}).get("ros__parameters", {})
        values |= document.get(manifest.namespace, {}).get(node, {}).get("ros__parameters", {})
    return values


def process_parameters(manifest: Manifest, process: Process) -> dict[str, Any]:
    return parameters(manifest, process.params, process.node_name or process.name) | dict(
        process.param_values
    )


def tf_edges(manifest: Manifest, process: Process) -> Iterator[tuple[str, str]]:
    values = process_parameters(manifest, process)
    if process.executable == "fixposition_driver_ros2_exec" and values["nav2_mode"]:
        yield from FIXPOSITION_NAV2_EDGES
    if process.executable == "gnss_shim.py":
        odom = values.get("frame_odom", "odom")
        yield values.get("frame_global", "map"), odom
        yield odom, values.get("frame_root", "vrtk_link")
    if process.executable == "odom_converter.py" and values.get("publish_tf", True):
        yield values.get("odom_frame", "odom"), values.get("base_frame", "base_link")
    if process.executable == "ros2_control_node":
        drive = parameters(manifest, process.params, "diff_drive_controller")
        if drive.get("enable_odom_tf", True):
            yield drive["odom_frame_id"], drive["base_frame_id"]


def test_twist_mux_locks_have_positive_timeouts(rendered):
    for manifest in unit_manifests(rendered):
        for process in manifest.processes:
            if process.executable != "twist_mux":
                continue
            locks = process_parameters(manifest, process)["locks"]
            assert locks, f"{manifest.unit} twist_mux has no locks"
            for name, lock in locks.items():
                assert lock["timeout"] > 0, (
                    f"{manifest.unit} twist_mux lock {name} timeout {lock['timeout']}"
                )


def test_collision_monitor_polygons_are_enabled(rendered):
    for manifest in unit_manifests(rendered):
        for process in manifest.processes:
            for component in process.load:
                if component.plugin != COLLISION_MONITOR_PLUGIN:
                    continue
                monitor = parameters(manifest, component.params, component.node_name)
                enabled = {
                    monitor[name]["action_type"]
                    for name in monitor["polygons"]
                    if monitor[name]["enabled"] is True
                }
                for action in REQUIRED_POLYGON_ACTIONS:
                    assert action in enabled, (
                        f"{manifest.unit} {component.node_name} has no enabled {action} polygon"
                    )


def test_unit_services_run_an_init_process(rendered):
    services = yaml.safe_load((rendered / "compose.yaml").read_text())["services"]
    for unit in yaml.safe_load((rendered / "fleet.yaml").read_text())["units"]:
        assert services[unit["service"]].get("init") is True, (
            f"service {unit['service']} has no init: true"
        )


def test_every_tf_edge_has_one_publisher(rendered):
    for manifest in unit_manifests(rendered):
        edges = Counter(
            edge for process in manifest.processes for edge in tf_edges(manifest, process)
        )
        shared = [edge for edge, count in edges.items() if count > 1]
        assert not shared, f"{manifest.unit} publishes {shared} from more than one process"


def test_foxglove_bridges_have_topic_whitelists(rendered):
    for manifest in unit_manifests(rendered):
        for process in manifest.processes:
            if process.executable != "foxglove_bridge":
                continue
            values = process_parameters(manifest, process)
            assert values.get("topic_whitelist"), (
                f"{manifest.unit} foxglove_bridge has no topic_whitelist"
            )
            if CLIENT_PUBLISH in values["capabilities"]:
                assert values.get("client_topic_whitelist"), (
                    f"{manifest.unit} foxglove_bridge allows {CLIENT_PUBLISH} without client_topic_whitelist"
                )
