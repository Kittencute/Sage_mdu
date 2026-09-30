import pytest
import yaml

from fleet_config import model
from fleet_config.model import Instance
from fleet_config.render import render_target
from fleet_config.targets import TARGETS
from fleet_unit.manifest import load_manifest


@pytest.mark.parametrize("name", sorted(TARGETS))
def test_every_target_renders_and_its_manifests_load(name, tmp_path, monkeypatch):
    monkeypatch.setattr(
        model, "SITE_FILE", model.SOURCE_ROOT / "fleet_config" / "site.example.yaml"
    )
    out = tmp_path / name
    render_target(name, Instance(1), out)
    summary = yaml.safe_load((out / "fleet.yaml").read_text())
    services = yaml.safe_load((out / "compose.yaml").read_text())["services"]
    assert summary["target"] == name
    for unit in summary["units"]:
        assert unit["service"] in services
        manifest = load_manifest(out / "units" / unit["id"])
        assert manifest.unit == unit["id"]
        assert manifest.router.tcp_port == Instance(1).unit_router_tcp_port(
            summary["units"].index(unit)
        )
