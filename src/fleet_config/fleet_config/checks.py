from __future__ import annotations

import json
import math
import re
from collections.abc import Iterator, Sequence
from pathlib import Path
from typing import Any

import yaml

from fleet_config.errors import RenderError
from fleet_config.paths import SOURCE_ROOT

PACKAGE_SHARE_PATH = re.compile(r"^\$\(find-pkg-share (?P<package>[A-Za-z0-9_]+)\)/(?P<path>.+)$")
LATTICE_PLANNER = "nav2_smac_planner::SmacPlannerLattice"
FOXGLOVE_PARAMETERS_KEYS = ("/**", "ros__parameters")
FOXGLOVE_READ_WHITELIST = "topic_whitelist"
FOXGLOVE_PUBLISH_WHITELIST = "client_topic_whitelist"
LAYOUT_TOPIC_KEYS = frozenset(
    {"imageTopic", "calibrationTopic", "topicPath", "topicToRender", "topic"}
)
LAYOUT_PUBLISH_KEYS = frozenset({"poseTopic", "pointTopic", "poseEstimateTopic"})
LAYOUT_LAYER_SETTINGS_KEY = "topics"
LAYOUT_PLOT_PANEL = "Plot!"
LAYOUT_PLOT_PATH_KEY = "value"
LAYOUT_PUBLISHING_PANELS = ("Teleop!",)
TOLERANCE = 1e-9


def read_yaml(path: Path) -> Any:
    try:
        return yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise RenderError(f"read {path}", exc) from exc


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RenderError(f"read {path}", exc) from exc


def value_at(document: Any, keys: Sequence[str], path: Path) -> Any:
    current = document
    for depth, key in enumerate(keys):
        if not isinstance(current, dict) or key not in current:
            raise RenderError(
                f"read {'.'.join(keys)} from {path}", f"{'.'.join(keys[: depth + 1])} is missing"
            )
        current = current[key]
    return current


def check_equal(label: str, actual: float, expected: float, path: Path) -> None:
    if not math.isclose(actual, expected, abs_tol=TOLERANCE):
        raise RenderError(f"check {label} in {path}", f"{actual} differs from {expected}")


def package_share_file(reference: str, path: Path) -> Path:
    match = PACKAGE_SHARE_PATH.match(reference)
    if match is None:
        raise RenderError(
            f"resolve {reference} from {path}", "expected $(find-pkg-share <package>)/<path>"
        )
    return SOURCE_ROOT / match["package"] / match["path"]


def check_lattice_resolution(nav2: Path, parameters_root: str) -> None:
    document = read_yaml(nav2)
    costmap_resolution = value_at(
        document,
        (parameters_root, "global_costmap", "global_costmap", "ros__parameters", "resolution"),
        nav2,
    )
    planners = value_at(document, (parameters_root, "planner_server", "ros__parameters"), nav2)
    for name, planner in planners.items():
        if not isinstance(planner, dict) or planner.get("plugin") != LATTICE_PLANNER:
            continue
        lattice = package_share_file(value_at(planner, ("lattice_filepath",), nav2), nav2)
        lattice_resolution = value_at(
            read_json(lattice), ("lattice_metadata", "grid_resolution"), lattice
        )
        check_equal(
            f"planner {name} lattice {lattice.name} grid_resolution against global_costmap resolution",
            lattice_resolution,
            costmap_resolution,
            nav2,
        )


def check_footprint_covers(nav2: Path, keys: Sequence[str], length: float, width: float) -> None:
    footprint = [tuple(point) for point in yaml.safe_load(value_at(read_yaml(nav2), keys, nav2))]
    edges = list(zip(footprint, footprint[1:] + footprint[:1], strict=True))
    crossings = [
        (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        for (ax, ay), (bx, by) in edges
        for px, py in footprint
    ]
    if min(crossings) < -TOLERANCE and max(crossings) > TOLERANCE:
        raise RenderError(
            f"check {'.'.join(keys)} in {nav2}", f"footprint {footprint} is not convex"
        )
    orientation = 1.0 if max(crossings) > TOLERANCE else -1.0
    for corner in ((x, y) for x in (-length / 2, length / 2) for y in (-width / 2, width / 2)):
        if any(
            orientation * ((bx - ax) * (corner[1] - ay) - (by - ay) * (corner[0] - ax)) < -TOLERANCE
            for (ax, ay), (bx, by) in edges
        ):
            raise RenderError(
                f"check {'.'.join(keys)} in {nav2}",
                f"footprint {footprint} does not cover corner {corner} of the {length} x {width} outline",
            )


def check_inflation_covers_footprint(nav2: Path, costmap_keys: Sequence[str]) -> None:
    costmap = value_at(read_yaml(nav2), costmap_keys, nav2)
    padding = value_at(costmap, ("footprint_padding",), nav2)
    footprint = yaml.safe_load(value_at(costmap, ("footprint",), nav2))
    padded = [[abs(value) + padding if value else 0.0 for value in point] for point in footprint]
    circumscribed = max(math.hypot(x, y) for x, y in padded)
    inflation = value_at(costmap, ("inflation_layer", "inflation_radius"), nav2)
    if inflation < circumscribed - TOLERANCE:
        raise RenderError(
            f"check {'.'.join(costmap_keys)}.inflation_layer.inflation_radius in {nav2}",
            f"{inflation} is below the circumscribed radius {circumscribed:.3f} of the padded footprint",
        )


def layout_topics(value: Any, panel: str) -> Iterator[tuple[str, str]]:
    if isinstance(value, list):
        for item in value:
            yield from layout_topics(item, panel)
        return
    if not isinstance(value, dict):
        return
    for key, item in value.items():
        if key == LAYOUT_LAYER_SETTINGS_KEY and isinstance(item, dict):
            for topic in item:
                yield topic, FOXGLOVE_READ_WHITELIST
            continue
        published = key in LAYOUT_PUBLISH_KEYS or (
            key == "topic" and panel.startswith(LAYOUT_PUBLISHING_PANELS)
        )
        read = key in LAYOUT_TOPIC_KEYS or (
            key == LAYOUT_PLOT_PATH_KEY and panel.startswith(LAYOUT_PLOT_PANEL)
        )
        if (published or read) and isinstance(item, str):
            whitelist = FOXGLOVE_PUBLISH_WHITELIST if published else FOXGLOVE_READ_WHITELIST
            yield item.split(".", 1)[0], whitelist
        else:
            yield from layout_topics(item, panel)


def check_layout_topics(layout: Path, foxglove_params: Path) -> None:
    parameters = value_at(read_yaml(foxglove_params), FOXGLOVE_PARAMETERS_KEYS, foxglove_params)
    panels = value_at(read_json(layout), ("configById",), layout)
    for panel, config in panels.items():
        for topic, whitelist in layout_topics(config, panel):
            patterns = value_at(parameters, (whitelist,), foxglove_params)
            if not any(re.fullmatch(pattern, topic) for pattern in patterns):
                raise RenderError(
                    f"check panel {panel} of {layout}",
                    f"topic {topic} matches no {whitelist} entry in {foxglove_params}",
                )
