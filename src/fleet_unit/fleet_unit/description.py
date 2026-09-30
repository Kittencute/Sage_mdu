from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from fleet_unit.errors import UnitError
from fleet_unit.manifest import from_document


@dataclass(frozen=True)
class BoundaryEntry:
    topic: str
    type: str
    max_hz: float


@dataclass(frozen=True)
class Description:
    id: str
    robot_class: str
    mode: str
    ros_namespace: str
    domain_id: int
    platform_kind: str
    capabilities: tuple[str, ...]
    exports: tuple[BoundaryEntry, ...]
    imports: tuple[BoundaryEntry, ...]


def load_description(path: Path, unit: str) -> Description:
    operation = f"load description {path}"
    try:
        document = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise UnitError(operation, exc, unit) from exc
    return from_document(Description, document, operation, unit)
