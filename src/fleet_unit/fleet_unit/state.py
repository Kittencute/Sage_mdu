from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from fleet_unit.errors import UnitError

PENDING = "pending"
WAITING = "waiting"
STARTING = "starting"
READY = "ready"
FAILED = "failed"
STOPPED = "stopped"


@dataclass
class ProcessState:
    name: str
    kind: str
    state: str
    ready: bool = False
    pid: int | None = None
    exit_code: int | None = None
    signal: int | None = None
    detail: str = ""


@dataclass
class UnitState:
    unit: str
    runner_pid: int
    updated_at: float
    state: str
    detail: str
    processes: list[ProcessState] = field(default_factory=list)


def write_state(path: Path, state: UnitState) -> None:
    document = asdict(state)
    temporary = path.with_name(path.name + ".new")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_text(json.dumps(document, indent=2, sort_keys=False) + "\n")
        os.replace(temporary, path)
    except OSError as exc:
        raise UnitError(f"write state file {path}", exc, state.unit) from exc


def read_state(path: Path) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise UnitError(f"read state file {path}", exc) from exc
    if not isinstance(document, dict):
        raise UnitError(f"read state file {path}", "document must be a mapping")
    return document
