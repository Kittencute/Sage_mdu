from __future__ import annotations

import json
import os
from enum import Enum
from pathlib import Path

from madum_bridge.protocol import Command, MissionSend


class MissionState(Enum):
    IDLE = "idle"
    EXECUTING = "executing"
    SUSPENDED = "suspended"


class MissionExecutor:
    def __init__(self, persist_path: str | None = None):
        self.state = MissionState.IDLE
        self.mission: MissionSend | None = None
        self.command_index: int = 0
        self.last_completed_mission_id: int | None = None
        self.ended_mission_ids: set[int] = set()
        self._persist_path = persist_path
        self._load_persisted()

    def _load_persisted(self):
        if self._persist_path is None:
            return
        path = Path(self._persist_path)
        try:
            contents = path.read_text()
        except FileNotFoundError:
            return
        except OSError as exc:
            raise RuntimeError(f"read completed mission state {path} failed: cause: {exc}") from exc
        try:
            mission_id = json.loads(contents)["mission_id"]
            if type(mission_id) is not int:
                raise TypeError(f"mission_id must be an integer, got {mission_id!r}")
        except (json.JSONDecodeError, KeyError, TypeError) as exc:
            raise RuntimeError(
                f"parse completed mission state {path} failed: cause: {exc}"
            ) from exc
        self.last_completed_mission_id = mission_id
        self.ended_mission_ids.add(mission_id)

    def _save_persisted(self):
        if self._persist_path is None or self.last_completed_mission_id is None:
            return
        path = Path(self._persist_path)
        temporary = path.with_name(f".{path.name}.{os.getpid()}")
        try:
            with temporary.open("w") as stream:
                json.dump({"mission_id": self.last_completed_mission_id}, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        except OSError as exc:
            try:
                temporary.unlink(missing_ok=True)
            except OSError as cleanup_exc:
                raise RuntimeError(
                    f"write completed mission state {path} failed: cause: {exc}; "
                    f"remove temporary mission state {temporary} failed: "
                    f"cause: {cleanup_exc}"
                ) from exc
            raise RuntimeError(
                f"write completed mission state {path} failed: cause: {exc}"
            ) from exc

    @property
    def mission_id(self) -> int | None:
        return self.mission.id if self.mission else None

    @property
    def current_command(self) -> Command | None:
        if self.mission is None:
            return None
        if self.command_index >= len(self.mission.commands):
            return None
        return self.mission.commands[self.command_index]

    def is_stale_mission(self, mission_id: int) -> bool:
        return mission_id in self.ended_mission_ids

    def start_mission(self, mission: MissionSend) -> bool:
        if self.state != MissionState.IDLE:
            return False
        self.mission = mission
        self.command_index = 0
        self.state = MissionState.EXECUTING
        return True

    def advance(self) -> bool:
        if self.state != MissionState.EXECUTING or self.mission is None:
            return False
        self.command_index += 1
        if self.command_index >= len(self.mission.commands):
            self.last_completed_mission_id = self.mission.id
            self.ended_mission_ids.add(self.mission.id)
            self._reset()
            self._save_persisted()
            return False
        return True

    def suspend(self):
        if self.state == MissionState.EXECUTING:
            self.state = MissionState.SUSPENDED

    def resume(self):
        if self.state == MissionState.SUSPENDED:
            self.state = MissionState.EXECUTING

    def abort(self):
        if self.state in (MissionState.EXECUTING, MissionState.SUSPENDED):
            assert self.mission is not None
            self.ended_mission_ids.add(self.mission.id)
            self._reset()

    def _reset(self):
        self.mission = None
        self.command_index = 0
        self.state = MissionState.IDLE
