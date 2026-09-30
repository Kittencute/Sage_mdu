from __future__ import annotations


class UnitError(Exception):
    def __init__(
        self,
        operation: str,
        cause: object,
        unit: str | None = None,
        process: str | None = None,
    ) -> None:
        parts = [f"operation: {operation}"]
        if unit is not None:
            parts.append(f"unit: {unit}")
        if process is not None:
            parts.append(f"process: {process}")
        parts.append(f"cause: {cause}")
        super().__init__("fleet_unit failed: " + "; ".join(parts))
