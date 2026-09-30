from __future__ import annotations


class RenderError(Exception):
    def __init__(
        self,
        operation: str,
        cause: object,
        target: str | None = None,
        unit: str | None = None,
    ) -> None:
        scope = " ".join(
            part
            for part in (f"target {target}" if target else "", f"unit {unit}" if unit else "")
            if part
        )
        subject = f"render {scope}" if scope else "render"
        super().__init__(f"{subject} failed: operation: {operation}; cause: {cause}")
