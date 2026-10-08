from __future__ import annotations

import math


def finite(value: float, name: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")


def positive(value: float, name: str) -> None:
    finite(value, name)
    if not value > 0.0:
        raise ValueError(f"{name} must be positive, got {value}")


def build_rectangle(
    center_x: float,
    center_y: float,
    x_length: float,
    y_length: float,
    yaw_deg: float,
) -> list[tuple[float, float]]:
    positive(x_length, "x_length")
    positive(y_length, "y_length")
    finite(yaw_deg, "yaw_deg")
    half_x = x_length / 2.0
    half_y = y_length / 2.0
    radians = math.radians(yaw_deg)
    c = math.cos(radians)
    s = math.sin(radians)
    corners = [
        (half_x, half_y),
        (half_x, -half_y),
        (-half_x, -half_y),
        (-half_x, half_y),
    ]
    out = []
    for local_x, local_y in corners:
        out_x = center_x + (local_x * c - local_y * s)
        out_y = center_y + (local_x * s + local_y * c)
        out.append((out_x, out_y))
    return out
