"""Helpers for breaking long mission legs into rolling-costmap-sized goals."""

import math
from collections.abc import Sequence


def next_navigation_segment(
    current: tuple[float, float],
    destination: tuple[float, float],
    max_segment_length_m: float,
) -> tuple[tuple[float, float], bool]:
    """Return an intermediate goal when destination exceeds the segment limit.

    The intermediate point lies along the current-to-destination vector. Nav2
    still plans an obstacle-aware path to each segment goal; this helper only
    bounds the Euclidean distance of each successive navigation request.
    """
    if not math.isfinite(max_segment_length_m) or max_segment_length_m <= 0.0:
        raise ValueError("max_segment_length_m must be a finite positive number")

    dx = destination[0] - current[0]
    dy = destination[1] - current[1]
    distance = math.hypot(dx, dy)
    if distance <= max_segment_length_m:
        return destination, False

    scale = max_segment_length_m / distance
    return (current[0] + dx * scale, current[1] + dy * scale), True


def next_path_segment(
    path: Sequence[tuple[float, float]], max_segment_length_m: float
) -> tuple[tuple[float, float], bool]:
    """Select a bounded staging target from an obstacle-aware planner path.

    Uses accumulated path distance rather than a straight-line interpolation,
    ensuring the temporary goal lies on the route returned by the planner.
    The final path pose is returned directly when the route fits in one segment.
    """
    if not math.isfinite(max_segment_length_m) or max_segment_length_m <= 0.0:
        raise ValueError("max_segment_length_m must be a finite positive number")
    if not path:
        raise ValueError("path must contain at least one pose")
    if len(path) == 1:
        return path[0], False

    distance = 0.0
    for previous, point in zip(path, path[1:]):
        segment_length = math.dist(previous, point)
        if distance + segment_length > max_segment_length_m:
            # Interpolate only on this already-planned path segment.
            fraction = (max_segment_length_m - distance) / segment_length
            return (
                previous[0] + (point[0] - previous[0]) * fraction,
                previous[1] + (point[1] - previous[1]) * fraction,
            ), True
        distance += segment_length
    return path[-1], False
