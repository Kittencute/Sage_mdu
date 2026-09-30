from __future__ import annotations


def _check_params(params: list[float], expected: int, command: str) -> None:
    if len(params) < expected:
        raise ValueError(f"{command} requires {expected} params, got {len(params)}")


def parse_nav_waypoint_params(params: list[float]) -> dict[str, float | bool]:
    _check_params(params, 6, "NAV_WAYPOINT")
    return {
        "hold_time_s": params[0] / 10.0,
        "acceptance_radius_m": params[1],
        "mission_related": params[2] == 1.0,
        "desired_yaw_deg": params[3],
        "latitude": params[4],
        "longitude": params[5],
    }


def parse_camera_image_params(params: list[float]) -> dict[str, float | int]:
    _check_params(params, 4, "CAMERA_IMAGE")
    return {
        "yaw_deg": params[0],
        "pitch_deg": params[1],
        "photo_count": int(params[2]),
        "interval_s": params[3],
    }


def parse_video_start_params(params: list[float]) -> dict[str, float | int]:
    _check_params(params, 2, "VIDEO_START_CAPTURE")
    return {
        "stream_id": int(params[0]),
        "frequency_hz": params[1],
    }


def parse_video_stop_params(params: list[float]) -> dict[str, int]:
    _check_params(params, 1, "VIDEO_STOP_CAPTURE")
    return {
        "stream_id": int(params[0]),
    }
